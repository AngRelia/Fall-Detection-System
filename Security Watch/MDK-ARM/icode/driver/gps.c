#include "gps.h"

/* ===========================================================================
 * gps.c —— DX-GP22 / ATGM336H GNSS 模块（NMEA0183）驱动实现
 * 详细设计说明见 gps.h 头部注释与 docs/GPS模块接入方案.md
 *
 * 数据流（三级缓冲，各司其职）：
 *   USART RX ──DMA(循环)──▶ s_dma_buf ──ISR 按位置差搬运──▶ s_ring
 *        ──GPS_Process()(任务)──▶ 语句装配 ──▶ 校验/解析 ──▶ s_fix 快照
 * ========================================================================= */

/* ---------------------------------------------------------------------------
 * 可选：GNSS 电源门控（底板 ON/OFF，高电平=工作）
 * 硬件把 ON/OFF 接到 MCU 时，取消下面两行注释并改成实际引脚。
 * 不接（悬空或接高）时 GPS_Power() 是空操作，用于周期性省电实验。
 * ------------------------------------------------------------------------- */
/* #define GPS_PWR_GPIO_Port   GPIOB */
/* #define GPS_PWR_GPIO_Pin    GPIO_PIN_15 */

/* ---------------------------------------------------------------------------
 * 内部状态
 * ------------------------------------------------------------------------- */

static UART_HandleTypeDef *s_huart = NULL;

/* DMA 循环缓冲：中断里只搬字节，不解析 */
static uint8_t  s_dma_buf[GPS_RX_DMA_SIZE];
static volatile uint16_t s_dma_old_pos = 0;   /* 上一次事件处理到的绝对位置 */

/* 环形缓冲：中断 → 任务 */
static uint8_t  s_ring[GPS_RING_SIZE];
static volatile uint16_t s_ring_head = 0;
static volatile uint16_t s_ring_tail = 0;

/* NMEA 语句装配（只在任务上下文使用） */
static char     s_line[GPS_SENTENCE_MAX];
static uint16_t s_line_len = 0;
static uint8_t  s_line_bad = 0;

/* 最近一条通过校验的语句原文（不含前导 '$'），供 GPS_GetLastSentence() 显示。
   必须在切分字段之前留档 —— 切分是就地写 '\0'，会把原句改花。 */
static char     s_last[GPS_SENTENCE_MAX];

/* 最近一条**定位语句**（RMC/GGA）的原文。
   与 s_last 的区别：s_last 是"最近一条任意语句"，而 GPTXT/GSV/GSA 会频繁插进来
   （实测 GPTXT 占了 632/1478 条），所以排查字段问题时必须看这一份。 */
static char     s_last_pos[GPS_SENTENCE_MAX];

/* 各星座的"可见卫星数"（GSV）：0=GPS 1=北斗 2=GLONASS 3=其它
   为什么要分开存：GPS 与北斗各自发 GSV（$GPGSV / $GBGSV），若只用最后一句覆盖，
   会出现"参与定位 11 颗、可见却只有 6 颗"这种自相矛盾的显示。
   分开累计后再求和，VW 才是真正的可见总数。 */
static uint8_t  s_view[4];

/* 最新定位快照：seqlock（写方 = GPS 任务，读方 = 任意任务） */
static GPS_Fix_t s_fix;
static volatile uint32_t s_fix_seq = 0;

/* 统计量 */
static volatile uint32_t s_rx_bytes = 0;
static volatile uint32_t s_sentences = 0;
static volatile uint32_t s_csum_err = 0;
static volatile uint32_t s_overrun = 0;
static volatile uint32_t s_last_rx_tick = 0;

/* 语句类型普查：0=RMC 1=GGA 2=GSV 3=GSA 4=其它（见 gps.h 的 GPS_TYPE_*） */
static uint32_t s_type_cnt[5];

/* 字段切分结果（就地以 '\0' 分隔）。s_fp[0] 是语句头，所以时间/纬度字段从 1 起算。
   数组开到 24：GSV 最长（1 语句头 + 3 计数 + 4 颗星 × 4 字段 = 20 个字段），
   留足余量才能读到第 4 颗星的信噪比。 */
static char    *s_fp[24];

/* ---------------------------------------------------------------------------
 * 缓冲搬运
 * ------------------------------------------------------------------------- */

static void GPS_RingPush(uint8_t b)
{
    uint16_t next = (uint16_t)((s_ring_head + 1U) % GPS_RING_SIZE);

    if (next == s_ring_tail)
    {
        s_overrun++;                 /* 任务跟不上了：丢字节，NMEA 校验和会兜底 */
        return;
    }
    s_ring[s_ring_head] = b;
    s_ring_head = next;
    s_rx_bytes++;
}

static void GPS_RingPushBytes(const uint8_t *src, uint16_t len)
{
    uint16_t i;

    for (i = 0; i < len; i++)
    {
        GPS_RingPush(src[i]);
    }
}

/* ---------------------------------------------------------------------------
 * 小工具：整数 → 字符串（刻意不用 sprintf，避免把格式化库拉回 Flash）
 * ------------------------------------------------------------------------- */

static char GPS_HexChar(uint8_t v)
{
    v &= 0x0FU;
    return (char)((v < 10U) ? ('0' + v) : ('A' + (v - 10U)));
}

static uint8_t GPS_HexVal(char c)
{
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    return 0xFFU;
}

/* 追加字符串；pos 恒 <= buflen-1，保证有空间写结尾 '\0' */
static uint16_t GPS_PutStr(char *buf, uint16_t pos, uint16_t buflen, const char *s)
{
    while ((*s != '\0') && ((uint16_t)(pos + 1U) < buflen))
    {
        buf[pos++] = *s++;
    }
    buf[pos] = '\0';
    return pos;
}

static uint16_t GPS_PutUint(char *buf, uint16_t pos, uint16_t buflen, uint32_t v)
{
    char    tmp[11];
    uint8_t n = 0;

    if (v == 0U)
    {
        tmp[n++] = '0';
    }
    while (v > 0U)
    {
        tmp[n++] = (char)('0' + (v % 10U));
        v /= 10U;
    }
    while ((n > 0U) && ((uint16_t)(pos + 1U) < buflen))
    {
        buf[pos++] = tmp[--n];
    }
    buf[pos] = '\0';
    return pos;
}

static uint16_t GPS_PutInt(char *buf, uint16_t pos, uint16_t buflen, int32_t v)
{
    if (v < 0)
    {
        if ((uint16_t)(pos + 1U) < buflen) { buf[pos++] = '-'; buf[pos] = '\0'; }
        return GPS_PutUint(buf, pos, buflen, (uint32_t)(-(v + 1)) + 1U);
    }
    return GPS_PutUint(buf, pos, buflen, (uint32_t)v);
}

/* 度×1e7 → "d.ddddddN"，用于 OLED 显示 */
static uint16_t GPS_PutCoord(char *buf, uint16_t pos, uint16_t buflen,
                            int32_t e7, char pos_ch, char neg_ch)
{
    char     tmp[18];
    char     dig[11];
    uint16_t t = 0;
    uint8_t  dn = 0;
    uint8_t  i;
    uint32_t v = (uint32_t)((e7 < 0) ? -e7 : e7);
    uint32_t deg = v / 10000000UL;
    uint32_t frac6 = ((v % 10000000UL) + 5UL) / 10UL;   /* 7 位小数 → 6 位（四舍五入） */

    if (deg == 0U) dig[dn++] = '0';
    while (deg > 0U) { dig[dn++] = (char)('0' + (deg % 10U)); deg /= 10U; }
    while (dn > 0U) tmp[t++] = dig[--dn];               /* 整数部分 */
    tmp[t++] = '.';
    for (i = 0; i < 6U; i++)                            /* 小数部分（低位在前） */
    {
        tmp[t++] = (char)('0' + (frac6 % 10U));
        frac6 /= 10U;
    }
    for (i = 0; i < 3U; i++)                            /* 小数部分翻正 */
    {
        char c = tmp[t - 6U + i];
        tmp[t - 6U + i] = tmp[t - 1U - i];
        tmp[t - 1U - i] = c;
    }
    tmp[t++] = (e7 < 0) ? neg_ch : pos_ch;
    tmp[t] = '\0';

    return GPS_PutStr(buf, pos, buflen, tmp);
}

/* ---------------------------------------------------------------------------
 * NMEA 数值解析（全部整数运算）
 * ------------------------------------------------------------------------- */

static uint32_t GPS_ParseUint(const char *s)
{
    uint32_t v = 0;

    while (*s >= '0' && *s <= '9')
    {
        v = v * 10U + (uint32_t)(*s - '0');
        s++;
    }
    return v;
}

/* 字段长度（NMEA 字段不会超过 15 字符，u8 足够） */
static uint8_t GPS_StrLen(const char *s)
{
    uint8_t n = 0;

    while (s[n] != '\0') n++;
    return n;
}

/* ★ 定长 2 位十进制（NMEA 的 hh/mm/ss、dd/mm 都是定长 2 位）
 *
 * 为什么必须单独写一个：GPS_ParseUint() 是"吃掉所有连续数字"的通用解析器，
 * 用在定长字段上会把整串读走。实测踩到的坑（2026-10-13）：
 *   "073028.00" 经 GPS_ParseUint 得到 73028，再 s += 2 跳步 → hh=73028
 *   → 范围检查 hh>23 直接 return 0 → **UTC 永远显示 0**，而定位本身完全正常。
 *   日期 "090724" 同理（dd=90724、mm=724>12 → 日期永远是 0）。
 * 非法字符返回 0xFF，调用方的范围检查会自然拒掉。 */
static uint32_t GPS_Parse2(const char *s)
{
    if ((s[0] < '0') || (s[0] > '9') || (s[1] < '0') || (s[1] > '9'))
    {
        return 0xFFU;
    }
    return ((uint32_t)(s[0] - '0') * 10U) + (uint32_t)(s[1] - '0');
}

/* 定点小数：返回值 × 10^frac_digits，例如 ("43.5", 2) → 4350 */
static uint32_t GPS_ParseDec(const char *s, uint8_t frac_digits)
{
    uint32_t v = 0;
    uint8_t  fd = 0;

    while (*s >= '0' && *s <= '9')
    {
        v = v * 10U + (uint32_t)(*s - '0');
        s++;
    }
    if (*s == '.')
    {
        s++;
        while ((*s >= '0') && (*s <= '9') && (fd < frac_digits))
        {
            v = v * 10U + (uint32_t)(*s - '0');
            fd++;
            s++;
        }
    }
    while (fd < frac_digits) { v *= 10U; fd++; }
    return v;
}

static int32_t GPS_ParseSignedDec(const char *s, uint8_t frac_digits)
{
    if (*s == '-') return -(int32_t)GPS_ParseDec(s + 1, frac_digits);
    if (*s == '+') return (int32_t)GPS_ParseDec(s + 1, frac_digits);
    return (int32_t)GPS_ParseDec(s, frac_digits);
}

/* "hhmmss.sss" → UTC 当日毫秒 */
static uint32_t GPS_ParseUtcMs(const char *s)
{
    uint32_t hh, mm, ss;
    uint32_t frac = 0;
    uint8_t  fd = 0;

    /* 定长字段：先确认长度，后面才能安全地按 s += 2 跳步 */
    if (GPS_StrLen(s) < 6U) return 0U;
    hh = GPS_Parse2(s);
    s += 2;
    mm = GPS_Parse2(s);
    s += 2;
    ss = GPS_Parse2(s);
    s += 2;
    if (*s == '.')
    {
        s++;
        while ((*s >= '0') && (*s <= '9') && (fd < 3U))
        {
            frac = frac * 10U + (uint32_t)(*s - '0');
            fd++;
            s++;
        }
    }
    while (fd < 3U) { frac *= 10U; fd++; }

    if (hh > 23U || mm > 59U || ss > 60U) return 0U;
    return hh * 3600000UL + mm * 60000UL + ss * 1000UL + frac;
}

/* "ddmmyy" → yyyymmdd（20xx 世纪，够本课题用） */
static uint32_t GPS_ParseDate(const char *s)
{
    uint32_t dd, mm, yy;

    if (GPS_StrLen(s) < 6U) return 0U;
    dd = GPS_Parse2(s);  s += 2;
    mm = GPS_Parse2(s);  s += 2;
    yy = GPS_ParseUint(s);      /* 最后一段，用通用解析器才是对的 */

    if (dd == 0U || mm == 0U || mm > 12U) return 0U;
    return (2000UL + yy) * 10000UL + mm * 100UL + dd;
}

/* "2236.40101" → 度×1e7；neg=1 表示南纬/西经
 *
 * 换算：度度分分.分分分分 → deg + min/60
 *   mm_e5 为"分"的 1e-5 单位，1 单位 = 1e-5/60 度 = 1.6667e-7 度 ≈ (5/3)e-7 度
 *   故 e7 = deg*1e7 + (mm_e5*5 + 1)/3，全程整数，误差 < 4 mm */
static int32_t GPS_ParseCoordE7(const char *s, uint8_t neg)
{
    const char *dot = s;
    uint8_t int_digits = 0;
    uint8_t i, deg_digits;
    uint32_t deg = 0;
    uint32_t min_int = 0;
    uint32_t frac = 0;
    uint8_t  fd = 0;
    uint32_t mm_e5;
    int32_t  e7;

    while ((*dot != '\0') && (*dot != '.'))
    {
        int_digits++;
        dot++;
    }
    if (int_digits < 4U) return 0;              /* 度至少 2 位 + 分 2 位 */

    deg_digits = (uint8_t)(int_digits - 2U);    /* 分的整数部分恒为 2 位 */
    for (i = 0; i < deg_digits; i++)
    {
        if (s[i] < '0' || s[i] > '9') return 0;
        deg = deg * 10U + (uint32_t)(s[i] - '0');
    }
    for (; i < int_digits; i++)
    {
        if (s[i] < '0' || s[i] > '9') return 0;
        min_int = min_int * 10U + (uint32_t)(s[i] - '0');
    }
    if (min_int > 59U) return 0;

    if (*dot == '.')
    {
        const char *q = dot + 1;
        while ((*q >= '0') && (*q <= '9') && (fd < 5U))
        {
            frac = frac * 10U + (uint32_t)(*q - '0');
            fd++;
            q++;
        }
    }
    while (fd < 5U) { frac *= 10U; fd++; }

    mm_e5 = min_int * 100000UL + frac;                    /* ≤ 5,999,999 */
    e7 = (int32_t)(deg * 10000000UL + (mm_e5 * 5UL + 1UL) / 3UL);

    return neg ? -e7 : e7;
}

/* ---------------------------------------------------------------------------
 * 快照读写（seqlock）
 * ------------------------------------------------------------------------- */

static void GPS_FixCopy(GPS_Fix_t *out)
{
    uint32_t s1, s2;
    uint8_t  tries;

    for (tries = 0; tries < 8U; tries++)
    {
        s1 = s_fix_seq;
        if ((s1 & 1U) != 0U) continue;      /* 写方持有，重试 */
        *out = s_fix;
        s2 = s_fix_seq;
        if (s1 == s2) return;
    }
    *out = s_fix;                            /* 退化：极小概率混帧，可接受 */
}

static void GPS_FixCommit(const GPS_Fix_t *f)
{
    s_fix_seq++;                             /* 奇数：写入中 */
    s_fix = *f;
    /* 定位是否有效：**RMC 或 GGA 任一为"有效"即可**，不能要求两者都来。
       踩过的坑（2026-10-13 实测）：原来写成 (has_rmc && status && quality>0)，
       结果模块只输出 GGA 时（很多模块的精简配置就是只留 GGA+RMC 之一，
       用户也可能在 PC 上用 $PCAS03 关掉 RMC），即使 GGA 已给出完全正确的经纬度
       （实测杭州 30.1877N/120.8166E、HDOP 4.0、海拔 4 m），
       valid 也永远是 0 → 屏幕一直 FIX NO、告警报文一直 v=0。 */
    s_fix.valid = (uint8_t)(((f->has_rmc && f->status) ||
                             (f->has_gga && (f->quality > 0U))) ? 1U : 0U);
    s_fix_seq++;                             /* 偶数：写完 */
}

/* ---------------------------------------------------------------------------
 * 语句装配与解析
 * ------------------------------------------------------------------------- */

/* 就地按 ',' 切分（调用前已截掉 '*' 之后），返回字段数。
   上限 23+1=24：GSV 有 20 个字段，必须全切出来才能读第 4 颗星的信噪比。 */
static uint8_t GPS_SplitFields(char *s)
{
    uint8_t n = 0;
    char   *start = s;

    while ((*s != '\0') && (n < 23U))
    {
        if (*s == ',')
        {
            *s = '\0';
            s_fp[n] = start;
            n++;
            start = s + 1;
        }
        s++;
    }
    s_fp[n] = start;
    n++;
    return n;
}

/* 语句头匹配：field 形如 "GNRMC"/"GPRMC"/"BDGGA"，只看后 3 个字符 */
static uint8_t GPS_TypeIs(const char *field, const char *type3)
{
    uint8_t len = 0;

    while (field[len] != '\0') len++;
    if (len < 3U) return 0;
    return (uint8_t)((field[len - 3U] == type3[0]) &&
                     (field[len - 2U] == type3[1]) &&
                     (field[len - 1U] == type3[2]));
}

/* 校验和：'$' 与 '*' 之间所有字符异或 */
static uint8_t GPS_CheckSumOk(const char *s, uint16_t len)
{
    uint16_t i;
    uint16_t star = 0xFFFFU;
    uint8_t  cs = 0;
    uint8_t  hi, lo, want;

    for (i = 0; i < len; i++)
    {
        if (s[i] == '*') { star = i; break; }
        cs ^= (uint8_t)s[i];
    }
    if (star == 0xFFFFU) return 0;
    if ((uint16_t)(star + 2U) >= len) return 0;

    hi = GPS_HexVal(s[star + 1U]);
    lo = GPS_HexVal(s[star + 2U]);
    if (hi == 0xFFU || lo == 0xFFU) return 0;
    want = (uint8_t)((hi << 4) | lo);

    return (uint8_t)(want == cs);
}

/* $--RMC,UTC,status,lat,N,lon,E,spd,cog,date,mv,mvE,mode,nav*CS
 * 注意：s_fp[0] 是语句头（"GNRMC"），所以字段下标从 1 起算
 *
 * ★ "只保留最后已知的**有效**定位"（2026-10-14 实测修正）
 *   模块丢星后仍在 1 Hz 输出 RMC/GGA，但 `status=V`、经纬度字段为空。
 *   若把这种帧也照单全收，就会出现："室内 v=0，但 lat/lon 被刷成 0，
 *   且 fix_age 一直是几百毫秒" —— 与设计意图（v=0 时仍带**最后已知位置**，
 *   并给出它**有多旧**）正好相反。实测报文曾出现
 *   {"v":0,"lat":0,"lon":0,"hdop":2550,"fix_age":266} 就是这个原因。
 *   所以：**只有有效定位（status=A）才更新位置/时间并刷新 stamp_ms**；
 *   无效帧只更新"当前是否有效"的标志，位置保持上一次有效值不动。 */
static void GPS_ParseRMC(uint8_t nf)
{
    GPS_Fix_t f;
    uint8_t   ok;

    if (nf < 10U) return;

    GPS_FixCopy(&f);
    ok = (uint8_t)((s_fp[2][0] == 'A') ? 1U : 0U);

    f.status  = ok;
    f.has_rmc = 1U;

    if (ok != 0U)
    {
        f.utc_ms        = GPS_ParseUtcMs(s_fp[1]);
        f.lat_e7        = GPS_ParseCoordE7(s_fp[3], (uint8_t)((s_fp[4][0] == 'S') ? 1U : 0U));
        f.lon_e7        = GPS_ParseCoordE7(s_fp[5], (uint8_t)((s_fp[6][0] == 'W') ? 1U : 0U));
        /* 节 → cm/s：1 kn = 51.4444 cm/s，输入按 0.01 kn 解析 */
        f.speed_cms     = (uint16_t)((GPS_ParseDec(s_fp[7], 2) * 5144UL) / 10000UL);
        f.course_cdeg   = (uint16_t)GPS_ParseDec(s_fp[8], 2);
        f.date_yyyymmdd = GPS_ParseDate(s_fp[9]);
        f.stamp_ms      = HAL_GetTick();   /* 只在这时刷新"最后已知位置"的时刻 */
        f.fix_ever      = 1U;
    }

    GPS_FixCommit(&f);
}

/* $--GSV,totalMsgs,msgNum,satsInView,prn,elev,azim,snr,...*CS
 *
 * 定位用不到 GSV，但**联调时它是最重要的语句**：模块在无源（陶瓷）天线下会一直报
 * "$GPTXT,...,ANTENNA OPEN"，那条说明不了天线好坏；而 GSV 里的"可见卫星数"和
 * "各卫星信噪比(dBHz)"才真正反映天线/射频有没有在收信号。
 *   · 可见 0 颗、信噪比全 0  → 射频侧确实没信号（天线坏 / 完全在室内 / 屏蔽）
 *   · 可见若干颗、信噪比 20~45 → 天线在工作，只是还没凑够定位条件（等或换位置）
 *
 * 字段：[1]=总语句数 [2]=本句序号 [3]=可见卫星总数
 *       之后每 4 个字段一颗星：prn,elev,az,snr（最多 4 颗/句）
 * 只在 msgNum==1 时刷新"可见总数"并重置本轮最强信噪比，避免多句互相覆盖。 */
static void GPS_ParseGSV(uint8_t nf)
{
    GPS_Fix_t f;
    uint8_t   i;
    uint8_t   mx = 0U;
    uint8_t   idx = 3U;             /* 星座编号：0=GPS 1=北斗 2=GLONASS 3=其它 */
    uint16_t  sum;

    if (nf < 4U) return;

    /* 按语句头区分星座：$GPGSV / $GBGSV(北斗) / $GLGSV / $GAGSV */
    if ((s_fp[0][0] == 'G') && (s_fp[0][1] == 'P'))      idx = 0U;
    else if ((s_fp[0][0] == 'B') || ((s_fp[0][0] == 'G') && (s_fp[0][1] == 'B'))) idx = 1U;
    else if ((s_fp[0][0] == 'G') && (s_fp[0][1] == 'L')) idx = 2U;

    /* 本句 4 个卫星槽的 snr 落在 [7] [11] [15] [19] */
    for (i = 7U; (i < nf) && (i <= 19U); i += 4U)
    {
        uint8_t snr = (uint8_t)GPS_ParseUint(s_fp[i]);
        if (snr > mx)
        {
            mx = snr;
        }
    }

    GPS_FixCopy(&f);
    if (s_fp[2][0] == '1')          /* 该星座新一轮 GSV 的第 1 句 */
    {
        s_view[idx] = (uint8_t)GPS_ParseUint(s_fp[3]);
        /* 以 GPS 的 GSV 起点作为"一整轮"的刷新点：最强信噪比在这一轮内取最大 */
        if (idx == 0U)
        {
            f.snr_max = mx;
        }
        else if (mx > f.snr_max)
        {
            f.snr_max = mx;
        }
    }
    else if (mx > f.snr_max)
    {
        f.snr_max = mx;
    }

    /* 可见卫星总数 = 各星座之和（>99 顶格，屏幕只有 2 位） */
    sum = (uint16_t)s_view[0] + (uint16_t)s_view[1] + (uint16_t)s_view[2] + (uint16_t)s_view[3];
    f.sats_view = (sum > 99U) ? 99U : (uint8_t)sum;

    /* 注意：GSV 不是定位数据，**不更新 stamp_ms / has_* / quality**，
       否则会把"最后一帧定位的时间"刷新成 GSV 到达的时间，fix_age 就失真了。 */
    GPS_FixCommit(&f);
}

/* $--GGA,UTC,lat,N,lon,E,quality,sats,hdop,alt,M,geoid,M,age,stn*CS
 * 同样：s_fp[0] 是语句头（"GNGGA"），字段下标从 1 起算。
 *
 * ★ 与 RMC 同一原则：**只有 quality>0（有效定位）才更新位置/海拔/HDOP/时刻**。
 *   丢星时模块会继续发 quality=0 的 GGA，其 hdop 常是 25.50 之类的占位值
 *   （实测报文里出现过 "hdop":2550）—— 收下它就把上一次有效定位的结果毁了。 */
static void GPS_ParseGGA(uint8_t nf)
{
    GPS_Fix_t f;
    uint8_t   quality;

    if (nf < 10U) return;

    GPS_FixCopy(&f);
    quality = (uint8_t)GPS_ParseUint(s_fp[6]);

    f.quality = quality;
    f.has_gga = 1U;

    if (quality > 0U)
    {
        f.utc_ms    = GPS_ParseUtcMs(s_fp[1]);
        f.lat_e7    = GPS_ParseCoordE7(s_fp[2], (uint8_t)((s_fp[3][0] == 'S') ? 1U : 0U));
        f.lon_e7    = GPS_ParseCoordE7(s_fp[4], (uint8_t)((s_fp[5][0] == 'W') ? 1U : 0U));
        f.sats      = (uint8_t)GPS_ParseUint(s_fp[7]);
        f.hdop_x100 = (uint16_t)GPS_ParseDec(s_fp[8], 2);
        f.alt_cm    = GPS_ParseSignedDec(s_fp[9], 2);
        f.stamp_ms  = HAL_GetTick();
        f.fix_ever  = 1U;
    }

    GPS_FixCommit(&f);
}

/* 一条完整语句（不含 '$'，含 "*CS"） */
static void GPS_ParseLine(char *line, uint16_t len)
{
    uint16_t i;
    uint8_t  nf;

    if (!GPS_CheckSumOk(line, len))
    {
        s_csum_err++;
        return;
    }
    s_sentences++;

    /* 先留档原文（下面的切分会就地写 '\0'，把原句改花） */
    if (len > (GPS_SENTENCE_MAX - 1U))
    {
        len = (GPS_SENTENCE_MAX - 1U);
    }
    for (i = 0; i < len; i++)
    {
        s_last[i] = line[i];
    }
    s_last[len] = '\0';

    /* 定位语句（RMC/GGA）另外再留一份原文，便于排查字段问题 */
    if (GPS_TypeIs(s_fp[0], "RMC") || GPS_TypeIs(s_fp[0], "GGA"))
    {
        for (i = 0; i < len; i++)
        {
            s_last_pos[i] = line[i];
        }
        s_last_pos[len] = '\0';
    }

    for (i = 0; i < len; i++)                /* 截掉 '*' 及其后 */
    {
        if (line[i] == '*') { line[i] = '\0'; break; }
    }

    nf = GPS_SplitFields(line);
    if (nf < 2U) return;

    if (GPS_TypeIs(s_fp[0], "RMC"))       { s_type_cnt[GPS_TYPE_RMC]++; GPS_ParseRMC(nf); }
    else if (GPS_TypeIs(s_fp[0], "GGA"))  { s_type_cnt[GPS_TYPE_GGA]++; GPS_ParseGGA(nf); }
    else if (GPS_TypeIs(s_fp[0], "GSV"))  { s_type_cnt[GPS_TYPE_GSV]++; GPS_ParseGSV(nf); }
    else if (GPS_TypeIs(s_fp[0], "GSA"))  { s_type_cnt[GPS_TYPE_GSA]++; }
    else                                  { s_type_cnt[GPS_TYPE_OTH]++; }
    /* VTG/ZDA/TXT 等归入"其它"，本课题不做处理，只计数（便于发现"模块根本没吐定位语句"） */
}

/* 逐字节装配：'$' 开始，'\n' 结束 */
static void GPS_FeedByte(uint8_t c)
{
    if (c == '$')
    {
        s_line_len = 0;
        s_line_bad = 0;
        return;
    }
    if (c == '\r') return;                   /* CRLF 的 CR 丢弃 */
    if (c == '\n')
    {
        if ((s_line_bad == 0U) && (s_line_len > 6U))
        {
            s_line[s_line_len] = '\0';
            GPS_ParseLine(s_line, s_line_len);
        }
        s_line_len = 0;
        s_line_bad = 0;
        return;
    }
    if (s_line_bad != 0U) return;            /* 本句已废，等下一个 '$' */
    if ((c < 0x20U) || (c > 0x7EU)) return;  /* 不可打印噪声 */
    if (s_line_len >= (GPS_SENTENCE_MAX - 1U))
    {
        s_line_bad = 1U;
        s_line_len = 0;
        return;
    }
    s_line[s_line_len++] = (char)c;
}

/* ---------------------------------------------------------------------------
 * 对外接口：生命周期
 * ------------------------------------------------------------------------- */

void GPS_Init(UART_HandleTypeDef *huart)
{
    if (huart == NULL) return;

    s_huart     = huart;
    s_dma_old_pos = 0;
    s_ring_head = 0;
    s_ring_tail = 0;
    s_line_len  = 0;
    s_line_bad  = 0;

    /* "最后已知位置"的基准时刻：开机时先填上当前 tick，避免刚开机就报出
       一个巨大的 fix_age（stamp_ms 若为 0，age 会等于开机时长）。 */
    s_fix.stamp_ms = HAL_GetTick();
    s_fix.fix_ever = 0U;
    s_fix_seq = 0U;

    /* 循环 DMA + 空闲中断：启动一次即可，HAL 每次事件回调给出绝对位置 */
    (void)HAL_UARTEx_ReceiveToIdle_DMA(huart, s_dma_buf, GPS_RX_DMA_SIZE);
}

void GPS_Reinit(void)
{
    if (s_huart == NULL) return;

    (void)HAL_UART_AbortReceive(s_huart);
    s_dma_old_pos = 0;
    s_ring_head = 0;
    s_ring_tail = 0;
    s_line_len  = 0;
    s_line_bad  = 0;
    (void)HAL_UARTEx_ReceiveToIdle_DMA(s_huart, s_dma_buf, GPS_RX_DMA_SIZE);
}

void GPS_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == NULL || huart != s_huart) return;

    if (Size > s_dma_old_pos)
    {
        GPS_RingPushBytes(&s_dma_buf[s_dma_old_pos], (uint16_t)(Size - s_dma_old_pos));
    }
    else if (Size < s_dma_old_pos)
    {
        /* 环形回卷：先搬老位置到缓冲末尾，再搬 0 到新位置 */
        GPS_RingPushBytes(&s_dma_buf[s_dma_old_pos], (uint16_t)(GPS_RX_DMA_SIZE - s_dma_old_pos));
        GPS_RingPushBytes(&s_dma_buf[0], Size);
    }
    else
    {
        /* 位置没变：连续两次事件之间没有新数据，忽略 */
    }

    s_dma_old_pos = (uint16_t)((Size >= GPS_RX_DMA_SIZE) ? 0U : Size);
    s_last_rx_tick = HAL_GetTick();
}

void GPS_Process(void)
{
    while (s_ring_tail != s_ring_head)
    {
        uint8_t c = s_ring[s_ring_tail];
        s_ring_tail = (uint16_t)((s_ring_tail + 1U) % GPS_RING_SIZE);
        GPS_FeedByte(c);
    }
}

void GPS_FeedBytes(const uint8_t *data, uint16_t len)
{
    uint16_t i;

    if (data == NULL) return;
    for (i = 0; i < len; i++)
    {
        GPS_FeedByte(data[i]);
        s_rx_bytes++;
    }
}

/* ---------------------------------------------------------------------------
 * 对外接口：读取
 * ------------------------------------------------------------------------- */

uint8_t GPS_GetFix(GPS_Fix_t *out)
{
    uint32_t s1, s2;
    uint8_t  tries;

    if (out == NULL) return 0U;

    for (tries = 0; tries < 8U; tries++)
    {
        s1 = s_fix_seq;
        if ((s1 & 1U) != 0U) continue;
        *out = s_fix;
        s2 = s_fix_seq;
        if (s1 == s2) return 1U;
    }
    return 0U;
}

uint8_t GPS_IsFixFresh(uint32_t max_age_ms, uint32_t now_ms)
{
    GPS_Fix_t f;

    if (!GPS_GetFix(&f)) return 0U;
    if (f.valid == 0U) return 0U;
    if ((uint32_t)(now_ms - f.stamp_ms) > max_age_ms) return 0U;
    return 1U;
}

uint16_t GPS_BuildJson(char *buf, uint16_t buflen, uint32_t now_ms)
{
    GPS_Fix_t f;
    uint16_t  n = 0;
    uint32_t  age;

    if (buf == NULL || buflen < 8U) return 0U;

    if (!GPS_GetFix(&f))
    {
        f.lat_e7 = 0; f.lon_e7 = 0; f.valid = 0; f.quality = 0; f.sats = 0;
        f.hdop_x100 = 0; f.alt_cm = 0; f.speed_cms = 0; f.course_cdeg = 0;
        f.stamp_ms = now_ms;
        f.fix_ever = 0U;
    }
    /* fix_age 的语义（务必与云端约定一致）：
         fix_ever = 1 → 距**最后一次有效定位**的毫秒数（室内丢星后它会持续增长）；
         fix_ever = 0 → 从未定位成功，用 -1 表达，避免与"刚刚定位过（0）"混淆。 */
    age = (uint32_t)(now_ms - f.stamp_ms);

    n = GPS_PutStr(buf, n, buflen, "{\"v\":");
    n = GPS_PutUint(buf, n, buflen, f.valid);
    n = GPS_PutStr(buf, n, buflen, ",\"lat\":");
    n = GPS_PutInt(buf, n, buflen, f.lat_e7);
    n = GPS_PutStr(buf, n, buflen, ",\"lon\":");
    n = GPS_PutInt(buf, n, buflen, f.lon_e7);
    n = GPS_PutStr(buf, n, buflen, ",\"q\":");
    n = GPS_PutUint(buf, n, buflen, f.quality);
    n = GPS_PutStr(buf, n, buflen, ",\"sat\":");
    n = GPS_PutUint(buf, n, buflen, f.sats);
    n = GPS_PutStr(buf, n, buflen, ",\"hdop\":");
    n = GPS_PutUint(buf, n, buflen, f.hdop_x100);
    n = GPS_PutStr(buf, n, buflen, ",\"alt\":");
    n = GPS_PutInt(buf, n, buflen, f.alt_cm);
    n = GPS_PutStr(buf, n, buflen, ",\"spd\":");
    n = GPS_PutUint(buf, n, buflen, f.speed_cms);
    n = GPS_PutStr(buf, n, buflen, ",\"cog\":");
    n = GPS_PutUint(buf, n, buflen, f.course_cdeg);
    n = GPS_PutStr(buf, n, buflen, ",\"fix_age\":");
    if (f.fix_ever != 0U)
    {
        n = GPS_PutUint(buf, n, buflen, age);
    }
    else
    {
        n = GPS_PutInt(buf, n, buflen, -1);   /* -1 = 从未定位成功 */
    }
    n = GPS_PutStr(buf, n, buflen, "}");

    return n;
}

/* 最近一条通过校验的语句原文（含前导 '$'），给 GPS 接收页显示用 */
void GPS_GetLastSentence(char *buf, uint16_t buflen)
{
    uint16_t n = 0;

    if (buf == NULL || buflen < 4U)
    {
        if (buf != NULL && buflen > 0U) buf[0] = '\0';
        return;
    }

    buf[n++] = '$';
    while ((s_last[n - 1U] != '\0') && (n < (uint16_t)(buflen - 1U)))
    {
        buf[n] = s_last[n - 1U];
        n++;
    }
    buf[n] = '\0';
}

/* 最近一条**定位语句**（RMC/GGA）原文（含前导 '$'）。
   排查字段类问题（比如某个字段解析不出来）时看它 —— 它是"经纬度/时间/卫星数"的来源，
   不会被频发的 GPTXT/GSV/GSA 挤掉。没收到过则写入空串。 */
void GPS_GetLastPosSentence(char *buf, uint16_t buflen)
{
    uint16_t n = 0;

    if (buf == NULL || buflen < 4U)
    {
        if (buf != NULL && buflen > 0U) buf[0] = '\0';
        return;
    }

    buf[n++] = '$';
    while ((s_last_pos[n - 1U] != '\0') && (n < (uint16_t)(buflen - 1U)))
    {
        buf[n] = s_last_pos[n - 1U];
        n++;
    }
    buf[n] = '\0';
}

uint16_t GPS_FormatPosition(char *buf, uint16_t buflen, uint32_t now_ms)
{
    GPS_Fix_t f;
    uint16_t  n = 0;

    if (buf == NULL || buflen < 24U)
    {
        if (buf != NULL && buflen > 0U) buf[0] = '\0';
        return 0U;
    }

    if (!GPS_GetFix(&f) || (f.valid == 0U))
    {
        return GPS_PutStr(buf, n, buflen, "NO GPS FIX");
    }

    n = GPS_PutCoord(buf, n, buflen, f.lat_e7, 'N', 'S');
    n = GPS_PutStr(buf, n, buflen, " ");
    n = GPS_PutCoord(buf, n, buflen, f.lon_e7, 'E', 'W');

    (void)now_ms;
    return n;
}

/* ---------------------------------------------------------------------------
 * 对外接口：$PCA 配置命令
 * ------------------------------------------------------------------------- */

void GPS_SendCmd(const char *body)
{
    char     out[GPS_CMD_MAX];
    uint16_t i = 0;
    uint16_t n = 0;
    uint8_t  cs = 0;

    if (s_huart == NULL || body == NULL) return;

    out[n++] = '$';
    while ((body[i] != '\0') && (n < (GPS_CMD_MAX - 5U)))
    {
        out[n++] = body[i];
        cs ^= (uint8_t)body[i];
        i++;
    }
    out[n++] = '*';
    out[n++] = GPS_HexChar((uint8_t)(cs >> 4));
    out[n++] = GPS_HexChar(cs);
    out[n++] = '\r';
    out[n++] = '\n';

    (void)HAL_UART_Transmit(s_huart, (uint8_t *)out, n, 200U);
}

void GPS_ConfigMinimal(void)
{
    /* 只保留 GGA + GSV + RMC：
     *   GGA 提供定位质量/卫星数/HDOP/海拔，RMC 提供经纬度/速度/航向/日期，
     *   **GSV 提供可见卫星数与各星信噪比** —— 联调时判断"天线/视野"就靠它，
     *   所以精简输出时也要留着（默认约 3 s 一轮，流量很小）。
     * 参数顺序：nGGA,nGLL,nGSA,nGSV,nRMC,nVTG,nZDA,nTXT,nDHV,Res1,Res2,Res3,Res4,nGST
     * （与厂家例程注释里的 $PCAS03,0,0,0,0,1,0,0,0,0,0,,,1,0 同一形状）
     * ⚠ 上板前先用 PC 端 sscom 实测确认模块接受该命令，再固化。 */
    GPS_SendCmd("PCAS03,1,0,0,1,1,0,0,0,0,0,,,0,0");

    /* 需要时按需打开（括号内为出厂默认值，一般不必发）：
     *   GPS_SendCmd("PCAS02,1000");   更新率 1 Hz（默认）
     *   GPS_SendCmd("PCAS04,3");      卫星系统 GPS+BDS（默认）
     *   GPS_SendCmd("PCAS01,1");      波特率 9600（默认）
     *   GPS_SendCmd("PCAS10,0");      热启动（保留备电数据，启动最快） */
}

void GPS_ConfigAll(void)
{
    /* 恢复"全部语句打开"（等价于出厂默认输出集合）。
       用途：若模块被之前的 PC 端试验用 $PCAS03 关掉了 GGA/RMC/GSV，
       屏幕上会只看到 $GPTXT 之类零星语句、经纬度永远出不来 —— 发这一条即可救回。
       原文来自厂家《串口UART应用指导》5.3 节的全开示例，校验和运行时算。 */
    GPS_SendCmd("PCAS03,1,1,1,1,1,1,1,1,1,1,,,1,1");
}

void GPS_Power(uint8_t on)
{
#if defined(GPS_PWR_GPIO_Port) && defined(GPS_PWR_GPIO_Pin)
    /* ON/OFF 高电平=工作、低电平=关断 */
    HAL_GPIO_WritePin(GPS_PWR_GPIO_Port, GPS_PWR_GPIO_Pin,
                      (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    if (on != 0U) GPS_Reinit();
#else
    (void)on;   /* 硬件未接 ON/OFF：空操作 */
#endif
}

/* ---------------------------------------------------------------------------
 * 对外接口：诊断量
 * ------------------------------------------------------------------------- */

uint32_t GPS_GetRxBytes(void)       { return s_rx_bytes; }
uint32_t GPS_GetSentenceCount(void) { return s_sentences; }
uint32_t GPS_GetCsumErrCount(void)  { return s_csum_err; }
uint32_t GPS_GetOverrunCount(void)  { return s_overrun; }

uint32_t GPS_GetTypeCount(uint8_t type)
{
    if (type > GPS_TYPE_OTH)
    {
        return 0U;
    }
    return s_type_cnt[type];
}

uint32_t GPS_GetLastRxAge(uint32_t now_ms)
{
    if (s_last_rx_tick == 0U) return 0xFFFFFFFFUL;
    return (uint32_t)(now_ms - s_last_rx_tick);
}

uint8_t GPS_IsAlive(uint32_t now_ms)
{
    return (uint8_t)((GPS_GetLastRxAge(now_ms) < 5000UL) ? 1U : 0U);
}
