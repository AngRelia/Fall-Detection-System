$ErrorActionPreference = 'Stop'
$gbk  = [System.Text.Encoding]::GetEncoding(936)
$ascii = New-Object System.Text.UTF8Encoding($false)
$CRLF = "`r`n"
$R    = "D:\STM32Project\ICM42688+MMC5603 9 axis gyroscope\"
$script:ok = $true

function Rep([string]$path, [string]$label, [string]$old, [string]$new, [switch]$Ascii) {
    $enc = if ($Ascii) { $script:ascii } else { $script:gbk }
    $tt  = $enc.GetString([System.IO.File]::ReadAllBytes($path))
    $n   = ([regex]::Matches($tt, [regex]::Escape($old))).Count
    if ($n -ne 1) { Write-Output ("  FAIL {0}: matches={1}" -f $label, $n); $script:ok = $false; return }
    [System.IO.File]::WriteAllBytes($path, $enc.GetBytes($tt.Replace($old, $new)))
    Write-Output ("  ok {0}" -f $label)
}

$mc  = $R + "Core\Src\main.c"
$mf  = $R + "MDK-ARM\icode\my_flash.c"
$mh  = $R + "MDK-ARM\icode\my_flash.h"
$us  = $R + "Core\Src\usart.c"
$ioc = $R + "ICM42688+MMC5603 9 axis gyroscope.ioc"

# ================= 一、改回上电默认六轴 =================

# 1) main.c 初值
Rep $mc 'main.c 初值=六轴' `
"uint8_t mag_enabled = 1;    // 是否启用磁力计 (真值由 Flash 决定; 这里给 1 与默认九轴一致)$CRLF" `
"uint8_t mag_enabled = 0;    // 是否启用磁力计 (真值由 Flash 决定; 这里给 0 与默认六轴一致)$CRLF"

# 2) my_flash.c 默认分支
Rep $mf 'Flash 默认=六轴' `
("         * status = 1 -> 上电默认【九轴】(加速度计 + 陀螺仪 + 磁力计)。$CRLF" +
 "         *   当前选择九轴: 六轴下 Yaw 没有绝对基准, 只能靠静止自动零偏$CRLF" +
 "         *   抑制漂移; 九轴可以给出绝对航向。$CRLF" +
 "         *   短按可切回六轴, 状态写回 Flash, 下次上电保持。$CRLF" +
 "         *   用九轴前建议先长按做一次磁力计硬铁校准,$CRLF" +
 "         *   并确认轴重映射方向正确(绕竖轴转 360 度 Yaw 应单调)。$CRLF" +
 "         */$CRLF" +
 "        *x = 0.0f;$CRLF" +
 "        *y = 0.0f;$CRLF" +
 "        *z = 0.0f;$CRLF" +
 "        *status = 1; // 上电默认九轴$CRLF") `
("         * status = 0 -> 上电默认【六轴】(只用加速度计 + 陀螺仪)。$CRLF" +
 "         *   理由: 毕设要求(1)只要三轴加速度计 + 陀螺仪融合;$CRLF" +
 "         *   而九轴依赖磁力计, 受佩戴环境磁干扰影响大、标定要求高,$CRLF" +
 "         *   尚未充分验证。九轴保留为论文里的【九轴 vs 六轴】对比实验,$CRLF" +
 "         *   运行时短按即可切入, 切换后状态写回 Flash, 下次上电保持。$CRLF" +
 "         */$CRLF" +
 "        *x = 0.0f;$CRLF" +
 "        *y = 0.0f;$CRLF" +
 "        *z = 0.0f;$CRLF" +
 "        *status = 0; // 上电默认六轴$CRLF")

# 3) 换 magic, 让板子里存的 status=1 失效
Rep $mh 'magic 升到 V4' `
(" * 演进: 0xA5A5A5A5 (初版)$CRLF" +
 " *       0xA5A55A5A (引入 status 字段, 默认六轴)$CRLF" +
 " *       0xA5A5C33C (当前, 默认九轴)$CRLF" +
 " */$CRLF" +
 "#define FLASH_FLAG_VALUE_V2   0xA5A5C33CU$CRLF") `
(" * 演进: 0xA5A5A5A5 (初版)$CRLF" +
 " *       0xA5A55A5A (引入 status 字段, 默认六轴)$CRLF" +
 " *       0xA5A5C33C (默认九轴)$CRLF" +
 " *       0xA5A5C33D (当前, 改回默认六轴)$CRLF" +
 " */$CRLF" +
 "#define FLASH_FLAG_VALUE_V2   0xA5A5C33DU$CRLF")

# ================= 二、波特率 9600 -> 115200 =================

# 4) usart.c
Rep $us 'usart.c 波特率' `
("  huart2.Init.BaudRate = 9600;$CRLF") `
("  /* ⚠️ 这个值必须与工程根目录 .ioc 里的 USART2.BaudRate 保持一致!$CRLF" +
 "   * 只改这里、不改 .ioc 的话, 下次用 CubeMX 重新生成代码会把它打回 9600。$CRLF" +
 "   * 双方已同步为 115200。$CRLF" +
 "   * 时钟核算: PCLK1 = 36MHz, USARTDIV = 36e6/16/115200 = 19.53125,$CRLF" +
 "   *           BRR 只能取 M=19 / F=8 (F 仅 4 位) -> 实际 115385, 误差 +0.16%。$CRLF" +
 "   *           UART 容差 ±2~3%, 完全安全。$CRLF" +
 "   * 带宽核算: 100Hz x 32B x 10bit = 32 kbps, 占 115200 的 27.8%;$CRLF" +
 "   *           一帧 2.78ms, DMA 后台发送不阻塞主循环。 */$CRLF" +
 "  huart2.Init.BaudRate = 115200;$CRLF")

# 5) .ioc (纯 ASCII, 无 BOM)
Rep $ioc '.ioc 波特率' "USART2.BaudRate=9600" "USART2.BaudRate=115200" -Ascii

if (-not $script:ok) { Write-Output "ABORT"; exit 1 }
Write-Output "`n全部应用成功"
