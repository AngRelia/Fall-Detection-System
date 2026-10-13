# =============================================================================
#  rebuild_keil_project.ps1
#  用途：把 Smart Home 的 Keil 工程改写为 Security Watch：
#        改项目名/输出名/输出目录、把 icode 分组拆成 icode\driver 与 icode\task、
#        追加任务源文件、修正头文件搜索路径。
#  特点：分组与文件清单从现有工程读取后重建，GroupNumber/FileNumber 连续无冲突。
#  用法：pwsh -File rebuild_keil_project.ps1
# =============================================================================
$ErrorActionPreference = 'Stop'

$root   = 'D:\同学们的毕设\江彬锋的毕设\Security Watch'
$mdk    = Join-Path $root 'MDK-ARM'
$oldName = 'Smart Home'
$newName = 'Security Watch'
$utf8NoBom = New-Object System.Text.UTF8Encoding($false)
$gbk = [System.Text.Encoding]::GetEncoding(936)

# 保留的底层驱动（按依赖顺序）
$drivers = @('delay','OLED','OLED_Data','key','led','beep','ws2812b')
# 新建的任务文件
$tasks   = @('imu_recv_task','tinyml_task','fatigue_task','alarm_task','ble_task','oled_task')

# --- 1. 读取 ----------------------------------------------------------------
$prjPath = Join-Path $mdk "$oldName.uvprojx"
$optPath = Join-Path $mdk "$oldName.uvoptx"
$prj = [System.IO.File]::ReadAllText($prjPath, $utf8NoBom)
$opt = [System.IO.File]::ReadAllText($optPath, $gbk)
Write-Host "[1/6] 已读取 uvprojx ($($prj.Length) 字符) 与 uvoptx ($($opt.Length) 字符)"

# --- 2. 改项目名 / 输出 ------------------------------------------------------
$prj = $prj.Replace("<TargetName>$oldName</TargetName>", "<TargetName>$newName</TargetName>")
$prj = $prj.Replace("<OutputDirectory>$oldName\</OutputDirectory>", "<OutputDirectory>$newName\</OutputDirectory>")
$prj = $prj.Replace("<OutputName>$oldName</OutputName>", "<OutputName>$newName</OutputName>")
$opt = $opt.Replace("<TargetName>$oldName</TargetName>", "<TargetName>$newName</TargetName>")
Write-Host "[2/6] 已改项目名与输出名 -> $newName"

# --- 3. 头文件搜索路径 ------------------------------------------------------
$oldInc = './icode;'
$newInc = './icode/driver;./icode/task;'
if ($prj.Contains($oldInc)) {
    $prj = $prj.Replace($oldInc, $newInc)
    Write-Host "[3/6] 头文件路径已更新为 ./icode/driver 与 ./icode/task"
} elseif ($prj.Contains('./icode/driver')) {
    Write-Host "[3/6] 头文件路径已是新形式，跳过"
} else {
    throw "未找到 ./icode; 头文件路径，工程结构可能与预期不符"
}

# --- 4. 重建 uvprojx 的 icode 分组 -----------------------------------------
function New-ProjGroup {
    param([string]$GroupName, [string]$SubDir, [string[]]$Names)
    $sb = New-Object System.Text.StringBuilder
    [void]$sb.Append("<Group>`r`n          <GroupName>$GroupName</GroupName>`r`n          <Files>`r`n")
    foreach ($n in $Names) {
        foreach ($e in @(@('c','1'), @('h','5'))) {
            $file = "$n.$($e[0])"
            [void]$sb.Append("            <File>`r`n")
            [void]$sb.Append("              <FileName>$file</FileName>`r`n")
            [void]$sb.Append("              <FileType>$($e[1])</FileType>`r`n")
            [void]$sb.Append("              <FilePath>.\icode\$SubDir\$file</FilePath>`r`n")
            [void]$sb.Append("            </File>`r`n")
        }
    }
    [void]$sb.Append("          </Files>`r`n        </Group>")
    return $sb.ToString()
}

$newProjGroups = (New-ProjGroup -GroupName 'icode/driver' -SubDir 'driver' -Names $drivers) +
                 "`r`n        " +
                 (New-ProjGroup -GroupName 'icode/task'   -SubDir 'task'   -Names $tasks)

$m = [regex]::Match($prj, '(?s)<Group>\s*<GroupName>icode</GroupName>.*?</Group>')
if (-not $m.Success) { throw "在 uvprojx 中未找到 icode 分组" }
$prj = $prj.Remove($m.Index, $m.Length).Insert($m.Index, $newProjGroups)
Write-Host "[4/6] uvprojx icode 分组已拆分为 icode/driver + icode/task"

# --- 5. 重建 uvoptx 的 icode 分组 -----------------------------------------
# 解析现有 icode 分组里的文件条目，保留其原有属性
$oldGroups = [regex]::Matches($opt, '(?s)<Group>.*?</Group>')
$groupNames = @()
foreach ($g in $oldGroups) {
    $gm = [regex]::Match($g.Value, '<GroupName>([^<]*)</GroupName>')
    $groupNames += $gm.Groups[1].Value
}
$icodeIdx = [array]::IndexOf($groupNames, 'icode')
if ($icodeIdx -lt 0) { throw "在 uvoptx 中未找到 icode 分组" }

$oldIcodeBlock = $oldGroups[$icodeIdx].Value
$oldFileBlocks = [regex]::Matches($oldIcodeBlock, '(?s)<File>.*?</File>')

# 按"去扩展名的文件名"索引原有 File 块，保留 tvExp / RteFlg / bShared 等设置
$byStem = @{}
foreach ($fb in $oldFileBlocks) {
    $pn = [regex]::Match($fb.Value, '<PathWithFileName>([^<]*)</PathWithFileName>').Groups[1].Value
    $stem = [System.IO.Path]::GetFileNameWithoutExtension($pn)
    if (-not $byStem.ContainsKey($stem)) { $byStem[$stem] = @{} }
    $ext = [System.IO.Path]::GetExtension($pn).TrimStart('.')
    $byStem[$stem][$ext] = $fb.Value
}

# 现有最大 FileNumber
$maxFileNo = 0
foreach ($mm in [regex]::Matches($opt, '<FileNumber>(\d+)</FileNumber>')) {
    $v = [int]$mm.Groups[1].Value
    if ($v -gt $maxFileNo) { $maxFileNo = $v }
}

function New-OptGroup {
    param([string]$GroupName, [int]$GroupNumber, [string]$SubDir, [string[]]$Names, [int]$StartFileNo)
    $sb = New-Object System.Text.StringBuilder
    [void]$sb.Append("<Group>`r`n")
    [void]$sb.Append("    <GroupName>$GroupName</GroupName>`r`n")
    [void]$sb.Append("    <tvExp>1</tvExp>`r`n")
    [void]$sb.Append("    <tvExpOptDlg>0</tvExpOptDlg>`r`n")
    [void]$sb.Append("    <cbSel>0</cbSel>`r`n")
    [void]$sb.Append("    <RteFlg>0</RteFlg>`r`n")
    $no = $StartFileNo
    foreach ($n in $Names) {
        foreach ($e in @('c','h')) {
            $file = "$n.$e"
            $fileType = if ($e -eq 'c') { 1 } else { 5 }
            [void]$sb.Append("    <File>`r`n")
            [void]$sb.Append("      <GroupNumber>$GroupNumber</GroupNumber>`r`n")
            [void]$sb.Append("      <FileNumber>$no</FileNumber>`r`n")
            [void]$sb.Append("      <FileType>$fileType</FileType>`r`n")
            [void]$sb.Append("      <tvExp>0</tvExp>`r`n")
            [void]$sb.Append("      <tvExpOptDlg>0</tvExpOptDlg>`r`n")
            [void]$sb.Append("      <bDave2>0</bDave2>`r`n")
            [void]$sb.Append("      <PathWithFileName>.\icode\$SubDir\$file</PathWithFileName>`r`n")
            [void]$sb.Append("      <FilenameWithoutPath>$file</FilenameWithoutPath>`r`n")
            [void]$sb.Append("      <RteFlg>0</RteFlg>`r`n")
            [void]$sb.Append("      <bShared>0</bShared>`r`n")
            [void]$sb.Append("    </File>`r`n")
            $no++
        }
    }
    [void]$sb.Append("  </Group>")
    return @{ Text = $sb.ToString(); Next = $no }
}

$g1Start = 29
$g1 = New-OptGroup -GroupName 'icode/driver' -GroupNumber 5 -SubDir 'driver' -Names $drivers -StartFileNo $g1Start
# driver 有 7 个文件 * 2 = 14 项，从 29 开始 -> 到 42，故 task 从 43 起
$g2Start = $g1.Next
$g2 = New-OptGroup -GroupName 'icode/task'   -GroupNumber 6 -SubDir 'task'   -Names $tasks   -StartFileNo $g2Start

$newOptGroups = $g1.Text + "`r`n  " + $g2.Text
$opt = $opt.Remove($oldGroups[$icodeIdx].Index, $oldGroups[$icodeIdx].Length).Insert($oldGroups[$icodeIdx].Index, $newOptGroups)

# 把 icode 之后的分组的 GroupNumber 后移一位（原 6=FreeRTOS, 7=CMSIS -> 7, 8）
$opt = [regex]::Replace($opt, '<GroupNumber>(\d+)</GroupNumber>', {
    param($mm)
    $v = [int]$mm.Groups[1].Value
    if ($v -ge 6 -and $v -le 7) { "<GroupNumber>$($v + 1)</GroupNumber>" } else { $mm.Value }
})
Write-Host "[5/6] uvoptx 已重建（driver 文件号 $g1Start-$($g1.Next - 1)，task 文件号 $g2Start-$($g2.Next - 1)，原 6/7 组号已后移）"

# --- 6. 写回 ---------------------------------------------------------------
[System.IO.File]::WriteAllText((Join-Path $mdk "$newName.uvprojx"), $prj, $utf8NoBom)
[System.IO.File]::WriteAllText((Join-Path $mdk "$newName.uvoptx"), $opt, $gbk)
Remove-Item -LiteralPath $prjPath, $optPath -Force

# 顶层 .ioc 与 .uvoptx
[System.IO.File]::WriteAllText((Join-Path $mdk "$newName.uvguix.wcx12"),
    ([System.IO.File]::ReadAllText((Join-Path $mdk "$oldName.uvguix.wcx12"), $gbk)).Replace("<TargetName>$oldName</TargetName>", "<TargetName>$newName</TargetName>"),
    $gbk)
Remove-Item -LiteralPath (Join-Path $mdk "$oldName.uvguix.wcx12") -Force

$ioc = [System.IO.File]::ReadAllText((Join-Path $root "$oldName.ioc"), $utf8NoBom)
$ioc = $ioc.Replace("ProjectManager.ProjectFileName=$oldName.ioc", "ProjectManager.ProjectFileName=$newName.ioc")
$ioc = $ioc.Replace("ProjectManager.ProjectName=$oldName", "ProjectManager.ProjectName=$newName")
[System.IO.File]::WriteAllText((Join-Path $root "$newName.ioc"), $ioc, $utf8NoBom)
Remove-Item -LiteralPath (Join-Path $root "$oldName.ioc") -Force
Remove-Item -LiteralPath (Join-Path $root "$oldName.uvoptx") -Force

Write-Host "[6/6] 写回完成，旧名文件已清理"
Write-Host ""
Write-Host "=== MDK-ARM 目录 ==="
Get-ChildItem -LiteralPath $mdk | Select-Object Name | Format-Table -AutoSize
Write-Host "=== 顶层目录 ==="
Get-ChildItem -LiteralPath $root -Force | Select-Object Name | Format-Table -AutoSize
