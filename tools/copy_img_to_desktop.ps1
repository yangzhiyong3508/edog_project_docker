# copy_img_to_desktop.ps1
# 规则：每次修改、编译完成后，把生成的 Firmware.img 下载到 Windows 桌面。
#
# 用法（在 Windows PowerShell 里执行）：
#   .\copy_img_to_desktop.ps1              # 用默认容器名 edog，默认镜像名 Firmware.img
#   .\copy_img_to_desktop.ps1 -ContainerId 42fd8167b0bf
#   .\copy_img_to_desktop.ps1 -ImgName Firmware.img
#
# 行为：
#   1. 用 docker cp 从容器拷出 Firmware.img。
#   2. 文件名带时间戳，避免覆盖历史版本：Firmware_YYYYMMDD_HHMMSS.img
#   3. 目标目录：%USERPROFILE%\Desktop（当前用户桌面）。
#   4. 拷贝完成后打印最终路径，并保留最近 20 个历史 img（更早的自动清理）。
#
param(
    [string]$ContainerId = "",
    [string]$ImgName = "Firmware.img",
    [string]$ImgPathInContainer = "/home/openharmony/txsmartropenharmony/out/rk2206/isoftstone-rk2206/images/Firmware.img"
)

$ErrorActionPreference = "Stop"

# 1. 解析容器：未传 -ContainerId 时，自动列出运行中容器供选择。
if ([string]::IsNullOrEmpty($ContainerId)) {
    Write-Host "[copy_img] 未指定 -ContainerId，当前运行中的容器：" -ForegroundColor Cyan
    docker ps --format "  {{.ID}}  {{.Names}}  {{.Image}}"
    $ContainerId = Read-Host "[copy_img] 请输入容器 ID 或名称（短 ID 即可）"
    if ([string]::IsNullOrEmpty($ContainerId)) {
        Write-Host "[copy_img] 未输入容器标识，退出。" -ForegroundColor Yellow
        exit 1
    }
}

# 校验容器存在且在运行
$probe = docker ps --filter "id=$ContainerId" --format "{{.ID}}" 2>$null
if (-not $probe) {
    $probe = docker ps --filter "name=$ContainerId" --format "{{.ID}}" 2>$null
}
if (-not $probe) {
    Write-Host "[copy_img] 找不到运行中的容器 '$ContainerId'。" -ForegroundColor Red
    docker ps --format "  {{.ID}}  {{.Names}}  {{.Image}}"
    exit 1
}
$container = $probe

# 2. 确认容器内 img 存在
$check = docker exec $container test -f $ImgPathInContainer 2>$null
if ($LASTEXITCODE -ne 0) {
    Write-Host "[copy_img] 容器内未找到 img: $ImgPathInContainer" -ForegroundColor Red
    Write-Host "[copy_img] 请先在容器内完成编译（hb build -f）。" -ForegroundColor Yellow
    exit 1
}

# 3. 目标桌面路径
$desktop = Join-Path $env:USERPROFILE "Desktop"
if (-not (Test-Path $desktop)) {
    # 中文系统桌面可能是“桌面”
    $desktop = Join-Path $env:USERPROFILE "桌面"
}
if (-not (Test-Path $desktop)) {
    Write-Host "[copy_img] 找不到桌面目录（既无 Desktop 也无 桌面）。" -ForegroundColor Red
    exit 1
}

# 4. 带时间戳的文件名
$ts = Get-Date -Format "yyyyMMdd_HHmmss"
$dstName = "$($ImgName -replace '\.img$','')_$ts.img"
$dst = Join-Path $desktop $dstName

# 5. docker cp
Write-Host "[copy_img] 从容器 $container 拷出 img ..." -ForegroundColor Cyan
docker cp "${container}:$ImgPathInContainer" $dst
if ($LASTEXITCODE -ne 0) {
    Write-Host "[copy_img] docker cp 失败。" -ForegroundColor Red
    exit 1
}

# 6. 清理超过 20 个的历史 img（按时间倒序保留最新 20）
Get-ChildItem -Path $desktop -Filter "Firmware_*.img" |
    Sort-Object LastWriteTime -Descending |
    Select-Object -Skip 20 |
    Remove-Item -Force -ErrorAction SilentlyContinue

# 7. 打印结果
$size = (Get-Item $dst).Length
Write-Host "[copy_img] 完成: $dst ($([math]::Round($size/1MB,2)) MB)" -ForegroundColor Green
Write-Host "[copy_img] 历史保留最近 20 个 Firmware_*.img。" -ForegroundColor DarkGray
