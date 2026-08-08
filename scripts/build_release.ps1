# Build release archive on Windows (PowerShell).
$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
$Version = if ($env:AMP_VERSION) { $env:AMP_VERSION } else { "0.1.0" }
$Out = Join-Path $Root "dist"
$Stage = Join-Path $Out "robot_release_$Version"
if (Test-Path $Stage) { Remove-Item -Recurse -Force $Stage }
New-Item -ItemType Directory -Force -Path $Stage | Out-Null
$items = @(
  "amp_core","config","web_dashboard","scripts","deployment","docs",
  "calibration","models","ros2_ws","evaluation","paper",
  "requirements.txt","pyproject.toml","README.md","LICENSE","FINAL_SYSTEM_REPORT.md"
)
foreach ($i in $items) {
  $src = Join-Path $Root $i
  if (Test-Path $src) {
    $dst = Join-Path $Stage $i
    New-Item -ItemType Directory -Force -Path (Split-Path $dst) | Out-Null
    Copy-Item -Recurse -Force $src $dst
  }
}
Get-ChildItem -Path $Stage -Recurse -Directory -Filter "__pycache__" | Remove-Item -Recurse -Force -ErrorAction SilentlyContinue
New-Item -ItemType Directory -Force -Path $Out | Out-Null
$archive = Join-Path $Out "robot_release_$Version.zip"
if (Test-Path $archive) { Remove-Item $archive }
Compress-Archive -Path $Stage -DestinationPath $archive
Write-Host "Created $archive"
