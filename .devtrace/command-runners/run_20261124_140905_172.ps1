$ErrorActionPreference = 'Continue'
$latestLog = 'E:/code/bigmoon77/BoneFlex\GPTShare\logs\commands\latest\command.log'
$historyLog = 'E:/code/bigmoon77/BoneFlex\GPTShare\logs\commands\history\20261124_140905_172.log'
Set-Content -LiteralPath $latestLog -Value '' -Encoding UTF8
Set-Content -LiteralPath $historyLog -Value '' -Encoding UTF8
function Write-CommandLog([string]$message) {
  $line = '[' + (Get-Date -Format 'yyyy-MM-dd HH:mm:ss.fff') + '] ' + $message
  Add-Content -LiteralPath $latestLog -Value $line -Encoding UTF8
  Add-Content -LiteralPath $historyLog -Value $line -Encoding UTF8
  Write-Host $line
}
Write-CommandLog 'Developer PowerShell initialization'
$devShellOutput = & 'E:\application\Microsoft Visual Studio\18\Community\Common7\Tools\Launch-VsDevShell.ps1' -SkipAutomaticLocation *>&1
$devShellSucceeded = $?
foreach ($line in $devShellOutput) { Write-CommandLog ([string]$line) }
if (-not $devShellSucceeded) { Write-CommandLog 'Developer PowerShell initialization failed.'; return }
Set-Location -LiteralPath 'E:\code\bigmoon77\BoneFlex\build'
$commands = @(
  @{ Path = 'E:\code\bigmoon77\BoneFlex\GPTShare\commands\test_command_4.ps1'; StatePath = 'E:/code/bigmoon77/BoneFlex\.devtrace\command-state\1F75FAFB1A58F171CF2AE563545A5E70D0AEEBB7BDE0509EDDBC048723C1FDC7.sha256'; Hash = '34AD24A8B1250161C729FA952D538C4B26A2DD10322A8DAD1070C43991361567' }
)
foreach ($command in $commands) {
  $scriptPath = $command.Path
  $statePath = $command.StatePath
  $expectedHash = $command.Hash
  Write-CommandLog ('START ' + $scriptPath)
  $currentHash = (Get-FileHash -LiteralPath $scriptPath -Algorithm SHA256).Hash
  if ($currentHash -ne $expectedHash) { Write-CommandLog 'Skipped: script changed after pull.'; continue }
  $previousHash = if (Test-Path -LiteralPath $statePath) { (Get-Content -LiteralPath $statePath -Raw).Trim() } else { '' }
  if ($previousHash -eq $currentHash) { Write-CommandLog 'Skipped: unchanged script already succeeded.'; continue }
  $Error.Clear()
  $global:LASTEXITCODE = 0
  try {
    & $scriptPath *>&1 | ForEach-Object { Write-CommandLog ([string]$_) }
    $scriptSucceeded = $?
    $scriptExitCode = $global:LASTEXITCODE
    if ($scriptSucceeded -and $scriptExitCode -eq 0 -and $Error.Count -eq 0) {
      $stateDirectory = Split-Path -Parent $statePath
      New-Item -ItemType Directory -Path $stateDirectory -Force | Out-Null
      Set-Content -LiteralPath $statePath -Value $currentHash -Encoding ASCII
      Write-CommandLog 'SUCCESS'
    } else { Write-CommandLog ('FAILED exit=' + $scriptExitCode + ' errors=' + $Error.Count) }
  } catch { Write-CommandLog ('EXCEPTION: ' + $_.Exception.Message) }
  Write-CommandLog ('END ' + $scriptPath)
}
Write-CommandLog 'All pulled command scripts processed. Closing Developer PowerShell.'
