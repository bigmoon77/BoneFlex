$ErrorActionPreference = 'Continue'
$latestLog = 'E:/code/bigmoon77/BoneFlex\GPTShare\logs\commands\latest\command.log'
$historyLog = 'E:/code/bigmoon77/BoneFlex\GPTShare\logs\commands\history\20261124_031555_366.log'
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
  @{ Path = 'E:\code\bigmoon77\BoneFlex\GPTShare\commands\test_command_4.ps1'; StatePath = 'E:/code/bigmoon77/BoneFlex\.devtrace\command-state\9209E6B4E5C82D1E7D508E8750486E678E49A13B1298187457BD4D120E69FFD4.sha256'; Hash = 'D09DC596C5F2A6351EBDE357BBD3AA374755BF62EABD3128598D3BEEAB6667B5' }
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
Write-CommandLog 'All pulled command scripts processed. This Developer PowerShell window will remain open.'
