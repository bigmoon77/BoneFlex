$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
Set-Location $repoRoot
Write-Output "Repository: $repoRoot"
Write-Output "BoneFlex Blender importer build started: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')"
cmake -S $repoRoot -B (Join-Path $repoRoot 'build') -DBONEFLEX_BUILD_EXAMPLES=ON -DBONEFLEX_BUILD_DX11_DEMO=ON -DBONEFLEX_BUILD_BENCHMARKS=ON
if ($LASTEXITCODE -ne 0) { throw "CMake configure failed: $LASTEXITCODE" }
cmake --build (Join-Path $repoRoot 'build') --config Release
if ($LASTEXITCODE -ne 0) { throw "Build failed: $LASTEXITCODE" }
ctest --test-dir (Join-Path $repoRoot 'build') -C Release --output-on-failure
if ($LASTEXITCODE -ne 0) { throw "Tests failed: $LASTEXITCODE" }
Write-Output "Blender importer build and tests succeeded."
