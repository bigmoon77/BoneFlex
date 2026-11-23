$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
Set-Location $repoRoot
Write-Output "Repository: $repoRoot"
Write-Output "BoneFlex smooth embedding rebuild started: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')"
cmake -S $repoRoot -B (Join-Path $repoRoot 'build') -DBONEFLEX_BUILD_DX11_DEMO=ON -DBONEFLEX_BUILD_BENCHMARKS=ON
if ($LASTEXITCODE -ne 0) { throw "CMake configure failed: $LASTEXITCODE" }
cmake --build (Join-Path $repoRoot 'build') --config Release
if ($LASTEXITCODE -ne 0) { throw "Build failed: $LASTEXITCODE" }
ctest --test-dir (Join-Path $repoRoot 'build') -C Release --output-on-failure
if ($LASTEXITCODE -ne 0) { throw "Tests failed: $LASTEXITCODE" }
Write-Output "Running mesh reduction benchmark..."
& (Join-Path $repoRoot 'build\Release\boneflex_mesh_reduction_benchmark.exe')
if ($LASTEXITCODE -ne 0) { throw "Mesh reduction benchmark failed: $LASTEXITCODE" }
Write-Output "Launching DX11 demo; close the demo window to finish this command."
& (Join-Path $repoRoot 'build\Release\boneflex_dx11_soft_body_demo.exe')
if ($LASTEXITCODE -ne 0) { throw "DX11 demo exited with code $LASTEXITCODE" }
Write-Output "DX11 demo closed."
