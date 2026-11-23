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

$blender = Get-Command blender -ErrorAction SilentlyContinue
if ($null -ne $blender) {
    $sampleAsset = Join-Path (Join-Path $repoRoot 'build') 'boneflex_test_character.bflex'
    Write-Output "Blender found. Generating a test character: $sampleAsset"
    & $blender.Source --background --python (Join-Path $repoRoot 'blender\create_test_character.py') -- --output $sampleAsset --skip-glb
    if ($LASTEXITCODE -ne 0) { throw "Blender test character export failed: $LASTEXITCODE" }
    & (Join-Path $repoRoot 'build\Release\boneflex_blender_asset_demo.exe') $sampleAsset
    if ($LASTEXITCODE -ne 0) { throw "Blender-exported asset import demo failed: $LASTEXITCODE" }
}
else {
    Write-Output "Blender executable not found; skipping the optional Blender round-trip smoke test."
}
