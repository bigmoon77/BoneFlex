$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
Set-Location $repoRoot
Write-Output "Repository: $repoRoot"
Write-Output "BoneFlex DX11 kinematic drag fix build and Blender import smoke test started: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')"
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
    $generatorScript = Join-Path $repoRoot 'blender\create_test_character.py'
    $blenderStdout = Join-Path (Join-Path $repoRoot 'build') 'blender_export.stdout.log'
    $blenderStderr = Join-Path (Join-Path $repoRoot 'build') 'blender_export.stderr.log'
    Write-Output "Blender found. Generating a test character: $sampleAsset"
    $quote = [string][char]34
    $blenderProcess = Start-Process -FilePath $blender.Source -ArgumentList @(
        '--background',
        '--python', ($quote + $generatorScript + $quote),
        '--',
        '--output', ($quote + $sampleAsset + $quote),
        '--skip-glb'
    ) -Wait -PassThru -NoNewWindow -RedirectStandardOutput $blenderStdout -RedirectStandardError $blenderStderr
    Write-Output "Blender exit code: $($blenderProcess.ExitCode)"
    Write-Output "===== Blender stdout ====="
    if (Test-Path $blenderStdout) { Get-Content $blenderStdout | ForEach-Object { Write-Output $_ } }
    Write-Output "===== Blender stderr ====="
    if (Test-Path $blenderStderr) { Get-Content $blenderStderr | ForEach-Object { Write-Output $_ } }
    if ($blenderProcess.ExitCode -ne 0) { throw "Blender test character export failed: $($blenderProcess.ExitCode)" }
    & (Join-Path $repoRoot 'build\Release\boneflex_blender_asset_demo.exe') $sampleAsset
    if ($LASTEXITCODE -ne 0) { throw "Blender-exported asset import demo failed: $LASTEXITCODE" }
}
else {
    Write-Output "Blender executable not found; skipping the optional Blender round-trip smoke test."
}

Write-Output "DX11 drag fix: kinematic selected control point with perspective-correct cursor mapping; selection radius 64px."
