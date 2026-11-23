$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
Set-Location $repoRoot
Write-Output "Repository: $repoRoot"
Write-Output "BoneFlex build and direct Blender import smoke test started: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')"
cmake -S $repoRoot -B (Join-Path $repoRoot 'build') -DBONEFLEX_BUILD_EXAMPLES=ON -DBONEFLEX_BUILD_DX11_DEMO=ON -DBONEFLEX_BUILD_BENCHMARKS=ON
if ($LASTEXITCODE -ne 0) { throw "CMake configure failed: $LASTEXITCODE" }
cmake --build (Join-Path $repoRoot 'build') --config Release
if ($LASTEXITCODE -ne 0) { throw "Build failed: $LASTEXITCODE" }
ctest --test-dir (Join-Path $repoRoot 'build') -C Release --output-on-failure
if ($LASTEXITCODE -ne 0) { throw "Tests failed: $LASTEXITCODE" }
Write-Output "Build and all unit tests succeeded."

$blenderPath = Join-Path 'E:\SteamLibrary\steamapps\common\Blender' 'blender.exe'
if (Test-Path $blenderPath) {
    $buildDirectory = Join-Path $repoRoot 'build'
    $sampleAsset = Join-Path $buildDirectory 'boneflex_test_character.bflex'
    $generatorScript = Join-Path $repoRoot 'blender\create_test_character.py'
    $sampleBlend = Join-Path $buildDirectory 'boneflex_test_character.blend'
    $diagnosticBlend = 'E:\code\DesktopMascot\model\EL-FPr244\EL-FPr244.blend'
    $blenderStdout = Join-Path $buildDirectory 'blender_export.stdout.log'
    $blenderStderr = Join-Path $buildDirectory 'blender_export.stderr.log'
    Write-Output "Blender found. Generating test character: $sampleAsset"
    $quote = [string][char]34
    $blenderProcess = Start-Process -FilePath $blenderPath -ArgumentList @(
        '--background',
        '--python', ($quote + $generatorScript + $quote),
        '--',
        '--output', ($quote + $sampleAsset + $quote),
        '--skip-glb',
        '--save-blend'
    ) -Wait -PassThru -NoNewWindow -RedirectStandardOutput $blenderStdout -RedirectStandardError $blenderStderr
    Write-Output "Blender fixture exit code: $($blenderProcess.ExitCode)"
    if (Test-Path $blenderStdout) { Get-Content $blenderStdout | ForEach-Object { Write-Output $_ } }
    if (Test-Path $blenderStderr) { Get-Content $blenderStderr | ForEach-Object { Write-Output $_ } }
    if ($blenderProcess.ExitCode -ne 0) { throw "Blender test character export failed: $($blenderProcess.ExitCode)" }

    & (Join-Path $repoRoot 'build\Release\boneflex_blender_asset_demo.exe') $sampleAsset
    if ($LASTEXITCODE -ne 0) { throw "Blender-exported asset import demo failed: $LASTEXITCODE" }

    $dx11Demo = Join-Path $repoRoot 'build\Release\boneflex_dx11_soft_body_demo.exe'
    $dx11Stdout = Join-Path $buildDirectory 'dx11_blend.stdout.log'
    $dx11Stderr = Join-Path $buildDirectory 'dx11_blend.stderr.log'
    Write-Output "Checking converter dependencies."
    foreach ($requiredFile in @('import_blend_to_bflex.py', 'boneflex_exporter.py')) {
        $requiredPath = Join-Path (Split-Path $dx11Demo) $requiredFile
        if (-not (Test-Path $requiredPath)) { throw "DX11 converter dependency missing: $requiredPath" }
        Write-Output "Found: $requiredPath"
    }

    Write-Output "Testing direct .blend loading through the DX11 demo: $sampleBlend"
    $env:BONEFLEX_BLENDER_EXE = $blenderPath
    $blendProcess = Start-Process -FilePath $dx11Demo -ArgumentList @($quote + $sampleBlend + $quote) -PassThru -RedirectStandardOutput $dx11Stdout -RedirectStandardError $dx11Stderr
    Start-Sleep -Seconds 5
    $blendProcess.Refresh()
    if ($blendProcess.HasExited) {
        Remove-Item Env:BONEFLEX_BLENDER_EXE -ErrorAction SilentlyContinue
        Write-Output "DX11 demo exited before the GUI smoke test completed."
        if (Test-Path $dx11Stdout) { Get-Content $dx11Stdout | ForEach-Object { Write-Output $_ } }
        if (Test-Path $dx11Stderr) { Get-Content $dx11Stderr | ForEach-Object { Write-Output $_ } }
        throw "DX11 demo failed during .blend loading or GUI initialization."
    }
    [void]$blendProcess.CloseMainWindow()
    if (-not $blendProcess.WaitForExit(10000)) {
        Stop-Process -Id $blendProcess.Id -Force
        Remove-Item Env:BONEFLEX_BLENDER_EXE -ErrorAction SilentlyContinue
        throw "DX11 demo did not close after the smoke test."
    }
    Remove-Item Env:BONEFLEX_BLENDER_EXE -ErrorAction SilentlyContinue
    Write-Output "===== DX11 stdout ====="
    if (Test-Path $dx11Stdout) { Get-Content $dx11Stdout | ForEach-Object { Write-Output $_ } }
    Write-Output "===== DX11 stderr ====="
    if (Test-Path $dx11Stderr) { Get-Content $dx11Stderr | ForEach-Object { Write-Output $_ } }
    $dx11Output = if (Test-Path $dx11Stdout) { Get-Content $dx11Stdout -Raw } else { '' }
    if ($dx11Output -notmatch 'Control points:' -or $dx11Output -notmatch 'Kinematic anchors:') {
        throw "DX11 demo did not report successful humanoid asset initialization."
    }
    Write-Output "DX11 .blend conversion, humanoid initialization, and GUI startup succeeded."

    Write-Output "Launching interactive drift diagnostic. Gravity and skeleton animation should start disabled."
    $env:BONEFLEX_BLENDER_EXE = $blenderPath
    $diagnosticStdout = Join-Path $buildDirectory 'dx11_diagnostic.stdout.log'
    $diagnosticStderr = Join-Path $buildDirectory 'dx11_diagnostic.stderr.log'
    if (-not (Test-Path $diagnosticBlend)) { throw "Diagnostic model not found: $diagnosticBlend" }
    $diagnosticProcess = Start-Process -FilePath $dx11Demo -ArgumentList @($quote + $diagnosticBlend + $quote) -PassThru -RedirectStandardOutput $diagnosticStdout -RedirectStandardError $diagnosticStderr
    Start-Sleep -Seconds 3
    $diagnosticProcess.Refresh()
    if ($diagnosticProcess.HasExited) {
        Remove-Item Env:BONEFLEX_BLENDER_EXE -ErrorAction SilentlyContinue
        if (Test-Path $diagnosticStdout) { Get-Content $diagnosticStdout | ForEach-Object { Write-Output $_ } }
        if (Test-Path $diagnosticStderr) { Get-Content $diagnosticStderr | ForEach-Object { Write-Output $_ } }
        throw "Interactive DX11 diagnostic exited during startup."
    }
    Write-Output "Interactive DX11 diagnostic started (PID: $($diagnosticProcess.Id))."
    Write-Output "Capturing the real model for 15 seconds with no input; gravity and animation remain disabled."
    Start-Sleep -Seconds 15
    $diagnosticProcess.Refresh()
    if (-not $diagnosticProcess.HasExited) {
        [void]$diagnosticProcess.CloseMainWindow()
        if (-not $diagnosticProcess.WaitForExit(10000)) {
            Stop-Process -Id $diagnosticProcess.Id -Force
            Remove-Item Env:BONEFLEX_BLENDER_EXE -ErrorAction SilentlyContinue
            throw "Interactive DX11 diagnostic did not close after capture."
        }
    }
    Remove-Item Env:BONEFLEX_BLENDER_EXE -ErrorAction SilentlyContinue
    Write-Output "===== Real model DX11 stdout ====="
    if (Test-Path $diagnosticStdout) { Get-Content $diagnosticStdout | ForEach-Object { Write-Output $_ } }
    Write-Output "===== Real model DX11 stderr ====="
    if (Test-Path $diagnosticStderr) { Get-Content $diagnosticStderr | ForEach-Object { Write-Output $_ } }
}
else {
    Write-Output "Blender executable not found at '$blenderPath'; skipping Blender smoke test."
}
Write-Output "Physics tuning GUI: stiffness, XPBD compliance, solver iterations, velocity damping, maximum deformation, gravity, and surface smoothing."
Write-Output "Drag interaction now uses a soft XPBD position target."
Write-Output "Capture Physics stability frame 90-130 lines; each reports rest deviation, maximum speed, per-step displacement, and interaction/toggle state. Also capture frame 60/120/180 summaries."
