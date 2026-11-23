$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
Push-Location $repoRoot
try {
    Write-Output "BoneFlex DX11 demo build started: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')"
    cmake -S . -B build -DBONEFLEX_BUILD_DX11_DEMO=ON
    if ($LASTEXITCODE -ne 0) { throw "CMake configure failed with exit code $LASTEXITCODE" }

    cmake --build build --config Release --target boneflex_dx11_soft_body_demo
    if ($LASTEXITCODE -ne 0) { throw "DX11 demo build failed with exit code $LASTEXITCODE" }

    $demoPath = Join-Path $repoRoot 'build\Release\boneflex_dx11_soft_body_demo.exe'
    if (-not (Test-Path -LiteralPath $demoPath)) { throw "Demo executable not found: $demoPath" }

    Write-Output 'Build succeeded. Launching DX11 demo; close the demo window to finish this command.'
    & $demoPath
    if ($LASTEXITCODE -ne 0) { throw "DX11 demo exited with code $LASTEXITCODE" }
    Write-Output 'DX11 demo closed.'
}
finally {
    Pop-Location
}
