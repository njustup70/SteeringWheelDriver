$ErrorActionPreference = "Stop"

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$libPath = Join-Path $projectRoot "Lib"
$stubPath = Join-Path $PSScriptRoot "stubs"
$source = Join-Path $PSScriptRoot "test_gm6020.c"
$exe = Join-Path $env:TEMP "gm6020_host_test.exe"

& gcc `
    -std=c99 `
    -Wall `
    -Wextra `
    -Werror `
    -I $stubPath `
    -I $libPath `
    $source `
    (Join-Path $libPath "GM6020.c") `
    (Join-Path $libPath "pid.c") `
    (Join-Path $libPath "bsp_can.c") `
    -lm `
    -o $exe

if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

try {
    & $exe
    $exitCode = $LASTEXITCODE
}
finally {
    Remove-Item -LiteralPath $exe -Force -ErrorAction SilentlyContinue
}

exit $exitCode
