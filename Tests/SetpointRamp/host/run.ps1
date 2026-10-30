$ErrorActionPreference = "Stop"

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$libPath = Join-Path $projectRoot "Lib"
$source = Join-Path $PSScriptRoot "test_setpoint_ramp.c"
$exe = Join-Path $env:TEMP "setpoint_ramp_host_test.exe"

& gcc `
    -std=c99 `
    -Wall `
    -Wextra `
    -Werror `
    -I $libPath `
    $source `
    (Join-Path $libPath "setpoint_ramp.c") `
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
