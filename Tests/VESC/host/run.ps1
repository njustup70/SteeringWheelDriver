$ErrorActionPreference = "Stop"

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$libPath = Join-Path $projectRoot "Lib"
$stubPath = Join-Path $PSScriptRoot "stubs"
$source = Join-Path $PSScriptRoot "test_vesc.c"
$exe = Join-Path $env:TEMP "vesc_host_test.exe"

& gcc `
    -std=c99 `
    -Wall `
    -Wextra `
    -Werror `
    -I $stubPath `
    -I $libPath `
    $source `
    (Join-Path $libPath "VESC.c") `
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
