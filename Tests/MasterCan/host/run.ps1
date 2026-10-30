$ErrorActionPreference = "Stop"

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$libPath = Join-Path $projectRoot "Lib"
$stubPath = Join-Path $PSScriptRoot "stubs"
$source = Join-Path $PSScriptRoot "test_master_can.c"
$exe = Join-Path $env:TEMP "master_can_host_test.exe"

& gcc `
    -std=c99 `
    -Wall `
    -Wextra `
    -Werror `
    -I $stubPath `
    -I $libPath `
    $source `
    (Join-Path $libPath "MasterCan.c") `
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
