$ErrorActionPreference = "Stop"

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$libPath = Join-Path $projectRoot "Lib"
$source = Join-Path $PSScriptRoot "test_angle_opt.c"
$exe = Join-Path $env:TEMP "angle_opt_host_test.exe"

& gcc `
    -std=c99 `
    -Wall `
    -Wextra `
    -Werror `
    -I $libPath `
    $source `
    (Join-Path $libPath "angle_opt.c") `
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
