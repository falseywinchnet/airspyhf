<#
.SYNOPSIS
    Build and install the Rust AirspyHF+ driver and tools on Windows.

.DESCRIPTION
    Builds the workspace in release mode and copies the driver DLL, its import
    library, the command-line tools and the C headers into a prefix directory.

    SDR consumers load "airspyhf.dll" (NOT "libairspyhf.dll"). The crate's
    [lib] name is "airspyhf", so cargo already produces airspyhf.dll; this
    script installs it under that name (override with -DllName if you want an
    "experimentalairspyhf.dll" side-by-side install).

.PARAMETER Prefix
    Install root. Defaults to "$PWD\dist\install". Under it: bin\, lib\, include\.

.PARAMETER Target
    Optional rustc target triple to cross-compile (e.g. x86_64-pc-windows-msvc).

.PARAMETER Native
    Optimize for the local CPU (-C target-cpu=native).

.PARAMETER DllName
    Base name for the installed DLL (default "airspyhf" -> airspyhf.dll).

.EXAMPLE
    .\install.ps1
    .\install.ps1 -Prefix "C:\airspyhf" -Native
#>
param(
    [string]$Prefix = (Join-Path $PSScriptRoot "dist\install"),
    [string]$Target = "",
    [switch]$Native,
    [string]$DllName = "airspyhf"
)

$ErrorActionPreference = "Stop"
$scriptDir = $PSScriptRoot
$repoRoot = (Resolve-Path (Join-Path $scriptDir "..")).Path
Set-Location $scriptDir

if (-not (Get-Command cargo -ErrorAction SilentlyContinue)) {
    Write-Error "cargo not found on PATH"
}

# ---- build ---------------------------------------------------------------
$cargoArgs = @("build", "--release")
if ($Target -ne "") { $cargoArgs += @("--target", $Target) }

if ($Native) {
    Write-Host ">> building (native CPU tuning) ..."
    $env:RUSTFLAGS = ($env:RUSTFLAGS + " -C target-cpu=native").Trim()
    & cargo @cargoArgs
    Remove-Item Env:\RUSTFLAGS -ErrorAction SilentlyContinue
} else {
    Write-Host ">> building (portable) ..."
    & cargo @cargoArgs
}

if ($Target -ne "") { $out = "target\$Target\release" } else { $out = "target\release" }

# ---- install -------------------------------------------------------------
$binDir = Join-Path $Prefix "bin"
$libDir = Join-Path $Prefix "lib"
$incDir = Join-Path $Prefix "include\libairspyhf"
New-Item -ItemType Directory -Force -Path $binDir, $libDir, $incDir | Out-Null

$tools = @("airspyhf_lib_version", "airspyhf_info", "airspyhf_gpio", "airspyhf_calibrate", "airspyhf_rx")

# The DLL goes next to the tools (bin) so they run in place, and also into lib.
Copy-Item (Join-Path $out "airspyhf.dll") (Join-Path $binDir "$DllName.dll") -Force
Copy-Item (Join-Path $out "airspyhf.dll") (Join-Path $libDir "$DllName.dll") -Force

# Import library: MSVC produces airspyhf.dll.lib, GNU produces libairspyhf.dll.a.
foreach ($implib in @("airspyhf.dll.lib", "libairspyhf.dll.a")) {
    $p = Join-Path $out $implib
    if (Test-Path $p) { Copy-Item $p $libDir -Force }
}

foreach ($t in $tools) {
    Copy-Item (Join-Path $out "$t.exe") $binDir -Force
}

foreach ($h in @("airspyhf.h", "airspyhf_commands.h", "iqbalancer.h")) {
    Copy-Item (Join-Path $repoRoot "libairspyhf\src\$h") $incDir -Force
}

Write-Host ""
Write-Host ">> Installed into $Prefix"
Write-Host "     lib\$DllName.dll"
foreach ($t in $tools) { Write-Host "     bin\$t.exe" }
Write-Host ""
Write-Host ">> Reminder: SDR consumers expect 'airspyhf.dll'. Keep that name."
