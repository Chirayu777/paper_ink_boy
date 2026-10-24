<#
.SYNOPSIS
    Write a Game Boy ROM into the "rom" partition of a Paper Ink Boy board.

.DESCRIPTION
    Writes the ROM with esptool at the rom partition's offset, read from partitions.csv, the same
    way idf.py flash writes the app. The firmware is not touched: swap games without rebuilding.
    If esptool's flasher stub fails to start (seen over USB-Serial/JTAG on the ESP32-C5), it retries
    without the stub.

    Run it from a shell where the ESP-IDF environment is active, or let the script activate it
    (-IdfActivate, the EIM v6.1 profile script by default). Close idf.py monitor first: it holds
    the port.

.EXAMPLE
    .\tools\flash_rom.ps1 -Rom .\game\tetris.gb -Port COM19
#>
param(
    [Parameter(Mandatory = $true)][string]$Rom,
    [Parameter(Mandatory = $true)][string]$Port,
    [string]$Baud = "460800",
    [string]$IdfActivate = "C:\Espressif\tools\Microsoft.v6.1.PowerShell_profile.ps1"
)

$ErrorActionPreference = "Stop"

if (-not (Test-Path $Rom)) {
    throw "ROM file not found: $Rom"
}
$Rom = (Resolve-Path $Rom).Path
$bytes = [System.IO.File]::ReadAllBytes($Rom)
$size = $bytes.Length
if ($size -lt 0x150) {
    throw "$Rom is too small to be a Game Boy ROM ($size bytes)"
}

# Game Boy header checks: header checksum, and the colour flag at 0x143
$sum = 0
for ($i = 0x134; $i -le 0x14C; $i++) { $sum = ($sum - $bytes[$i] - 1) -band 0xFF }
if ($sum -ne $bytes[0x14D]) {
    throw "$Rom has no valid Game Boy header (is it a .gba file?)"
}
if ($bytes[0x143] -eq 0xC0) {
    Write-Warning "This ROM is Game Boy Color only; the emulator is original Game Boy only, it may not run."
}

# The rom partition's offset and size, from partitions.csv
$csv = Join-Path $PSScriptRoot "..\partitions.csv"
$line = Get-Content $csv | Where-Object { $_ -match '^\s*rom\s*,' } | Select-Object -First 1
if (-not $line) {
    throw "no rom partition in $csv"
}
$fields = $line -split ',' | ForEach-Object { $_.Trim() }
$offset = $fields[3]
$partSize = [Convert]::ToInt64($fields[4].Replace("0x", ""), 16)
if ($size -gt $partSize) {
    throw "$Rom ($size bytes) is larger than the rom partition ($partSize bytes)"
}

if (-not $env:IDF_PATH) {
    Write-Host "Activating ESP-IDF with $IdfActivate"
    . $IdfActivate
}

$esptoolArgs = @("-m", "esptool", "--chip", "esp32c5", "-p", $Port, "-b", $Baud,
                 "--before", "default-reset", "--after", "hard-reset")
$writeArgs = @("write-flash", $offset, $Rom)

Write-Host "Writing $Rom ($([math]::Round($size / 1024)) KB) to the rom partition at $offset on $Port"
# esptool writes progress and errors to stderr; Windows PowerShell must not treat that as fatal.
$ErrorActionPreference = "Continue"
python @esptoolArgs @writeArgs
if ($LASTEXITCODE -ne 0) {
    Write-Warning "esptool failed ($LASTEXITCODE), retrying without the flasher stub (slower)"
    python @esptoolArgs "--no-stub" @writeArgs
    if ($LASTEXITCODE -ne 0) {
        throw "esptool failed ($LASTEXITCODE). Close idf.py monitor, unplug and replug the board, and try again."
    }
}
Write-Host "Done. The board restarts into the game."
