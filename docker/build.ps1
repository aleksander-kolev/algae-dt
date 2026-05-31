# Build the algae-dt dev image (Windows / PowerShell).
$ErrorActionPreference = 'Stop'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
docker build -t algae-dt:dev $here
