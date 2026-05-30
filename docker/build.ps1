# Build the algae-dt course-faithful dev image (Windows / PowerShell).
$ErrorActionPreference = 'Stop'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
docker build -t algae-dt:dev $here
