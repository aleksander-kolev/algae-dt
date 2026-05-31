#!/usr/bin/env bash
# Build the algae-dt dev image (WSL / lab laptop / Linux).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
docker build -t algae-dt:dev "$HERE"
