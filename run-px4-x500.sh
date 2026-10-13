#!/usr/bin/env bash
set -euo pipefail

PX4_RUN_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${PX4_RUN_DIR}/px4-gz-env.sh"
cd "${PX4_AUTOPILOT_DIR}"

exec make px4_sitl gz_x500 "$@"
