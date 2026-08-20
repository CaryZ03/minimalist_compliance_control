#!/usr/bin/env bash
# Run from the T800 Orin after deployment by sync.sh.

set -eo pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
DEPLOY_DIR="$(cd "$(dirname "$SCRIPT_PATH")/../.." && pwd)"
RUNTIME_ROOT="$(cd "$DEPLOY_DIR/.." && pwd)"

source /app/applications/install/bringup/ros_env.sh
export PYTHONPATH="$RUNTIME_ROOT/python${PYTHONPATH:+:$PYTHONPATH}"
cd "$DEPLOY_DIR"

exec python3 -m policy.run_t800_real "$@"
