#!/usr/bin/env bash
# Sync the minimal MCC runtime to one EngineAI T800 Orin.
# Usage: ./deploy/t800/sync.sh t800e

set -euo pipefail

ROBOT="${1:-}"
case "$ROBOT" in
    t800a|t800b|t800c|t800d|t800e) ;;
    *)
        echo "Usage: $0 {t800a|t800b|t800c|t800d|t800e}" >&2
        exit 1
        ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
REMOTE_ROOT="/home/ubuntu/mcc-t800"
STAGING_DIR="$(mktemp -d)"
CONTROL_SOCKET="$STAGING_DIR/ssh-control"

cleanup() {
    ssh -S "$CONTROL_SOCKET" -O exit "$ROBOT" >/dev/null 2>&1 || true
    rm -rf -- "$STAGING_DIR"
}
trap cleanup EXIT

rsync -a --files-from="$SCRIPT_DIR/files.txt" "$REPO_ROOT/" "$STAGING_DIR/src/"

echo "Connecting to $ROBOT (SSH may ask for the robot password once)..."
ssh -M -S "$CONTROL_SOCKET" -o ControlPersist=60 -fnNT "$ROBOT"

rsync -az --delete \
    -e "ssh -S $CONTROL_SOCKET" \
    --rsync-path="mkdir -p $REMOTE_ROOT/src && rsync" \
    "$STAGING_DIR/src/" "$ROBOT:$REMOTE_ROOT/src/"

ssh -S "$CONTROL_SOCKET" "$ROBOT" bash -s -- "$REMOTE_ROOT" <<'REMOTE_SCRIPT'
set -eo pipefail

runtime_root="$1"
requirements="$runtime_root/src/deploy/t800/requirements.txt"
python_dir="$runtime_root/python"
hash_file="$runtime_root/.requirements.sha256"

new_hash="$(sha256sum "$requirements" | awk '{print $1}')"
old_hash="$(cat "$hash_file" 2>/dev/null || true)"
if [[ "$new_hash" != "$old_hash" ]]; then
    echo "Installing minimal Python dependencies into $python_dir..."
    rm -rf -- "$python_dir"
    mkdir -p "$python_dir"
    python3 -m pip install --target "$python_dir" -r "$requirements"
    printf '%s\n' "$new_hash" > "$hash_file"
fi

# Remove an incomplete venv left by older versions of this deploy script.
rm -rf -- "$runtime_root/.venv"

chmod 755 "$runtime_root/src/deploy/t800/run.sh"
ln -sfn src/deploy/t800/run.sh "$runtime_root/run.sh"

echo "Running headless model/import smoke test..."
"$runtime_root/run.sh" shadow --smoke-test
REMOTE_SCRIPT

echo "Deployment complete: $ROBOT:$REMOTE_ROOT"
echo "Shadow test: ssh -t $ROBOT '$REMOTE_ROOT/run.sh shadow --duration 10'"
