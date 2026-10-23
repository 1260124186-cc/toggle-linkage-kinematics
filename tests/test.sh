#!/bin/bash
set -euo pipefail

LOG_DIR=/logs/verifier
REWARD_FILE="$LOG_DIR/reward.txt"
STATUS_FILE="$LOG_DIR/verification-status.txt"
REPORT="$LOG_DIR/results.xml"
STDOUT_FILE="$LOG_DIR/test-stdout.txt"
SITE_PACKAGES=/tmp/site-packages
TRUSTED_PYTHON=/usr/local/bin/python3
TRUSTED_TIMEOUT=/usr/bin/timeout

mkdir -p "$LOG_DIR"
echo "0" > "$REWARD_FILE"
printf 'INCOMPLETE: verification started\n' > "$STATUS_FILE"
: > "$STDOUT_FILE"

# Offline policy equivalent: --no-index --find-links=/tests/wheels --require-hashes
# is enforced by bootstrap_wheels.py against tests/requirements.txt.

write_failure() {
  local reason="$1"
  printf '0.000000\n' > "$REWARD_FILE"
  printf 'INCOMPLETE: %s\n' "$reason" > "$STATUS_FILE"
}

seal_reward() {
  if [ -f "$REWARD_FILE" ]; then
    chmod 0444 "$REWARD_FILE" || true
  fi
}

trap 'write_failure "interrupted before completion"; exit 130' INT TERM HUP
trap 'status=$?; if [ "$status" -ne 0 ]; then write_failure "unexpected exit ${status}"; fi; seal_reward; exit "$status"' EXIT

if ! env -i PATH=/usr/local/bin:/usr/bin:/bin HOME=/tmp \
    "$TRUSTED_PYTHON" -I -S -B /tests/bootstrap_wheels.py \
    --wheel-dir /tests/wheels \
    --requirements /tests/requirements.txt \
    --target "$SITE_PACKAGES" > "$LOG_DIR/bootstrap.log" 2>&1; then
  write_failure "offline wheel bootstrap failed"
  exit 0
fi

set +e
env -i PATH=/usr/local/bin:/usr/bin:/bin HOME=/tmp \
    PYTHONPATH="$SITE_PACKAGES:/tests" \
    PYTHONNOUSERSITE=1 \
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
    "$TRUSTED_TIMEOUT" --signal=TERM --kill-after=10s 300s \
    "$TRUSTED_PYTHON" -S -s -P -B -c \
    'import pytest; raise SystemExit(pytest.main())' \
    -p no:cacheprovider --junitxml="$REPORT" \
    /tests/test_outputs.py > "$STDOUT_FILE" 2>&1
pytest_status=$?
set -e
printf '\npytest_exit=%s\n' "$pytest_status" >> "$STDOUT_FILE"

if [ ! -s "$REPORT" ]; then
  write_failure "pytest did not produce a structured report"
  exit 0
fi

if ! env -i PATH=/usr/local/bin:/usr/bin:/bin HOME=/tmp \
    PYTHONPATH="$SITE_PACKAGES:/tests" \
    "$TRUSTED_PYTHON" -S -s -P -B /tests/score.py \
    --junit "$REPORT" \
    --config /tests/scoring.json \
    --reward-out "$REWARD_FILE" \
    --status-out "$STATUS_FILE" \
    --report-out "$LOG_DIR/scoring-report.json"; then
  write_failure "weighted scorer failed to produce a result"
  exit 0
fi

seal_reward
trap - INT TERM HUP
exit 0
