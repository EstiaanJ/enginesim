#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_CSV="${1:-${ROOT_DIR}/tests/performance_profile_stats.csv}"
RUNS="${RUNS:-50}"

tmp_dir="$(mktemp -d)"
trap 'rm -rf "${tmp_dir}"' EXIT

raw_log="${tmp_dir}/performance_profile.log"
build_log="${tmp_dir}/cargo_build.json"

: > "${raw_log}"

echo "Building performance_profile test binary..."
cargo test --release --test performance_profile --no-run --message-format=json >"${build_log}" 2>&1

TEST_BIN="$(python3 - "${build_log}" <<'PY'
import json
import sys
from pathlib import Path

build_log = Path(sys.argv[1])
for line in build_log.read_text().splitlines():
    try:
        obj = json.loads(line)
    except json.JSONDecodeError:
        continue
    executable = obj.get("executable")
    target = obj.get("target", {})
    if executable and target.get("name") == "performance_profile" and "test" in target.get("kind", []):
        print(executable)
        raise SystemExit(0)
raise SystemExit("could not locate test binary in cargo JSON output")
PY
)"

echo "Running performance_profile ${RUNS} times..."
for run in $(seq 1 "${RUNS}"); do
  echo "  run ${run}/${RUNS} started"
  "${TEST_BIN}" ${TEST_FILTER:+"${TEST_FILTER}"} --ignored --nocapture >>"${raw_log}" 2>&1 &
  test_pid=$!
  (
    while kill -0 "${test_pid}" 2>/dev/null; do
      echo "  run ${run}/${RUNS} still running..."
      sleep 5
    done
  ) &
  heartbeat_pid=$!

  set +e
  wait "${test_pid}"
  run_status=$?
  set -e

  kill "${heartbeat_pid}" 2>/dev/null || true
  wait "${heartbeat_pid}" 2>/dev/null || true

  if [ "${run_status}" -ne 0 ]; then
    echo "  run ${run}/${RUNS} failed with exit code ${run_status}" >&2
    exit "${run_status}"
  fi

  echo "  run ${run}/${RUNS} finished"
done

echo "Aggregating results..."
python3 - "${raw_log}" "${OUT_CSV}" <<'PY'
import csv
import math
import re
import statistics
import sys
from collections import defaultdict
from pathlib import Path

raw_log = Path(sys.argv[1])
out_csv = Path(sys.argv[2])

line_re = re.compile(
    r'^(?P<label>.*): rpm=(?P<rpm>\s*[\d.]+)(?: cycles=(?P<cycles>\d+))? '
    r'steps=(?P<steps>\d+) elapsed=(?P<elapsed>[^ ]+) per_step=(?P<per_step>[^ ]+)$'
)
duration_re = re.compile(r'^(?P<value>[\d.]+)(?P<unit>ns|us|µs|ms|s)$')


def to_millis(text: str) -> float:
    match = duration_re.match(text)
    if not match:
        raise ValueError(f"unrecognized duration: {text!r}")
    value = float(match.group("value"))
    unit = match.group("unit")
    if unit == "ns":
        return value / 1_000_000.0
    if unit in ("us", "µs"):
        return value / 1_000.0
    if unit == "ms":
        return value
    if unit == "s":
        return value * 1000.0
    raise AssertionError(unit)


def to_micros(text: str) -> float:
    match = duration_re.match(text)
    if not match:
        raise ValueError(f"unrecognized duration: {text!r}")
    value = float(match.group("value"))
    unit = match.group("unit")
    if unit == "ns":
        return value / 1000.0
    if unit in ("us", "µs"):
        return value
    if unit == "ms":
        return value * 1000.0
    if unit == "s":
        return value * 1_000_000.0
    raise AssertionError(unit)


samples = defaultdict(list)

for line in raw_log.read_text().splitlines():
    match = line_re.match(line.strip())
    if not match:
        continue
    label = match.group("label")
    samples[(label, "elapsed_ms")].append(to_millis(match.group("elapsed")))
    samples[(label, "per_step_us")].append(to_micros(match.group("per_step")))

if not samples:
    raise SystemExit("no performance timing lines were found in test output")

rows = []
for (label, metric), values in samples.items():
    count = len(values)
    mean = statistics.fmean(values)
    median = statistics.median(values)
    lo = min(values)
    hi = max(values)
    stddev = statistics.stdev(values) if count > 1 else 0.0
    rows.append(
        {
            "label": label,
            "metric": metric,
            "count": count,
            "mean": mean,
            "median": median,
            "min": lo,
            "max": hi,
            "range": hi - lo,
            "stddev": stddev,
        }
    )

rows.sort(key=lambda row: (row["label"], row["metric"]))

with out_csv.open("w", newline="") as f:
    writer = csv.DictWriter(
        f,
        fieldnames=["label", "metric", "count", "mean", "median", "min", "max", "range", "stddev"],
    )
    writer.writeheader()
    writer.writerows(rows)
PY

echo "Wrote ${OUT_CSV}"
