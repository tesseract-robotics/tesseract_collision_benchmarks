#!/usr/bin/env bash
# Contract tests for the tesseract_collision_benchmarks_only CLI.
#
# Each case asserts on something a caller observes: an exit status, a diagnostic, a scenario
# label or a CSV cell. The real binary is run rather than an internal function, because the
# contract being pinned is what a caller sees.
#
# Benchmark runs are slow (the trial count is compiled in), so cases that need a full run are
# held to one manager and one contact test type.
set -uo pipefail

BIN="${1:?usage: cli_contract.bash <path-to-tesseract_collision_benchmarks_only>}"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

failures=0
fail() {
  echo "FAIL: $1"
  failures=$((failures + 1))
}
pass() { echo "ok: $1"; }

# Asserts the binary refuses an argument list and says why. The pattern must appear in its output.
expect_reject() {
  local desc="$1" pattern="$2"
  shift 2
  local out status
  out="$("$BIN" "$WORK/reject.csv" "$@" 2>&1)"
  status=$?
  if [[ $status -eq 0 ]]; then
    fail "$desc: expected a non-zero exit status, got 0"
    return
  fi
  if ! grep -qF -- "$pattern" <<<"$out"; then
    fail "$desc: no diagnostic matching '$pattern'. Last line: $(tail -n1 <<<"$out")"
    return
  fi
  pass "$desc"
}

# Asserts the binary accepts an argument list far enough to start benchmarking.
expect_accept() {
  local desc="$1"
  shift
  local out status
  out="$("$BIN" "$WORK/accept.csv" "$@" 2>&1)"
  status=$?
  if [[ $status -ne 0 ]]; then
    fail "$desc: expected exit 0, got $status. Last line: $(tail -n1 <<<"$out")"
    return
  fi
  pass "$desc"
}

# Runs a benchmark and echoes its combined output, for cases that assert on labels or the CSV.
run_bench() {
  local csv="$1"
  shift
  "$BIN" "$csv" --mode discrete --test-type first --manager CoalDiscreteBVHManager --seed 42 "$@" 2>&1
}

# Echoes one named column of a CSV, one value per row, in row order. The scenario column is quoted
# and contains a comma, so cut -d, and awk -F, read the column to its right; only a real CSV parser
# gets the field the header names.
csv_column() {
  local csv="$1" column="$2"
  python3 - "$csv" "$column" <<'PY'
import csv, sys

with open(sys.argv[1], newline="") as handle:
    for row in csv.DictReader(handle):
        print(row[sys.argv[2]])
PY
}

expect_reject "unknown option is refused" "Unknown option" --not-a-flag

expect_reject "non-numeric --margin is refused" "--margin expects a number" --margin abc
expect_reject "non-numeric --waypoints is refused" "--waypoints expects an integer" --waypoints abc
expect_reject "non-numeric --seed is refused" "--seed expects an integer" --seed notanumber
expect_reject "out-of-range --margin is refused" "--margin expects a number" --margin 1e999
expect_reject "trailing junk in --waypoints is refused" "--waypoints expects an integer" --waypoints 10x
expect_reject "zero --waypoints is refused" "must be at least 1" --waypoints 0
expect_reject "negative --waypoints is refused" "must be at least 1" --waypoints -3

expect_reject "--waypoints without trajectory is refused" \
  "--waypoints applies only to --duty-cycle trajectory" --waypoints 10
expect_reject "--waypoints with an explicit sweep is refused" \
  "--waypoints applies only to --duty-cycle trajectory" --duty-cycle sweep --waypoints 10
expect_reject "--waypoints under a clone-implied repeat is refused" \
  "--waypoints applies only to --duty-cycle trajectory" --clone --waypoints 7
expect_reject "indivisible --waypoints under trajectory is still refused" \
  "does not divide the 1000 trials" --duty-cycle trajectory --waypoints 7

# The guard must not over-reject: a divisor of the trial count under trajectory is the one
# combination --waypoints exists for. This case runs a full benchmark, so keep it to one manager.
expect_accept "--waypoints with trajectory is accepted" \
  --mode discrete --test-type first --manager CoalDiscreteBVHManager --seed 42 \
  --duty-cycle trajectory --waypoints 10

# The scenario column is the join key between runs. It must describe the sampled population, which
# every duty cycle shares, not the population a particular duty cycle walked.
sweep_out="$("$BIN" "$WORK/sweep.csv" --mode continuous --test-type first \
  --manager CoalCastBVHManager --seed 42 2>&1)"
traj_out="$("$BIN" "$WORK/traj.csv" --mode continuous --test-type first \
  --manager CoalCastBVHManager --seed 42 --duty-cycle trajectory --waypoints 10 2>&1)"

sweep_labels="$(grep -o 'Continuous: [^"]*state pairs' <<<"$sweep_out" | sort -u)"
traj_labels="$(grep -o 'Continuous: [^"]*state pairs' <<<"$traj_out" | sort -u)"
if [[ "$sweep_labels" == "$traj_labels" ]]; then
  pass "continuous scenario labels match across sweep and trajectory"
else
  fail "continuous scenario labels differ across duty cycles:
  sweep:      $(tr '\n' '|' <<<"$sweep_labels")
  trajectory: $(tr '\n' '|' <<<"$traj_labels")"
fi

# A trajectory run walks interpolations whose collision status was never sampled. The run must say
# how many of them collide, because contact_test cost depends on it and the scenario label only
# ever describes the sampled set. Both margins are required: whether a waypoint has any contact at
# all differs between the zero-margin scenarios and the two distance ones.
traj_survey="$(run_bench "$WORK/survey.csv" --duty-cycle trajectory --waypoints 10 2>&1)"
if grep -qE 'Expanded 50 sampled states into 500 trajectory waypoints, [0-9]+ of them in collision at margin 0 and [0-9]+ at margin [0-9.]+ m' \
  <<<"$traj_survey"; then
  pass "trajectory run reports the walked population's collision count at both margins"
else
  fail "trajectory run did not report collision counts for the walked waypoints. Got: $(grep Expanded <<<"$traj_survey")"
fi

if [[ $failures -ne 0 ]]; then
  echo "$failures case(s) failed"
  exit 1
fi
echo "all cases passed"
