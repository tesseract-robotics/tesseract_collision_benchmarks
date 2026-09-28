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
  echo "FAIL: $*"
  failures=$((failures + 1))
}
pass() { echo "ok: $1"; }

# Asserts the binary refuses an argument list and says why. The pattern must appear in its output.
# The run is narrowed to one manager and one test type first, so a guard that regresses costs one
# short benchmark and a readable failure rather than a full run into the ctest timeout.
expect_reject() {
  local desc="$1" pattern="$2"
  shift 2
  local out status
  out="$("$BIN" "$WORK/reject.csv" --mode discrete --test-type first --manager CoalDiscreteBVHManager "$@" 2>&1)"
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
expect_reject "--margin at the 1e6 cap is refused" "below 1e6 metres" --margin 1e6
expect_reject "trailing junk in --waypoints is refused" "--waypoints expects an integer" --waypoints 10x
expect_reject "zero --waypoints is refused" "must be at least 1" --waypoints 0
expect_reject "negative --waypoints is refused" "must be at least 1" --waypoints -3
expect_reject "negative --seed is refused" "--seed must be non-negative" --seed -5
expect_reject "--manager naming only the other mode's managers is refused" \
  "in --mode discrete. Available managers: BulletDiscreteBVHManager" --manager BulletCastBVHManager

expect_reject "--waypoints without trajectory is refused" \
  "--waypoints applies only to --duty-cycle trajectory" --waypoints 10
expect_reject "--waypoints with an explicit sweep is refused" \
  "--waypoints applies only to --duty-cycle trajectory" --duty-cycle sweep --waypoints 10
expect_reject "--waypoints under a clone-implied repeat is refused" \
  "--waypoints applies only to --duty-cycle trajectory" --clone --waypoints 7
expect_reject "indivisible --waypoints under trajectory is still refused" \
  "does not divide the 1000 trials" --duty-cycle trajectory --waypoints 7

# The remaining cases share one sweep and one trajectory run. The two are independent and no case
# asserts on timing, so they run concurrently.
continuous_run() {
  local csv="$1"
  shift
  "$BIN" "$csv" --mode continuous --test-type first --manager CoalCastBVHManager --seed 42 "$@" >"$csv.log" 2>&1
}
continuous_run "$WORK/sweep.csv" &
sweep_pid=$!
continuous_run "$WORK/traj.csv" --duty-cycle trajectory --waypoints 10 &
traj_pid=$!
"$BIN" "$WORK/margin.csv" --mode discrete --test-type first --manager CoalDiscreteBVHManager --seed 42 \
  --margin 0.20000001 >"$WORK/margin.csv.log" 2>&1 &
margin_pid=$!
wait "$sweep_pid"
sweep_status=$?
wait "$traj_pid"
traj_status=$?
wait "$margin_pid"
margin_status=$?

if [[ $sweep_status -ne 0 ]]; then
  fail "sweep continuous run exited $sweep_status. Last line: $(tail -n1 "$WORK/sweep.csv.log")"
fi

# The guard must not over-reject: a divisor of the trial count under trajectory is the one
# combination --waypoints exists for.
if [[ $traj_status -eq 0 ]]; then
  pass "--waypoints with trajectory is accepted"
else
  fail "--waypoints with trajectory is accepted: expected exit 0, got $traj_status." \
    "Last line: $(tail -n1 "$WORK/traj.csv.log")"
fi

# A trajectory run walks interpolations whose collision status was never sampled. It must say how
# many of them collide, because contact_test cost depends on it and the scenario label only ever
# describes the sampled set. Both margins are required: whether a waypoint has any contact at all
# differs between the zero-margin scenarios and the two distance ones.
survey_pattern='Expanded 50 sampled states into 500 trajectory waypoints, [0-9]+ of them in collision'
survey_pattern+=' at margin 0 and [0-9]+ at margin [0-9.]+ m'
if grep -qE "$survey_pattern" "$WORK/traj.csv.log"; then
  pass "trajectory run reports the walked population's collision count at both margins"
else
  fail "trajectory run did not report collision counts for the walked waypoints." \
    "Got: $(grep Expanded "$WORK/traj.csv.log")"
fi

# The scenario column is the join key between runs. It must describe the sampled population, which
# every duty cycle shares, not the population a particular duty cycle walked. Pin the pair count as
# well: two runs that both lost their labels, or both carried the same wrong count, would otherwise
# still compare equal.
sweep_labels="$(csv_column "$WORK/sweep.csv" scenario 2>/dev/null)"
traj_labels="$(csv_column "$WORK/traj.csv" scenario 2>/dev/null)"
if [[ "$(grep -c ', 49 state pairs$' <<<"$sweep_labels")" -ne 4 ]]; then
  fail "sweep continuous labels do not all quote the 49 sampled pairs: $(tr '\n' '|' <<<"$sweep_labels")"
elif [[ "$sweep_labels" != "$traj_labels" ]]; then
  fail "continuous scenario labels differ across duty cycles:
  sweep:      $(tr '\n' '|' <<<"$sweep_labels")
  trajectory: $(tr '\n' '|' <<<"$traj_labels")"
else
  pass "continuous scenario labels match across sweep and trajectory"
fi

# total_num_checks for a sweep continuous run is trials * pairs = 1000 * 49. Pinning it catches a
# refactor that changes how many pairs the run walks.
checks="$(csv_column "$WORK/sweep.csv" total_num_checks 2>/dev/null | sort -u)"
if [[ "$checks" == "49000" ]]; then
  pass "sweep continuous run walks 49 pairs for 1000 trials"
else
  fail "sweep continuous total_num_checks was '$(tr '\n' ',' <<<"$checks")', expected 49000 on every row"
fi

# The margin label is part of the join key, so it must print every digit the margin needs: a margin
# that rounds to the default's label would join as if both runs used the same margin.
margin_labels="$(csv_column "$WORK/margin.csv" scenario 2>/dev/null | grep -F 'Distance (')"
if [[ $margin_status -ne 0 ]]; then
  fail "--margin 0.20000001 run exited $margin_status. Last line: $(tail -n1 "$WORK/margin.csv.log")"
elif [[ "$(grep -cF '(0.20000001 m)' <<<"$margin_labels")" -ne 2 ]]; then
  fail "distance labels do not carry the full margin: $(tr '\n' '|' <<<"$margin_labels")"
else
  pass "distance labels carry every digit of the margin"
fi

if [[ $failures -ne 0 ]]; then
  echo "$failures case(s) failed"
  exit 1
fi
echo "all cases passed"
