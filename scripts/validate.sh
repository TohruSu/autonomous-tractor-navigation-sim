#!/usr/bin/env bash
set -euo pipefail
project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$project_root"
dotnet restore NavigationSim.slnx --configfile NuGet.Config --locked-mode
dotnet build NavigationSim.slnx -c Release --no-restore -maxcpucount:2
dotnet run --project tests/TractorNavigation.Tests -c Release --no-build

mkdir -p artifacts
validation_dir=$(mktemp -d "$project_root/artifacts/validation.XXXXXXXX")
cli=(dotnet run --project src/TractorNavigation.Cli -c Release --no-build --)
"${cli[@]}" simulate --output "$validation_dir/first"
"${cli[@]}" simulate --output "$validation_dir/second"
cmp "$validation_dir/first/summary.csv" "$validation_dir/second/summary.csv"
cmp "$validation_dir/first/samples.csv" "$validation_dir/second/samples.csv"

expect_exit() {
  local expected_status=$1
  shift
  local actual_status=0
  "$@" > "$validation_dir/last-command.log" 2>&1 || actual_status=$?
  if [[ "$actual_status" != "$expected_status" ]]; then
    cat "$validation_dir/last-command.log"
    printf 'Expected exit %s, got %s\n' "$expected_status" "$actual_status" >&2
    return 1
  fi
}

# Every unfinished run must be written and yield the documented failure status.
expect_exit 2 "${cli[@]}" simulate --scenario straight --controller stanley \
  --speed 0 --duration 0.1 --output "$validation_dir/stationary"
grep -q ',TimedOut,' "$validation_dir/stationary/summary.csv"
expect_exit 2 "${cli[@]}" simulate --scenario straight --controller stanley \
  --speed 1.7976931348623157E308 --output "$validation_dir/numerical-failure"
grep -q ',NonFiniteValue,' "$validation_dir/numerical-failure/summary.csv"
grep -q 'Infinity' "$validation_dir/numerical-failure/samples.csv"
grep -q 'NaN' "$validation_dir/numerical-failure/summary.csv"
expect_exit 64 "${cli[@]}" simulate --output "$validation_dir/first"
expect_exit 64 "${cli[@]}" simulate --scenario unknown --output "$validation_dir/unknown"
expect_exit 64 "${cli[@]}" simulate --speed NaN --output "$validation_dir/nonfinite"
expect_exit 64 "${cli[@]}" simulate --step 0 --output "$validation_dir/zero-step"
expect_exit 64 "${cli[@]}" simulate --controller unknown --output "$validation_dir/bad-controller"
expect_exit 64 "${cli[@]}" simulate --speed 1 --speed 2 --output "$validation_dir/duplicate"
expect_exit 64 "${cli[@]}" simulate --output
expect_exit 64 "${cli[@]}" simulate --bogus value --output "$validation_dir/unknown-option"
expect_exit 64 "${cli[@]}" simulate --step 0.000025 --output "$validation_dir/excessive-comparison"
printf 'CLI checks and byte-identical CSV reproducibility passed. Results: %s\n' "$validation_dir"
