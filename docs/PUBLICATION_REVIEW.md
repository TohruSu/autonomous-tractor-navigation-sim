# Validation record and project scope

This record documents automated checks, reproducible simulation results and the scope of the standalone C# toolkit. Source, tests and documentation are released under the [MIT License](../LICENSE); algorithm attribution is recorded in [Provenance](PROVENANCE.md).

## Validation Environment

[GitHub Actions run 38292295510](https://github.com/TohruSu/autonomous-tractor-navigation-sim/actions/runs/38292295510) passed at commit `d02d02be91c2f85c5a415966e651b15675c076cf` on Ubuntu 24.04 x86-64 with .NET SDK 10.0.401 and runtime 10.0.12. Its logs confirm the build, tests, complete comparisons and reproduction checks below.

The workflow runs `bash scripts/validate.sh` and separately regenerates and compares the tracked SVG. Dependencies are supplied by the SDK, and `NuGet.Config` disables external package sources.

## Automated Tests

- **61 passed, 0 failed, 0 skipped** in the deterministic console test harness.
- Offline restore and Release solution build succeeded with **0 warnings and 0 errors**.
- Nine invalid or unsupported CLI cases returned exit 64, including duplicate/unknown arguments, existing output, invalid numbers and excessive comparison allocation.
- Deliberate stationary timeout and extreme-speed numerical failure exported explicit statuses and returned exit 2. The numerical-failure checks preserve Infinity in the candidate state and NaN in undefined metrics.

The harness also covers stationary, insufficient-horizon and infeasible-route timeouts, nonfinite requests, throwing controllers and nonfinite integration. These expected outcomes are checked by passing tests. Numerical regressions cover timestamp overflow, zero-yaw unused arithmetic overflow, sub-step duration and extremely unequal coordinate scales.

Acceptance budgets were fixed before execution in [VALIDATION_PLAN.md](VALIDATION_PLAN.md). Controller defaults and tracking acceptance budgets remain unchanged.

## Simulation Results

Two complete comparisons each finished **28/28** runs: seven routes against four controller configurations. The maximum RMS below is across all seven routes for each configuration, including initial displaced samples; it is not a general ranking or a hardware measurement.

| Configuration | Completed | Largest RMS error | Largest absolute error |
| --- | ---: | ---: | ---: |
| Stanley gain 1, softening 0.5 | 7/7 | 0.236276 m | 0.500000 m |
| Pure Pursuit lookahead 1, speed gain 0.3 | 7/7 | 0.170155 m | 0.500000 m |
| Stanley gain 0.5, softening 0.5 | 7/7 | 0.284152 m | 0.500000 m |
| Pure Pursuit lookahead 2, speed gain 0.3 | 7/7 | 0.213070 m | 0.500000 m |

No nominal comparison failed. Status and failure reason remain part of every exported run.

## Reproducibility

Repeated comparisons produced byte-identical `summary.csv` and `samples.csv`. The visualization step regenerated `docs/images/tracking-comparison.svg` from the simulator's nominal `left-turn-offset` CSV results and compared it exactly with the tracked figure. The reference arc uses the documented analytic route equation, and `scripts/plot_trajectories.py` uses Python's standard library without third-party plotting dependencies.

Additional local validation used Debian x86-64 with the same SDK/runtime:

- Full CLI comparisons each recorded **14,342 state observations**.
- A literal 49-file allowlisted export built and ran the complete validation workflow outside the source checkout, with fresh build outputs, a new CLI home and an empty package cache. All project references resolved inside the export; the check reused the installed, checksum-verified SDK.
- The export helper accepted a trailing slash, rejected a symlinked source ancestor and cleaned its temporary directory.

These export checks and the observation count are local results. [EXPORT.md](EXPORT.md) documents the export procedure, and `PUBLIC_EXPORT.txt` defines the file allowlist. Generated CSVs, binaries and caches are excluded.

## Scope and Limitations

The constant-speed, planar, no-slip kinematic model uses sampled polylines, ideal state knowledge and deterministic steering limits. It omits tire/soil forces, slopes, actuator dynamics, sensing, latency, localization, collision checking and obstacle planning. The global nearest-point search is intended for simple nonintersecting routes. Steering feasibility does not guarantee route completion; statuses must be considered alongside error metrics.

The scenarios use generated geometry without imported coordinates. No Unity/AR, sensor, GNSS/UDP, actuator or network interfaces are required or bundled. No optional custom controller is included.

The results do not establish real-vehicle performance or safety, physical dynamics, obstacle-avoidance capability or field-measured accuracy. Comparisons cover the declared routes and parameters and do not establish general controller superiority. Floating-point byte identity is checked within the same SDK/runtime and architecture, not promised across all platforms. Numerical failure remains an explicit result.
