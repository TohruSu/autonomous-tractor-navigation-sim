# Validation record and project scope

This standalone C# simulation toolkit retains the owner-approved [MIT License](../LICENSE). This record preserves the testing methodology, recorded results, validation provenance and limitations. Synthetic simulation results do not establish real vehicle performance or safety.

## Recorded local validation — Codex-reported

The following local results were reported by Codex and reviewed by the owner. The recorded validation on 2026-10-12 used Debian x86-64, .NET SDK 10.0.401 / runtime 10.0.12:

- **61 passed, 0 failed, 0 skipped** in the deterministic console test harness.
- Offline restore and Release solution build succeeded with **0 warnings and 0 errors**.
- All **28/28** comparison runs completed: seven synthetic routes against four controller configurations.
- Two full CLI runs produced byte-identical `summary.csv` and `samples.csv`; each retained **14,342 state observations**.
- Deliberate stationary timeout and extreme-speed numerical failure exported explicit statuses and returned exit 2. The numerical-failure export visibly retained Infinity in its candidate state and NaN in undefined metrics.
- Nine invalid/unsupported CLI cases returned exit 64, including duplicate/unknown arguments, existing output, invalid numbers and excessive comparison allocation.
- A literal allowlisted export built and ran the same full workflow outside the source checkout, with no parent repository, fresh build outputs, a new CLI home and an empty package cache. The SDK itself was the previously installed, checksum-verified SDK; this is not a claim of a newly provisioned machine.
- Public file scanning found no credential patterns, IP addresses, private repository identifiers, external source links or package references. All project references resolve inside the export.
- Export with a trailing slash worked; a symlinked source ancestor was rejected and the helper cleaned its own temporary directory.

Test budgets were fixed before execution in `VALIDATION_PLAN.md`; its initial SHA-256 was `c36ee085b1ab3e6a0b5a3ee9ad1f8d3ee5ce4b6bc09310b61d6ad3cef5cbcc8f` and remained unchanged. Numerical regressions were added for discovered timestamp overflow, zero-yaw unused arithmetic overflow, sub-step duration, and extremely unequal coordinate scales. Controller defaults and tracking acceptance budgets were unchanged.

## Recorded hosted validation — independently reviewed

The earlier private integration's GitHub-hosted push and pull-request workflows both completed successfully. Independent review of run/job records **and raw hosted logs** confirmed 61 passing tests, 28/28 completed comparisons, zero build warnings/errors, and byte-identical CSV checks. These hosted outcomes are distinct from the original Codex-reported local validation.

The original standalone repository's root GitHub Actions workflow separately completed successfully before this portfolio export. Independent raw-log review confirmed 61/61 tests, two completed 28/28 comparisons, byte-identical repeated CSVs, and zero warnings/errors. These historical source-validation records and their original commit and run identifiers are retained in the owner's development archive rather than imported into this independent history.

This repository's own [GitHub Actions run 38285188430](https://github.com/TohruSu/autonomous-tractor-navigation-sim/actions/runs/38285188430) completed successfully at clean commit `f9d4a9a0824c686507e0688ba223fcae619069ba`. Its complete validation passed **61/61 deterministic tests**, two completed **28/28** comparisons, and byte-identical repeated `summary.csv` and `samples.csv`. The independent visualization step regenerated the synthetic SVG from simulation CSVs and verified it matched the checked-in diagram byte-for-byte. These results are supported by the successful run/job/step conclusions, fixed test registration and retained validation assertions. Consult this repository's Actions tab for subsequent runs.

## Comparison outcome

The maximum RMS below is across all seven routes for each configuration, including initial displaced samples; it is not a general ranking or a hardware measurement.

| Configuration | Completed | Largest RMS error | Largest absolute error |
| --- | ---: | ---: | ---: |
| Stanley gain 1, softening 0.5 | 7/7 | 0.236276 m | 0.500000 m |
| Pure Pursuit lookahead 1, speed gain 0.3 | 7/7 | 0.170155 m | 0.500000 m |
| Stanley gain 0.5, softening 0.5 | 7/7 | 0.284152 m | 0.500000 m |
| Pure Pursuit lookahead 2, speed gain 0.3 | 7/7 | 0.213070 m | 0.500000 m |

No nominal comparison failed. Tests also verified retained stationary, insufficient-horizon and infeasible-route timeouts, nonfinite requests, throwing controllers and nonfinite integration. These are expected failure cases, distinct from the 61 passing test cases.

## Source and scope review

- Historical files remain untouched. The new library/project references are wholly inside this directory; external package sources are empty.
- Source and tests are new implementations of acknowledged mathematical definitions. Historical teacher/reference/mixed-origin source, custom controllers, checkpoints, source archives and CSV baselines are excluded.
- Synthetic scenarios contain no imported coordinates. No Unity/AR, sensor, GNSS/UDP, actuator or network interfaces are required or bundled.
- `PUBLIC_EXPORT.txt` is a literal 49-file allowlist, including the approved MIT `LICENSE` and root CI workflow. It excludes private audit/delivery documents, historical material, generated CSV outputs, `bin/`, `obj/` and caches.
- Historical source comparisons, provenance uncertainties and exclusion decisions were reviewed separately. Private audit/delivery records are not distributed project files or build inputs.

## License and simulation limitations

The synthetic README figure at `docs/images/tracking-comparison.svg` is generated from the same simulator's nominal `left-turn-offset` CSV results, with the documented analytic reference arc. No historical coordinates or third-party plotting dependencies are included. Its deterministic generator is `scripts/plot_trajectories.py`.

The root [LICENSE](../LICENSE) is the standard MIT License with `Copyright (c) 2026 TohruSu`. Approval covers the newly implemented standalone source, tests and documentation; it does not license excluded historical materials, third-party components or the established algorithms themselves. Attribution to Coulter, Thrun et al. and LaValle is retained in [provenance](PROVENANCE.md).

No optional custom controller is included. The constant-speed kinematic simulation establishes no physical vehicle safety, obstacle-avoidance capability, sensor/GNSS integration, dynamic physics or field-measured accuracy. Comparisons cover the declared synthetic routes and parameters and do not establish general controller superiority. Floating-point byte identity is checked within the same SDK/runtime and architecture, not promised across all platforms.
