# Source provenance and algorithm attribution

All executable source in this standalone project was newly implemented from mathematical definitions with AI assistance. The owner reviewed the implementation and provenance records and approved MIT distribution of the newly implemented standalone source, tests and documentation. No historical teacher example, Unity script, sandbox controller, diagnostic implementation or recorded CSV was extracted into this project. This is not a claim that historical project modules were all independently owned.

Implementation contributors for geometry/controllers, dynamics/evaluation and tests received a mathematical API contract and synthetic acceptance requirements; they were instructed not to inspect or reuse historical source. A separate audit examined historical overlap. This separation supports the provenance and exclusion decisions; it is not a legal certification of ownership.

Useful engineering ideas retained at the design level are explicit axle references, mirrored synthetic scenarios, bounded steering, deterministic tests and CSV comparison. Those ideas are implemented afresh. The historical custom controller was omitted because its independent provenance could not be established from the available mixed-origin source.

## Portfolio export history

This repository starts from an owner-approved file-tree export of the completed standalone toolkit, with an independent Git history. The clean initial commit records preparation for portfolio publication; its author and committer metadata do not establish original individual authorship of any historical university team project. The mathematical attributions and implementation boundaries below remain unchanged. Historical development commits, pull requests, workflow run records and original release metadata were not imported into this history. The former development repository was subsequently deleted, and the `v0.1.0` tag and Release were recreated against the clean history.

This independent history does not erase cached or independently cloned copies of previously public development history.

## Mathematical references

- R. Craig Coulter, **Implementation of the Pure Pursuit Path Tracking Algorithm**, Carnegie Mellon University, CMU-RI-TR-92-01 (1992). [Publication](https://www.ri.cmu.edu/publications/implementation-of-the-pure-pursuit-path-tracking-algorithm/). This project uses a rear-axle target transformed into the vehicle frame and the geometric curvature relationship `kappa = 2 y_target / distance_target²`. Its target is selected by route distance; its base lookahead is extended by speed.
- Sebastian Thrun et al., **Stanley: The robot that won the DARPA Grand Challenge**, Journal of Field Robotics 23(9), 661–692 (2006), [doi:10.1002/rob.20147](https://doi.org/10.1002/rob.20147). This project implements heading correction and front-axle cross-track feedback with positive speed softening. It uses the explicitly documented left-positive error sign and omits curvature feedforward and the competition vehicle's additional compensation.
- Steven M. LaValle, **Planning Algorithms**, Cambridge University Press (2006), Chapter 13, [author's book](https://lavalle.pl/planning/). The planar kinematic bicycle equations are integrated here with fixed-step explicit Euler.
- Polyline projection, cumulative segment distance, angle wrapping, RMS and maximum-error calculations use standard Euclidean and statistical definitions documented in `ARCHITECTURE.md`.

These references acknowledge algorithmic ideas; no source code or figures from them are bundled. Standard algorithm names do not imply reproduction of the published systems.

## File decisions

- All `src/TractorNavigation.Core/**/*.cs`: new geometry, controller, scenario, dynamics and evaluation implementations from the definitions above.
- All `src/TractorNavigation.Cli/**/*.cs`: new argument handling and CSV formatting; no historical logging or device interface.
- All `tests/TractorNavigation.Tests/**/*.cs`: new analytic expectations and synthetic integration checks; no historical test or result baseline.
- Project files, scripts and public documentation: new standalone build/test/export configuration and explanations.
- Teacher/reference modules, historical custom and diagnostic controllers, checkpoint archives, Unity scenes/assets, third-party Unity packages, AR/GNSS/UDP/actuator code and identifying logs: excluded in full.
- Private source-audit and delivery documents: excluded from the standalone project and every export; they are review records, not distributed project inputs.

## License boundary

The root [LICENSE](../LICENSE) contains the standard MIT License with `Copyright (c) 2026 TohruSu`. The owner's approval applies to this standalone project's newly implemented source, tests and documentation. It does not extend to historical Unity source, teacher/reference implementations, mixed-origin controllers, third-party packages or assets, excluded development materials, or ownership of the Stanley and Pure Pursuit algorithms themselves. No historical code has been relicensed.

The references above identify established mathematical ideas, not algorithms invented by the owner. The implementation separation and review support the exclusion decisions; they do not constitute a formal certification of exclusive ownership.
