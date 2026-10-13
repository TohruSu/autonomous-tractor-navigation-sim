# Source provenance and algorithm attribution

## Implementation provenance

All executable source in this standalone project was newly implemented from mathematical definitions with AI assistance. Geometry, controllers, dynamics, evaluation and tests were developed for this toolkit using a mathematical API contract and synthetic acceptance requirements. Historical teacher/reference examples, Unity scripts, sandbox controllers, diagnostic implementations and recorded CSVs were not reused as source or result baselines.

Design-level ideas include explicit axle references, mirrored synthetic scenarios, bounded steering, deterministic tests and CSV comparison. The historical custom controller was omitted because its independent provenance could not be established from the available mixed-origin source.

## Mathematical references

- R. Craig Coulter, **Implementation of the Pure Pursuit Path Tracking Algorithm**, Carnegie Mellon University, CMU-RI-TR-92-01 (1992). [Publication](https://www.ri.cmu.edu/publications/implementation-of-the-pure-pursuit-path-tracking-algorithm/). This project uses a rear-axle target transformed into the vehicle frame and the geometric curvature relationship `kappa = 2 y_target / distance_target²`. Its target is selected by route distance; its base lookahead is extended by speed.
- Sebastian Thrun et al., **Stanley: The robot that won the DARPA Grand Challenge**, Journal of Field Robotics 23(9), 661–692 (2006), [doi:10.1002/rob.20147](https://doi.org/10.1002/rob.20147). This project implements heading correction and front-axle cross-track feedback with positive speed softening. It uses the explicitly documented left-positive error sign and omits curvature feedforward and the competition vehicle's additional compensation.
- Steven M. LaValle, **Planning Algorithms**, Cambridge University Press (2006), Chapter 13, [author's book](https://lavalle.pl/planning/). The planar kinematic bicycle equations are integrated here with fixed-step explicit Euler.
- Polyline projection, cumulative segment distance, angle wrapping, RMS and maximum-error calculations use standard Euclidean and statistical definitions documented in `ARCHITECTURE.md`.

These references acknowledge algorithmic ideas; no source code or figures from them are bundled. Standard algorithm names do not imply reproduction of the published systems.

## Implementation layout

- `src/TractorNavigation.Core/**/*.cs`: geometry, controller, scenario, dynamics and evaluation implementations from the definitions above.
- `src/TractorNavigation.Cli/**/*.cs`: argument handling and CSV formatting, with no historical logging or device interface.
- `tests/TractorNavigation.Tests/**/*.cs`: analytic expectations and synthetic integration checks, with no historical test or result baseline.
- Project files, scripts and documentation: standalone build, validation, export configuration and explanations.

## License

The root [LICENSE](../LICENSE) contains the standard MIT License with `Copyright (c) 2026 TohruSu`, covering this standalone project's newly implemented source, tests and documentation. It does not extend to excluded historical code, third-party packages or assets, or ownership of the established mathematical algorithms. The cited ideas predate this implementation and remain attributed to their published sources.
