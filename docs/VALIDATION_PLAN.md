# Navigation simulator validation plan

This plan was fixed before any simulator execution or measured result was inspected. Tests use freshly implemented mathematical definitions and synthetic geometry; historical teaching, Unity, and sandbox source and results are not inputs. These are functional acceptance budgets for this kinematic demonstration, not claims about hardware, field navigation, or controller optimality. Acceptance limits must not be adjusted to fit observed runs.

## Fixed nominal acceptance budgets

Run each default controller (Stanley gain 1, speed softening 0.5; Pure Pursuit lookahead 1 m, speed gain 0.3 s) on the seven catalog scenarios. Speed is 1 m/s, wheelbase is 1 m, steering limit is 0.6 rad, steering rate limit is 1.2 rad/s, fixed step is 0.02 s, duration is 25 s, and endpoint tolerance is 0.25 m.

| Route family | RMS cross-track error | Maximum absolute cross-track error | Endpoint distance | Completion time |
| --- | --- | --- | --- | --- |
| 12 m straight, including mirrored initial offsets of 0.5 m | <= 0.35 m | <= 0.65 m | <= 0.25 m | <= 25 s |
| Radius 6 m quarter arcs, including mirrored initial offsets of 0.5 m | <= 0.60 m | <= 0.90 m | <= 0.25 m | <= 25 s |

The initial offset alone establishes a 0.5 m peak error in offset cases. The peak budgets allow a modest transient above that initial geometry; RMS budgets require convergence through most of each route. Quarter arcs require approximately `atan(1/6) = 0.1651 rad` of bicycle steering, comfortably inside the 0.6 rad bound. A 1 m/s vehicle needs about 12 s for the straight and 9.42 s for a quarter arc, so the 25 s budget leaves time for transient convergence without accepting indefinite wandering. These reasons precede measurement; the budgets do not describe empirical tracking precision.

Alternate controller settings are informative runs, not replacements for default-controller acceptance. Failed, time-limited, stationary, and infeasible runs remain visible with their recorded samples, status, reason, and metrics. No threshold is obtained by discarding failed samples or picking a favorable run.

## Mathematical and behavioral checks

The dependency-free .NET 10 console harness reports each case and total passed/failed counts, continues after every case failure, and returns a nonzero exit code on any failure. It tests:

- Vector arithmetic, dot and cross products, finite-value detection, and angle normalization boundaries and full turns.
- Analytic projection onto straight and cornered polylines, signed error on either side, global nearest-segment choice, deterministic earliest-segment ties, finite input checks, distance clamping, chord tangents, copied immutable route data, and exact line/arc endpoints and sampling.
- Stanley's front-axle projection, left/right corrective steering, reference formula, and zero-error behavior; Pure Pursuit's rear projection, vehicle-frame reference formula, mirrored signs, zero-error behavior, and coincident target behavior. Controllers return their mathematical requests without clipping.
- Explicit-Euler bicycle one-step position and heading, straight-motion and constant-speed invariants, mirrored turning, normalized heading, and invalid inputs.
- Default-controller nominal scenarios against the fixed budgets; mirrored scenarios against independent reflected states and steering; bounded applied steering and steering increments; positive evidence of angle saturation and rate limiting when commands demand them.
- Completion requires both route progress and endpoint proximity, retains the final unmodified integrated state, and uses integer-step timestamps.
- Stationary, insufficient-duration, and geometrically infeasible runs must remain timed out. Nonfinite and throwing controller requests produce explicit failed results with a useful reason. Initial-invalid configuration and state are rejected.
- Exact repeated-run equality on the same runtime, and independently recomputed RMS, maximum absolute error, endpoint distance, elapsed time, progress, saturation count, and rate-limit count from recorded samples and analytic polyline projection. Extremely large finite errors require finite robust RMS; failures cannot silently filter samples.

The root workflow separately checks CLI arguments, exports, and deterministic file contents. Build and test commands and counts will be reported after execution; this plan does not claim unrun checks passed.
