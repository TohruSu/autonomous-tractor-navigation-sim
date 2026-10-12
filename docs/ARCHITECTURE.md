# Geometry, control and simulation

## Coordinates and route model

Distances are metres, time seconds, angles radians. Vehicle position is the **rear axle center**, heading zero points along +x, positive heading is counterclockwise, and speed is a nonnegative constant. These are synthetic model conventions, not measurements of a tractor.

A route owns a copy of finite vertices, strictly positive segment lengths, and cumulative distances. It contains no mutable nearest-point cache. Global projection evaluates each segment with `t = clamp(dot(query-start, end-start)/length², 0, 1)`. Ties select the earliest segment. Left of the direction of travel has positive error: `e = cross(unit_tangent, query-projection)`. `AtDistance` clamps to the route and interpolates on a segment; tangents are chord headings. Angles normalize to `[-pi, pi)`.

Straight routes have exact endpoints. An arc begins at the origin with heading zero. For positive travelled angle `phi`, radius `R` and turn sign `sigma`, its sampled vertices are `(R sin(phi), sigma R (1-cos(phi)))`. Sampling includes the exact endpoint; route length is the sum of chords, which approximates circular arc length. The catalog uses 0.05 m maximum nominal spacing, 12 m straights and radius 6 m quarter turns.

## Controllers

Each controller returns a mathematical steering request. Steering angle and rate constraints belong to the shared simulator so every configuration faces the same constraints.

Stanley projects the **front axle** `rear + wheelbase * (cos(heading), sin(heading))`. With route tangent heading `theta`, its request is:

```text
delta = normalize(theta - heading) + atan2(-gain * e_front, speed + softening)
```

The negative cross-track term corrects left-positive displacement by steering right. Softening is positive, gain positive, and there is no curvature feedforward. Evaluation still uses the rear axle for consistent comparisons between controllers.

Pure Pursuit projects the rear axle and selects a target at `projected_distance + base_lookahead + speed_gain * speed`, clamped to the route endpoint. Transform that target into the vehicle frame; with lateral coordinate `y_local` and squared Euclidean target distance `d²`:

```text
delta = atan2(2 * wheelbase * y_local, d²)
```

A coincident target returns zero. Both controllers reject invalid inputs; their requests are not clipped internally. Algorithm references and adaptation boundaries are in `PROVENANCE.md`.

## Dynamics and run lifecycle

The constant-speed bicycle model uses explicit Euler:

```text
x_next       = x + speed * cos(heading) * dt
y_next       = y + speed * sin(heading) * dt
heading_next = normalize(heading + speed / wheelbase * tan(delta_applied) * dt)
```

Default wheelbase is 1 m, speed 1 m/s, timestep 0.02 s, duration 25 s, steering magnitude limit 0.6 rad, steering rate limit 1.2 rad/s, and endpoint tolerance 0.25 m. These are declared demonstration parameters, not inferred real vehicle specifications. Initial heading and applied steering are zero. Offset cases start 0.5 m left or right.

The runner clamps the requested angle, then limits its change from the previous applied angle. It counts angle saturation and rate limiting separately. Time is `step_index * dt`; it executes at most `floor(duration / dt)` steps, with no fractional final step. Completion requires projected rear-axle progress at least `route.Length - tolerance` **and** Euclidean rear-axle endpoint distance at most tolerance. It never snaps the vehicle onto the endpoint.

Samples include t=0 and successful post-step states. A post-step sample's request was computed at the previous state and its applied angle acted over that interval; its error/progress describe the new state. Controller failures retain the failed attempt, integer step/time and nonfinite value or exception in the result's reason, without duplicating the previous state into RMS statistics. An obtainable nonfinite integration state is retained as a diagnostic sample with undefined geometric fields.

Statuses are `Completed`, `TimedOut`, `NonFiniteValue` and `ControllerError`. Invalid configuration/initial state is rejected. Ordinary controller failures and numerical failures are explicit results. Resource guards reject excessive route sampling, more than one million integration steps per run, and more than two million total recorded samples in one CLI comparison. No failed run is excluded from comparison/export.

## Metrics and CSV

Let `e_i` be the rear-axle signed perpendicular error for each recorded state, including t=0 and the last state. All catalog samples have equal time spacing:

- RMS cross-track error: `sqrt(sum(e_i²)/N)`, computed with scaled arithmetic to avoid avoidable overflow.
- Maximum absolute error: `max(abs(e_i))`.
- Elapsed time, projected progress and endpoint distance: values at the final recorded state.
- Angle saturation and rate-limit counts: steps where the request exceeded the corresponding shared limit.

Errors measure perpendicular displacement from the route's tangent, including at clamped endpoint projections; completion separately uses endpoint distance. An error metric alone cannot prove route completion. Failed-controller runs report metrics over their recorded state observations. Invalid recorded errors yield explicit `NaN` metrics; values are never filtered to make an aggregate pass.

The CLI writes one summary row per requested run and all its recorded states to a second CSV. It records settings and parameterized controller names, uses invariant round-trip numbers, standard CSV escaping, UTF-8 without BOM, and LF endings. A simulation failure yields exit 2 after export; argument and output failures have separate exit codes.

## Boundaries

Core is a library with no file/network IO. CLI owns export. Tests independently check mathematical cases and predeclared functional budgets. The library's global closest-segment method has linear cost and may be ambiguous on self-intersecting or looping routes; such courses are not evaluated here.

This model omits dynamic vehicle physics, sensors, localization, obstacles, terrain, latency and actuators. Constant-speed route tracking is not a complete autonomy stack. Repeatability is checked on the same SDK/runtime/architecture; bit-identical floating-point results across all machines are not claimed.
