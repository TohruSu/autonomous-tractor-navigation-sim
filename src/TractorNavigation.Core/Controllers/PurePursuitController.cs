namespace TractorNavigation;

/// <summary>Rear-axle geometric tracking with a speed-dependent lookahead distance.</summary>
public sealed class PurePursuitController : IPathController
{
    public PurePursuitController(double lookahead = 1.0, double speedGain = 0.3)
    {
        NumericGeometry.RequirePositive(lookahead, nameof(lookahead));
        if (!double.IsFinite(speedGain) || speedGain < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(speedGain), "The speed gain must be finite and nonnegative.");
        }

        Lookahead = lookahead;
        SpeedGain = speedGain == 0 ? 0 : speedGain;
        Name = FormattableString.Invariant($"PurePursuit(lookahead={lookahead:R},speedGain={SpeedGain:R})");
    }

    public double Lookahead { get; }

    public double SpeedGain { get; }

    public string Name { get; }

    public double ComputeSteering(VehicleState state, Route route, double wheelbase)
    {
        ControllerValidation.Validate(state, route, wheelbase);
        var projection = route.Project(state.Position);
        var remaining = route.Length - projection.Distance;
        double targetDistance;
        if (Lookahead >= remaining)
        {
            targetDistance = route.Length;
        }
        else
        {
            var speedLookahead = SpeedGain * state.Speed;
            // An overflowing positive lookahead still has a well-defined clamped target.
            targetDistance = speedLookahead >= remaining - Lookahead
                ? route.Length
                : Math.Min(route.Length, projection.Distance + Lookahead + speedLookahead);
        }

        var target = route.AtDistance(targetDistance).Position;
        var displacement = NumericGeometry.Difference(target, state.Position);
        if (displacement.IsZero)
        {
            return 0;
        }

        var heading = Angles.Normalize(state.Heading);
        var targetY = -Math.Sin(heading) * displacement.X + Math.Cos(heading) * displacement.Y;
        var normalizedDistanceSquared = displacement.X * displacement.X + displacement.Y * displacement.Y;

        // Coulter, "Implementation of the Pure Pursuit Path Tracking Algorithm"
        // (CMU-RI-TR-92-01, 1992): curvature = 2*y / targetDistanceSquared.
        // Cancel the shared coordinate scale before evaluating atan2(2*L*y, d^2).
        var numerator = NumericGeometry.Magnitude.FromDouble(wheelbase).Times(2).Times(Math.Abs(targetY));
        var denominator = NumericGeometry.Magnitude.FromDouble(displacement.Scale)
            .Times(normalizedDistanceSquared).Shift(displacement.Exponent);
        return NumericGeometry.Magnitude.Atan2(numerator, denominator, targetY);
    }
}
