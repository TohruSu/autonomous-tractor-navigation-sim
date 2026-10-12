namespace TractorNavigation;

/// <summary>Front-axle heading and cross-track feedback without curvature feedforward.</summary>
public sealed class StanleyController : IPathController
{
    public StanleyController(double gain = 1.0, double speedSoftening = 0.5)
    {
        NumericGeometry.RequirePositive(gain, nameof(gain));
        NumericGeometry.RequirePositive(speedSoftening, nameof(speedSoftening));
        Gain = gain;
        SpeedSoftening = speedSoftening;
        Name = FormattableString.Invariant($"Stanley(gain={gain:R},speedSoftening={speedSoftening:R})");
    }

    public double Gain { get; }

    public double SpeedSoftening { get; }

    public string Name { get; }

    public double ComputeSteering(VehicleState state, Route route, double wheelbase)
    {
        ControllerValidation.Validate(state, route, wheelbase);
        var heading = Angles.Normalize(state.Heading);
        var frontAxle = state.Position + new Vector2D(Math.Cos(heading), Math.Sin(heading)) * wheelbase;
        if (!frontAxle.IsFinite)
        {
            throw new OverflowException("The front axle position cannot be represented with finite coordinates.");
        }

        var projection = route.Project(frontAxle);
        var headingError = Angles.Normalize(projection.TangentHeading - heading);

        // Original implementation of the Stanley feedback equation described in
        // Thrun et al., "Stanley: The Robot that Won the DARPA Grand Challenge" (2006).
        // Our left-positive error requires the negative correction sign.
        var numerator = NumericGeometry.Magnitude.FromDouble(Math.Abs(projection.SignedCrossTrackError)).Times(Gain);
        var denominator = NumericGeometry.Magnitude.Sum(state.Speed, SpeedSoftening);
        var crossTrackCorrection = NumericGeometry.Magnitude.Atan2(numerator, denominator, -projection.SignedCrossTrackError);
        return headingError + crossTrackCorrection;
    }
}
