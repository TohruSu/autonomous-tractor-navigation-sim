namespace TractorNavigation;

/// <summary>Fixed-step integration and actuator limits for a single simulation.</summary>
public sealed record SimulationSettings
{
    /// <summary>Bounds both execution time and the size of the recorded trajectory.</summary>
    public const int MaximumSupportedSteps = 1_000_000;

    public double TimeStepSeconds { get; init; }
    public double MaxDurationSeconds { get; init; }
    public double WheelbaseMeters { get; init; }
    public double MaxSteeringRadians { get; init; }
    public double MaxSteeringRateRadiansPerSecond { get; init; }
    public double EndpointToleranceMeters { get; init; }

    public SimulationSettings(
        double TimeStepSeconds = 0.02,
        double MaxDurationSeconds = 25,
        double WheelbaseMeters = 1,
        double MaxSteeringRadians = 0.6,
        double MaxSteeringRateRadiansPerSecond = 1.2,
        double EndpointToleranceMeters = 0.25)
    {
        this.TimeStepSeconds = TimeStepSeconds;
        this.MaxDurationSeconds = MaxDurationSeconds;
        this.WheelbaseMeters = WheelbaseMeters;
        this.MaxSteeringRadians = MaxSteeringRadians;
        this.MaxSteeringRateRadiansPerSecond = MaxSteeringRateRadiansPerSecond;
        this.EndpointToleranceMeters = EndpointToleranceMeters;
        ValidateAndGetStepCount();
    }

    internal int ValidateAndGetStepCount()
    {
        RequirePositiveFinite(TimeStepSeconds, nameof(TimeStepSeconds));
        RequirePositiveFinite(MaxDurationSeconds, nameof(MaxDurationSeconds));
        RequirePositiveFinite(WheelbaseMeters, nameof(WheelbaseMeters));
        RequirePositiveFinite(MaxSteeringRadians, nameof(MaxSteeringRadians));
        RequirePositiveFinite(MaxSteeringRateRadiansPerSecond, nameof(MaxSteeringRateRadiansPerSecond));
        RequirePositiveFinite(EndpointToleranceMeters, nameof(EndpointToleranceMeters));
        if (MaxSteeringRadians >= Math.PI / 2)
        {
            throw new ArgumentOutOfRangeException(nameof(MaxSteeringRadians),
                "The steering limit must be less than pi/2 radians.");
        }

        double stepCount = Math.Floor(MaxDurationSeconds / TimeStepSeconds);
        if (!double.IsFinite(stepCount) || stepCount > MaximumSupportedSteps)
        {
            throw new ArgumentOutOfRangeException(nameof(MaxDurationSeconds),
                $"A simulation may contain at most {MaximumSupportedSteps} integration steps.");
        }

        if (!double.IsFinite(stepCount * TimeStepSeconds))
        {
            throw new ArgumentOutOfRangeException(nameof(TimeStepSeconds),
                "The final integer-step timestamp must be representable as a finite value.");
        }

        return (int)stepCount;
    }

    private static void RequirePositiveFinite(double value, string parameterName)
    {
        if (!double.IsFinite(value) || value <= 0)
        {
            throw new ArgumentOutOfRangeException(parameterName, "The value must be finite and positive.");
        }
    }
}
