namespace TractorNavigation;

/// <summary>Metrics over every recorded state observation, including the initial state.</summary>
public sealed record EvaluationMetrics(
    double RmsCrossTrackErrorMeters,
    double MaxAbsoluteCrossTrackErrorMeters,
    double ElapsedSeconds,
    double PathProgressMeters,
    double FinalEndpointDistanceMeters,
    int SteeringSaturationCount,
    int RateLimitCount);
