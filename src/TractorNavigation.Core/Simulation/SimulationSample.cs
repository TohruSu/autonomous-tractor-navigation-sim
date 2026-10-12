namespace TractorNavigation;

/// <summary>
/// An initial or post-step rear-axle observation. Post-step steering values belong
/// to the command computed from the preceding state and used over that interval.
/// </summary>
public sealed record SimulationSample(
    int Step,
    double TimeSeconds,
    VehicleState State,
    double RequestedSteeringRadians,
    double AppliedSteeringRadians,
    double CrossTrackErrorMeters,
    double ProgressMeters);
