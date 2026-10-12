namespace TractorNavigation;

/// <summary>Rear-axle position, counterclockwise heading from +x, and forward speed.</summary>
public readonly record struct VehicleState(Vector2D Position, double Heading, double Speed);
