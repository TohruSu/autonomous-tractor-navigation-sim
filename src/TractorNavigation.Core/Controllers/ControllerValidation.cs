namespace TractorNavigation;

internal static class ControllerValidation
{
    internal static void Validate(VehicleState state, Route route, double wheelbase)
    {
        ArgumentNullException.ThrowIfNull(route);
        NumericGeometry.RequirePositive(wheelbase, nameof(wheelbase));
        if (!state.Position.IsFinite || !double.IsFinite(state.Heading) || !double.IsFinite(state.Speed) || state.Speed < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(state), "Position and heading must be finite, and forward speed must be finite and nonnegative.");
        }
    }
}
