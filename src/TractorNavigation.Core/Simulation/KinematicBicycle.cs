using System.Globalization;

namespace TractorNavigation;

/// <summary>An explicit Euler step for a forward-speed, rear-axle bicycle model.</summary>
public static class KinematicBicycle
{
    public static VehicleState Step(
        VehicleState state,
        double steeringRadians,
        double wheelbase,
        double timeStep)
    {
        ValidateState(state, nameof(state));
        if (!double.IsFinite(steeringRadians) || Math.Abs(steeringRadians) >= Math.PI / 2)
        {
            throw new ArgumentOutOfRangeException(nameof(steeringRadians),
                "Steering must be finite and strictly between -pi/2 and pi/2 radians.");
        }

        RequirePositiveFinite(wheelbase, nameof(wheelbase));
        RequirePositiveFinite(timeStep, nameof(timeStep));

        double x = state.Position.X + state.Speed * Math.Cos(state.Heading) * timeStep;
        double y = state.Position.Y + state.Speed * Math.Sin(state.Heading) * timeStep;
        double tangent = Math.Tan(steeringRadians);
        // Straight or stationary motion has exactly zero yaw change, even when
        // the unused speed/wheelbase ratio would overflow for extreme inputs.
        double headingChange = state.Speed == 0 || tangent == 0
            ? 0
            : state.Speed / wheelbase * tangent * timeStep;
        double heading = state.Heading + headingChange;
        var nextState = new VehicleState(
            new Vector2D(x, y),
            double.IsFinite(heading) ? Angles.Normalize(heading) : heading,
            state.Speed);
        if (!nextState.Position.IsFinite || !double.IsFinite(nextState.Heading))
        {
            string details = string.Create(CultureInfo.InvariantCulture,
                $"x={nextState.Position.X:R}, y={nextState.Position.Y:R}, heading={nextState.Heading:R}, speed={nextState.Speed:R}");
            throw new NonFiniteIntegrationException(nextState,
                $"The kinematic bicycle produced a nonfinite state ({details}).");
        }

        return nextState;
    }

    internal static void ValidateState(VehicleState state, string parameterName)
    {
        if (!state.Position.IsFinite || !double.IsFinite(state.Heading)
            || !double.IsFinite(state.Speed) || state.Speed < 0)
        {
            throw new ArgumentException(
                "The position and heading must be finite, and speed must be finite and nonnegative.",
                parameterName);
        }
    }

    private static void RequirePositiveFinite(double value, string parameterName)
    {
        if (!double.IsFinite(value) || value <= 0)
        {
            throw new ArgumentOutOfRangeException(parameterName, "The value must be finite and positive.");
        }
    }

    internal sealed class NonFiniteIntegrationException(VehicleState candidateState, string message)
        : ArithmeticException(message)
    {
        internal VehicleState CandidateState { get; } = candidateState;
    }
}
