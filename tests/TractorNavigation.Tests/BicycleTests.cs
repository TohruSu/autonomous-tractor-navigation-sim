using TractorNavigation;

namespace TractorNavigation.Tests;

internal static class BicycleTests
{
    internal static void Register(List<(string Name, Action Run)> tests)
    {
        tests.Add(("bicycle: analytic one-step explicit Euler", OneStep));
        tests.Add(("bicycle: straight, stationary, speed and heading invariants", Invariants));
        tests.Add(("bicycle: reflected motion under reflected steering", Mirror));
        tests.Add(("bicycle: zero steering avoids unused overflowing yaw ratio", ExtremeStraight));
        tests.Add(("bicycle: invalid and overflowing inputs rejected", Invalid));
    }

    private static void OneStep()
    {
        var result = KinematicBicycle.Step(new(new(2, 3), 0, 2), Math.Atan(0.5), 2, 0.2);
        Check.Vector(new(2.4, 3), result.Position);
        Check.Near(0.1, result.Heading);
        Check.Near(2, result.Speed);
        var angled = KinematicBicycle.Step(new(new(2, 3), Math.PI / 2, 2), Math.Atan(0.5), 2, 0.2);
        Check.Vector(new(2, 3.4), angled.Position);
        Check.Near(Math.PI / 2 + 0.1, angled.Heading);
    }

    private static void Invariants()
    {
        var state = new VehicleState(new(-3, 4), 0.7, 1.5);
        for (var i = 0; i < 100; i++) state = KinematicBicycle.Step(state, 0, 1, 0.02);
        Check.Vector(new(-3 + 3 * Math.Cos(0.7), 4 + 3 * Math.Sin(0.7)), state.Position, 2e-13);
        Check.Near(0.7, state.Heading);
        Check.Near(1.5, state.Speed);
        var stationary = new VehicleState(new(4, 5), 0.4, 0);
        var still = KinematicBicycle.Step(stationary, 0.6, 1, 0.02);
        Check.Vector(stationary.Position, still.Position);
        Check.Near(stationary.Heading, still.Heading);
        Check.Near(stationary.Speed, still.Speed);
        var wrapped = KinematicBicycle.Step(new(new(0, 0), Math.PI - 0.01, 1), Math.Atan(1), 1, 0.02);
        Check.Near(-Math.PI + 0.01, wrapped.Heading);
    }

    private static void Mirror()
    {
        var left = new VehicleState(new(1, 2), 0.3, 1.7);
        var right = new VehicleState(new(1, -2), -0.3, 1.7);
        for (var i = 0; i < 50; i++)
        {
            left = KinematicBicycle.Step(left, 0.25, 1.4, 0.02);
            right = KinematicBicycle.Step(right, -0.25, 1.4, 0.02);
            Check.Vector(new(left.Position.X, -left.Position.Y), right.Position, 1e-12);
            Check.Near(-left.Heading, right.Heading, 1e-12);
            Check.Near(left.Speed, right.Speed);
        }
    }

    private static void ExtremeStraight()
    {
        var state = KinematicBicycle.Step(new(new(0, 0), 0, 1e308), 0, 1e-308, 1e-308);
        Check.Vector(new(1, 0), state.Position, 1e-14);
        Check.Near(0, state.Heading);
        Check.Near(1e308, state.Speed, 0);
    }

    private static void Invalid()
    {
        var valid = new VehicleState(new(0, 0), 0, 1);
        foreach (var bad in new[] { 0d, -1, double.NaN, double.PositiveInfinity })
        {
            Check.Reject(() => KinematicBicycle.Step(valid, 0, bad, 0.02));
            Check.Reject(() => KinematicBicycle.Step(valid, 0, 1, bad));
        }
        foreach (var bad in new[] { double.NaN, double.PositiveInfinity, Math.PI / 2, -Math.PI / 2 })
            Check.Reject(() => KinematicBicycle.Step(valid, bad, 1, 0.02));
        foreach (var state in new[]
        {
            new VehicleState(new(double.NaN, 0), 0, 1), new VehicleState(new(0, 0), double.NaN, 1),
            new VehicleState(new(0, 0), 0, -1), new VehicleState(new(0, 0), 0, double.PositiveInfinity)
        }) Check.Reject(() => KinematicBicycle.Step(state, 0, 1, 0.02));
        var overflowed = false;
        try { KinematicBicycle.Step(new(new(double.MaxValue, 0), 0, double.MaxValue), 0, 1, 2); }
        catch (Exception error) when (error is ArithmeticException or ArgumentException or InvalidOperationException)
        { overflowed = true; }
        Check.True(overflowed, "Overflowing integration must not return a nonfinite state.");
    }
}
