using System.Globalization;
using TractorNavigation;

namespace TractorNavigation.Tests;

internal static class ControllerTests
{
    internal static void Register(List<(string Name, Action Run)> tests)
    {
        tests.Add(("Stanley: analytic correction signs and zero error", StanleyStraight));
        tests.Add(("Stanley: front axle, heading reference and unclipped request", StanleyFront));
        tests.Add(("Pure Pursuit: analytic target, rear projection and vehicle frame", PursuitReference));
        tests.Add(("Pure Pursuit: correction signs, zero error and coincident target", PursuitSigns));
        tests.Add(("controllers: stable invariant parameter names", Names));
        tests.Add(("controllers: invalid parameters, state and wheelbase", Invalid));
    }

    private static void StanleyStraight()
    {
        var route = RouteFactory.Line(20);
        var controller = new StanleyController();
        Check.Near(-Math.Atan(1d / 3), controller.ComputeSteering(new(new(2, 0.5), 0, 1), route, 1));
        Check.Near(Math.Atan(1d / 3), controller.ComputeSteering(new(new(2, -0.5), 0, 1), route, 1));
        Check.Near(0, controller.ComputeSteering(new(new(2, 0), 0, 1), route, 1));
        Check.Near(-Math.Atan(1), controller.ComputeSteering(new(new(2, 0.5), 0, 0), route, 1));
        var custom = new StanleyController(2, 0.25);
        Check.Near(-Math.Atan(1d / 1.25), custom.ComputeSteering(new(new(2, 0.5), 0, 1), route, 1));
    }

    private static void StanleyFront()
    {
        var route = new Route(new[] { new Vector2D(0, 0), new Vector2D(2, 0), new Vector2D(2, 2) });
        var request = new StanleyController().ComputeSteering(new(new(1.5, 0.5), 0, 1), route, 1);
        Check.Near(Math.PI / 2 + Math.Atan(1d / 3), request);
        Check.True(request > 0.6, "Controller must expose the request before actuator limits.");
        var heading = 0.2;
        var frontOnPath = new VehicleState(new(3 - Math.Cos(heading), -Math.Sin(heading)), heading, 1);
        Check.Near(-heading, new StanleyController().ComputeSteering(frontOnPath, RouteFactory.Line(20), 1));
        var west = new Route(new[] { new Vector2D(5, 0), new Vector2D(-5, 0) });
        var yaw = -Math.PI + 0.1;
        var westFrontOnPath = new VehicleState(new(-Math.Cos(yaw), -Math.Sin(yaw)), yaw, 1);
        Check.Near(-0.1, new StanleyController().ComputeSteering(westFrontOnPath, west, 1), 1e-12);
    }

    private static void PursuitReference()
    {
        var route = RouteFactory.Line(20);
        var controller = new PurePursuitController();
        Check.Near(Math.Atan2(-1, 1.94), controller.ComputeSteering(new(new(0, 0.5), 0, 1), route, 1));
        Check.Near(Math.Atan2(-2, 1), controller.ComputeSteering(new(new(0, 0), Math.PI / 2, 0), route, 1));
        var yaw = 0.3;
        var localY = -3 * Math.Sin(yaw) - 0.5 * Math.Cos(yaw);
        Check.Near(Math.Atan2(3 * localY, 9.25), new PurePursuitController(2, 0.5)
            .ComputeSteering(new(new(2, 0.5), yaw, 2), route, 1.5));
        Check.Near(Math.Atan2(-1, 1.25), new PurePursuitController(1, 0)
            .ComputeSteering(new(new(0, 0.5), 0, 20), route, 1));
    }

    private static void PursuitSigns()
    {
        var controller = new PurePursuitController();
        var route = RouteFactory.Line(20);
        var left = controller.ComputeSteering(new(new(2, 0.5), 0, 1), route, 1);
        var right = controller.ComputeSteering(new(new(2, -0.5), 0, 1), route, 1);
        Check.True(left < 0 && right > 0, "Offset steering must point toward the route.");
        Check.Near(left, -right);
        Check.Near(0, controller.ComputeSteering(new(new(2, 0), 0, 1), route, 1));
        Check.Near(0, controller.ComputeSteering(new(new(20, 0), 0.7, 1), route, 1));
        Check.True(Math.Abs(controller.ComputeSteering(new(new(0, 0), Math.PI / 2, 0), route, 1)) > 0.6,
            "Controller must expose the request before actuator limits.");
    }

    private static void Names()
    {
        var original = CultureInfo.CurrentCulture;
        try
        {
            CultureInfo.CurrentCulture = CultureInfo.GetCultureInfo("fr-FR");
            Check.Equal("Stanley(gain=0.5,speedSoftening=0.25)", new StanleyController(0.5, 0.25).Name);
            Check.Equal("PurePursuit(lookahead=2,speedGain=0.3)", new PurePursuitController(2, 0.3).Name);
            Check.Equal(new StanleyController().Name, new StanleyController().Name);
            Check.Equal(new PurePursuitController().Name, new PurePursuitController().Name);
        }
        finally { CultureInfo.CurrentCulture = original; }
    }

    private static void Invalid()
    {
        foreach (var bad in new[] { 0d, -1, double.NaN, double.PositiveInfinity })
        {
            Check.Reject(() => new StanleyController(bad));
            Check.Reject(() => new StanleyController(1, bad));
            Check.Reject(() => new PurePursuitController(bad));
        }
        foreach (var bad in new[] { -1d, double.NaN, double.PositiveInfinity })
            Check.Reject(() => new PurePursuitController(1, bad));
        var route = RouteFactory.Line(10);
        foreach (var controller in new IPathController[] { new StanleyController(), new PurePursuitController() })
        {
            foreach (var bad in new[] { 0d, -1, double.NaN, double.PositiveInfinity })
                Check.Reject(() => controller.ComputeSteering(new(new(0, 0), 0, 1), route, bad));
            foreach (var state in new[]
            {
                new VehicleState(new(0, 0), 0, -1), new VehicleState(new(0, 0), 0, double.NaN),
                new VehicleState(new(0, 0), 0, double.PositiveInfinity), new VehicleState(new(double.NaN, 0), 0, 1),
                new VehicleState(new(0, 0), double.PositiveInfinity, 1)
            }) Check.Reject(() => controller.ComputeSteering(state, route, 1));
            Check.Reject(() => controller.ComputeSteering(new(new(0, 0), 0, 1), null!, 1));
        }
    }
}
