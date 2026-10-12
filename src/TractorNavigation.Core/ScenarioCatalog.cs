namespace TractorNavigation;

/// <summary>Synthetic routes in metres, with no historical coordinates or device inputs.</summary>
public static class ScenarioCatalog
{
    public static IReadOnlyList<Scenario> Create()
    {
        var settings = new SimulationSettings();
        return Array.AsReadOnly(new[]
        {
            Make("straight", RouteFactory.Line(12), 0, settings),
            Make("straight-offset-left", RouteFactory.Line(12), 0.5, settings),
            Make("straight-offset-right", RouteFactory.Line(12), -0.5, settings),
            Make("left-turn", RouteFactory.Arc(6, Math.PI / 2), 0, settings),
            Make("right-turn", RouteFactory.Arc(6, -Math.PI / 2), 0, settings),
            Make("left-turn-offset", RouteFactory.Arc(6, Math.PI / 2), 0.5, settings),
            Make("right-turn-offset", RouteFactory.Arc(6, -Math.PI / 2), -0.5, settings)
        });
    }

    public static IReadOnlyList<IPathController> Controllers() =>
        Array.AsReadOnly<IPathController>(new IPathController[]
        {
            new StanleyController(),
            new PurePursuitController(),
            new StanleyController(gain: 0.5),
            new PurePursuitController(lookahead: 2.0)
        });

    private static Scenario Make(string name, Route route, double offset, SimulationSettings settings) =>
        new(name, route, new VehicleState(new Vector2D(0, offset), 0, 1), settings);
}
