using TractorNavigation;

namespace TractorNavigation.Tests;

internal static class GeometryTests
{
    internal static void Register(List<(string Name, Action Run)> tests)
    {
        tests.Add(("vectors: arithmetic, products, norms and finiteness", Vectors));
        tests.Add(("angles: boundaries, turns and invalid values", AnglesWrap));
        tests.Add(("route: analytic straight projection and left-positive signs", StraightProjection));
        tests.Add(("route: small perpendicular error survives huge along-track coordinates", UnequalScaleProjection));
        tests.Add(("route: global projection, corner tangent and earliest ties", CornerProjection));
        tests.Add(("route: clamped sampling, cumulative length and copied immutable points", RouteSampling));
        tests.Add(("factories: exact line endpoints and spacing", LineFactory));
        tests.Add(("factories: signed arc endpoints and mirrored sampled chords", ArcFactory));
        tests.Add(("geometry: invalid coordinates, distances and factory parameters", Invalid));
    }

    private static void Vectors()
    {
        var a = new Vector2D(3, 4);
        var b = new Vector2D(-2, 5);
        Check.Vector(new(1, 9), a + b);
        Check.Vector(new(5, -1), a - b);
        Check.Vector(new(6, 8), a * 2);
        Check.Vector(new(6, 8), 2 * a);
        Check.Vector(new(1.5, 2), a / 2);
        Check.Near(25, a.LengthSquared);
        Check.Near(5, a.Length);
        Check.Near(14, Vector2D.Dot(a, b));
        Check.Near(23, Vector2D.Cross(a, b));
        Check.True(a.IsFinite, "Ordinary vector must be finite.");
        Check.True(!new Vector2D(double.NaN, 0).IsFinite, "NaN vector must be nonfinite.");
        Check.True(!new Vector2D(0, double.PositiveInfinity).IsFinite, "Infinite vector must be nonfinite.");
    }

    private static void AnglesWrap()
    {
        Check.Near(0, Angles.Normalize(0));
        Check.Near(-Math.PI, Angles.Normalize(Math.PI));
        Check.Near(-Math.PI, Angles.Normalize(-Math.PI));
        Check.Near(0.3, Angles.Normalize(8 * Math.PI + 0.3), 1e-14);
        Check.Near(-0.3, Angles.Normalize(-8 * Math.PI - 0.3), 1e-14);
        Check.Near(-Math.PI / 2, Angles.Normalize(3 * Math.PI / 2));
        foreach (var x in new[] { double.MaxValue, -double.MaxValue, 1e20, -1e20 })
        {
            var normalized = Angles.Normalize(x);
            Check.True(normalized >= -Math.PI && normalized < Math.PI, "Finite angle was not wrapped into [-pi,pi).");
        }
        Check.Reject(() => Angles.Normalize(double.NaN));
        Check.Reject(() => Angles.Normalize(double.PositiveInfinity));
    }

    private static void StraightProjection()
    {
        var route = new Route(new[] { new Vector2D(1, 2), new Vector2D(7, 2) });
        var above = route.Project(new(4, 5));
        Check.Vector(new(4, 2), above.Position);
        Check.Near(3, above.Distance);
        Check.Near(3, above.SignedCrossTrackError);
        Check.Near(0, above.TangentHeading);
        Check.Equal(0, above.SegmentIndex);
        Check.Near(-4, route.Project(new(4, -2)).SignedCrossTrackError);
        var before = route.Project(new(-2, 5));
        Check.Vector(new(1, 2), before.Position);
        Check.Near(0, before.Distance);
        Check.Near(3, before.SignedCrossTrackError);
        Check.Vector(new(7, 2), route.Project(new(10, 2)).Position);
        var diagonal = new Route(new[] { new Vector2D(0, 0), new Vector2D(3, 4) });
        var projection = diagonal.Project(new(0.2, 2.6));
        Check.Vector(new(1.32, 1.76), projection.Position);
        Check.Near(2.2, projection.Distance);
        Check.Near(1.4, projection.SignedCrossTrackError);
        Check.Near(Math.Atan2(4, 3), projection.TangentHeading);
    }

    private static void CornerProjection()
    {
        var route = new Route(new[] { new Vector2D(0, 0), new Vector2D(4, 0), new Vector2D(4, 3) });
        var nearest = route.Project(new(5, 2));
        Check.Equal(1, nearest.SegmentIndex);
        Check.Vector(new(4, 2), nearest.Position);
        Check.Near(6, nearest.Distance);
        Check.Near(-1, nearest.SignedCrossTrackError);
        Check.Near(Math.PI / 2, nearest.TangentHeading);
        var tie = route.Project(new(3, 1));
        Check.Equal(0, tie.SegmentIndex);
        Check.Vector(new(3, 0), tie.Position);
        Check.Near(1, tie.SignedCrossTrackError);
        var corner = route.Project(new(4, 0));
        Check.Equal(0, corner.SegmentIndex);
        Check.Near(0, corner.TangentHeading);
        var folded = new Route(new[] { new Vector2D(0, 0), new Vector2D(10, 0), new Vector2D(10, 10), new Vector2D(0, 10) });
        Check.Equal(2, folded.Project(new(2, 9)).SegmentIndex);
    }

    private static void UnequalScaleProjection()
    {
        var route = RouteFactory.Line(12);
        Check.Equal(1e-100, route.Project(new(1e308, 1e-100)).SignedCrossTrackError);
        Check.Equal(-1e-100, route.Project(new(1e308, -1e-100)).SignedCrossTrackError);
        Check.Near(1e-100, route.Project(new(1e-100, 1e308)).Position.X, 1e-114);
        var hugeAlongTrack = new Route(new[] { new Vector2D(-double.MaxValue, 0), new Vector2D(-double.MaxValue / 2, 0) });
        Check.Equal(double.Epsilon, hugeAlongTrack.Project(new(double.MaxValue, double.Epsilon)).SignedCrossTrackError);
        var vertical = new Route(new[] { new Vector2D(-double.MaxValue, 0), new Vector2D(-double.MaxValue, 1) });
        try
        {
            vertical.Project(new(double.MaxValue, 0));
        }
        catch (OverflowException)
        {
            return;
        }

        throw new InvalidOperationException("Unrepresentable cross-track error was not rejected explicitly.");
    }

    private static void RouteSampling()
    {
        var points = new[] { new Vector2D(0, 0), new Vector2D(3, 0), new Vector2D(3, 4) };
        var route = new Route(points);
        points[0] = new(100, 100);
        Check.Vector(new(0, 0), route.Points[0]);
        Check.Near(7, route.Length);
        Check.Equal(3, route.CumulativeDistances.Count);
        Check.Near(0, route.CumulativeDistances[0]);
        Check.Near(3, route.CumulativeDistances[1]);
        Check.Near(7, route.CumulativeDistances[2]);
        Check.Vector(new(0, 0), route.AtDistance(-10).Position);
        Check.Near(0, route.AtDistance(-10).Distance);
        Check.Vector(new(3, 4), route.AtDistance(20).Position);
        Check.Near(7, route.AtDistance(20).Distance);
        Check.Near(Math.PI / 2, route.AtDistance(20).Heading);
        Check.Vector(new(1.5, 0), route.AtDistance(1.5).Position);
        Check.Vector(new(3, 2), route.AtDistance(5).Position);
        Check.Near(Math.PI / 2, route.AtDistance(5).Heading);
        Check.Near(0, route.AtDistance(3).Heading);
        Check.ReadOnly(route.Points);
        Check.ReadOnly(route.CumulativeDistances);
    }

    private static void LineFactory()
    {
        var line = RouteFactory.Line(1, 0.3);
        Check.Equal(5, line.Points.Count);
        Check.Vector(new(0, 0), line.Points[0]);
        Check.Vector(new(1, 0), line.Points[^1]);
        Check.Near(1, line.Length);
        for (var i = 1; i < line.Points.Count; i++)
        {
            Check.Near(0, line.Points[i].Y);
            Check.True(line.Points[i].X > line.Points[i - 1].X, "Line samples must advance.");
            Check.True((line.Points[i] - line.Points[i - 1]).Length <= 0.3 + 1e-12, "Line spacing exceeded.");
        }
        var tiny = RouteFactory.Line(0.01, 1);
        Check.Equal(2, tiny.Points.Count);
        Check.Near(0.01, tiny.Length);
    }

    private static void ArcFactory()
    {
        var left = RouteFactory.Arc(2, Math.PI / 2, 0.2);
        var right = RouteFactory.Arc(2, -Math.PI / 2, 0.2);
        Check.Vector(new(0, 0), left.Points[0]);
        Check.Vector(new(2, 2), left.Points[^1]);
        Check.Vector(new(2, -2), right.Points[^1]);
        Check.Equal(left.Points.Count, right.Points.Count);
        Check.True(left.Length <= Math.PI && left.Length > Math.PI - 0.01, "Chord length must approach analytic arc length from below.");
        Check.Near(left.Length, right.Length);
        for (var i = 0; i < left.Points.Count; i++)
        {
            Check.Vector(new(left.Points[i].X, -left.Points[i].Y), right.Points[i]);
            Check.Near(4, left.Points[i].X * left.Points[i].X + Math.Pow(left.Points[i].Y - 2, 2), 2e-12);
            if (i > 0) Check.True((left.Points[i] - left.Points[i - 1]).Length <= 0.2 + 1e-12, "Arc spacing exceeded.");
        }
        var coarse = RouteFactory.Arc(2, Math.PI / 2, 100);
        Check.Equal(2, coarse.Points.Count);
        Check.Near(Math.PI / 4, coarse.AtDistance(coarse.Length / 2).Heading);
        Check.Vector(new(1, 1), coarse.AtDistance(coarse.Length / 2).Position);
        Check.Vector(new(0, 4), RouteFactory.Arc(2, Math.PI).Points[^1], 1e-12);
    }

    private static void Invalid()
    {
        Check.Reject(() => new Route(Array.Empty<Vector2D>()));
        Check.Reject(() => new Route(new[] { new Vector2D(0, 0) }));
        Check.Reject(() => new Route(new[] { new Vector2D(0, 0), new Vector2D(0, 0) }));
        Check.Reject(() => new Route(new[] { new Vector2D(0, 0), new Vector2D(double.NaN, 1) }));
        Check.Reject(() => new Route(null!));
        var route = RouteFactory.Line(1);
        Check.Reject(() => route.Project(new(double.NaN, 0)));
        Check.Reject(() => route.AtDistance(double.NaN));
        Check.Reject(() => route.AtDistance(double.PositiveInfinity));
        foreach (var invalid in new[] { 0d, -1, double.NaN, double.PositiveInfinity })
        {
            Check.Reject(() => RouteFactory.Line(invalid));
            Check.Reject(() => RouteFactory.Line(1, invalid));
            Check.Reject(() => RouteFactory.Arc(invalid, 1));
            Check.Reject(() => RouteFactory.Arc(1, 1, invalid));
        }
        foreach (var invalid in new[] { 0d, Math.PI + 0.01, -Math.PI - 0.01, double.NaN, double.PositiveInfinity })
            Check.Reject(() => RouteFactory.Arc(1, invalid));
        Check.Reject(() => RouteFactory.Line(1e20, 1e-20));
    }
}
