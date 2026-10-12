namespace TractorNavigation;

public static class RouteFactory
{
    /// <summary>Creates a straight route with exact endpoints and chords no longer than spacing.</summary>
    public static Route Line(double length, double spacing = 0.05)
    {
        NumericGeometry.RequirePositive(length, nameof(length));
        NumericGeometry.RequirePositive(spacing, nameof(spacing));
        var segments = SegmentCount(length, spacing);
        var points = new Vector2D[segments + 1];
        for (var index = 1; index < segments; index++)
        {
            points[index] = new Vector2D(length * ((double)index / segments), 0);
        }

        points[^1] = new Vector2D(length, 0);
        return new Route(points);
    }

    /// <summary>Creates a signed circular arc starting at the origin, tangent to +x.</summary>
    public static Route Arc(double radius, double sweepRadians, double spacing = 0.05)
    {
        NumericGeometry.RequirePositive(radius, nameof(radius));
        NumericGeometry.RequirePositive(spacing, nameof(spacing));
        if (!double.IsFinite(sweepRadians) || sweepRadians == 0 || Math.Abs(sweepRadians) > Math.PI)
        {
            throw new ArgumentOutOfRangeException(nameof(sweepRadians), "The signed sweep must be finite, nonzero, and no larger than pi in magnitude.");
        }

        var sweep = Math.Abs(sweepRadians);
        var arcLength = radius * sweep;
        if (!double.IsFinite(arcLength) || arcLength <= 0)
        {
            throw new ArgumentOutOfRangeException(nameof(radius), "The arc length must be representable as a finite positive number.");
        }

        var segments = SegmentCount(arcLength, spacing);
        var points = new Vector2D[segments + 1];
        var sign = Math.Sign(sweepRadians);
        for (var index = 1; index <= segments; index++)
        {
            var angle = index == segments ? sweep : sweep * ((double)index / segments);
            var halfSine = Math.Sin(angle * 0.5);
            // 1-cos(angle) = 2*sin(angle/2)^2 avoids cancellation for small sweeps.
            points[index] = new Vector2D(radius * Math.Sin(angle), sign * (radius * halfSine) * (2 * halfSine));
        }

        return new Route(points);
    }

    private static int SegmentCount(double length, double spacing)
    {
        var count = Math.Max(1, Math.Ceiling(length / spacing));
        if (!double.IsFinite(count) || count >= NumericGeometry.MaximumRoutePoints)
        {
            throw new ArgumentOutOfRangeException(nameof(spacing), $"Sampling would exceed {NumericGeometry.MaximumRoutePoints} route points.");
        }

        return (int)count;
    }
}
