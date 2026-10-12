namespace TractorNavigation;

public readonly record struct RouteProjection(
    Vector2D Position,
    double Distance,
    double SignedCrossTrackError,
    double TangentHeading,
    int SegmentIndex);

public readonly record struct RouteSample(Vector2D Position, double Heading, double Distance);

/// <summary>An immutable polyline parameterized by cumulative chord length.</summary>
public sealed class Route
{
    private readonly Vector2D[] _points;
    private readonly Vector2D[] _tangents;
    private readonly double[] _segmentLengths;
    private readonly double[] _cumulativeDistances;

    public Route(IEnumerable<Vector2D> points)
    {
        ArgumentNullException.ThrowIfNull(points);
        var copied = new List<Vector2D>();
        foreach (var point in points)
        {
            if (copied.Count >= NumericGeometry.MaximumRoutePoints)
            {
                throw new ArgumentException($"A route may contain at most {NumericGeometry.MaximumRoutePoints} points.", nameof(points));
            }

            NumericGeometry.RequireFinite(point, nameof(points));
            copied.Add(point);
        }

        if (copied.Count < 2)
        {
            throw new ArgumentException("A route requires at least two points.", nameof(points));
        }

        _points = copied.ToArray();
        _tangents = new Vector2D[_points.Length - 1];
        _segmentLengths = new double[_points.Length - 1];
        _cumulativeDistances = new double[_points.Length];
        for (var index = 0; index < _segmentLengths.Length; index++)
        {
            var delta = _points[index + 1] - _points[index];
            var length = delta.Length;
            if (!delta.IsFinite || !double.IsFinite(length) || length <= 0)
            {
                throw new ArgumentException("Adjacent route points must have a finite, positive separation.", nameof(points));
            }

            var cumulative = _cumulativeDistances[index] + length;
            if (!double.IsFinite(cumulative) || cumulative <= _cumulativeDistances[index])
            {
                throw new ArgumentException("Every segment must add a representable, finite amount to route length.", nameof(points));
            }

            _segmentLengths[index] = length;
            _tangents[index] = delta / length;
            _cumulativeDistances[index + 1] = cumulative;
        }

        Points = Array.AsReadOnly(_points);
        CumulativeDistances = Array.AsReadOnly(_cumulativeDistances);
    }

    public IReadOnlyList<Vector2D> Points { get; }

    public IReadOnlyList<double> CumulativeDistances { get; }

    public double Length => _cumulativeDistances[^1];

    /// <summary>Finds the globally nearest point; exact ties retain the earliest segment.</summary>
    public RouteProjection Project(Vector2D query)
    {
        NumericGeometry.RequireFinite(query, nameof(query));
        var bestSegment = -1;
        var bestFraction = 0.0;
        var bestPosition = default(Vector2D);
        var bestDistance = NumericGeometry.Magnitude.Zero;

        // Orthogonal projection onto each finite segment, clamped to its endpoints.
        for (var index = 0; index < _segmentLengths.Length; index++)
        {
            var displacement = NumericGeometry.Difference(query, _points[index]);
            var along = displacement.DotWithUnit(_tangents[index]);
            var fraction = 0.0;
            if (along.Sign > 0)
            {
                var alongMagnitude = along.Absolute;
                var segmentMagnitude = NumericGeometry.Magnitude.FromDouble(_segmentLengths[index]);
                fraction = alongMagnitude.CompareTo(segmentMagnitude) >= 0
                    ? 1
                    : Math.ScaleB(alongMagnitude.Significand / segmentMagnitude.Significand,
                        alongMagnitude.Exponent - segmentMagnitude.Exponent);
            }

            var candidate = Interpolate(index, fraction);
            var distance = NumericGeometry.Difference(query, candidate).Length;
            if (bestSegment < 0 || distance.CompareTo(bestDistance) < 0)
            {
                bestSegment = index;
                bestFraction = fraction;
                bestPosition = candidate;
                bestDistance = distance;
            }
        }

        var crossTrack = NumericGeometry.Difference(query, bestPosition).CrossWithUnit(_tangents[bestSegment]);
        return new RouteProjection(
            bestPosition,
            DistanceOnSegment(bestSegment, bestFraction),
            crossTrack,
            Math.Atan2(_tangents[bestSegment].Y, _tangents[bestSegment].X),
            bestSegment);
    }

    /// <summary>Samples clamped finite progress, using the preceding chord at an exact interior vertex.</summary>
    public RouteSample AtDistance(double distance)
    {
        if (!double.IsFinite(distance))
        {
            throw new ArgumentOutOfRangeException(nameof(distance), "The distance must be finite.");
        }

        var clamped = Math.Clamp(distance, 0, Length);
        var location = Array.BinarySearch(_cumulativeDistances, clamped);
        var segment = location >= 0 ? Math.Max(0, location - 1) : ~location - 1;
        // Exact stored vertices remain exact even when adding a short segment to
        // a much larger accumulated length rounded the distance increment.
        var fraction = location >= 0
            ? (location == 0 ? 0 : 1)
            : Math.Clamp((clamped - _cumulativeDistances[segment]) / _segmentLengths[segment], 0, 1);
        return new RouteSample(
            Interpolate(segment, fraction),
            Math.Atan2(_tangents[segment].Y, _tangents[segment].X),
            clamped);
    }

    private Vector2D Interpolate(int segment, double fraction)
    {
        if (fraction <= 0)
        {
            return _points[segment];
        }

        if (fraction >= 1)
        {
            return _points[segment + 1];
        }

        var delta = _points[segment + 1] - _points[segment];
        return fraction <= 0.5
            ? _points[segment] + delta * fraction
            : _points[segment + 1] - delta * (1 - fraction);
    }

    private double DistanceOnSegment(int segment, double fraction) => fraction switch
    {
        <= 0 => _cumulativeDistances[segment],
        >= 1 => _cumulativeDistances[segment + 1],
        _ => _cumulativeDistances[segment] + _segmentLengths[segment] * fraction
    };
}
