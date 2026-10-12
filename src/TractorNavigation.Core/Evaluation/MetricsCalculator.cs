namespace TractorNavigation;

internal static class MetricsCalculator
{
    internal static EvaluationMetrics Calculate(
        IReadOnlyList<SimulationSample> samples,
        Vector2D endpoint,
        int steeringSaturationCount,
        int rateLimitCount)
    {
        if (samples.Count == 0)
        {
            throw new ArgumentException("At least the initial state must be recorded.", nameof(samples));
        }

        (double rms, double maximum) = CrossTrackErrors(samples);
        SimulationSample finalSample = samples[^1];
        double endpointDistance = finalSample.State.Position.IsFinite
            && double.IsFinite(finalSample.State.Heading)
            && double.IsFinite(finalSample.State.Speed)
            ? Distance(finalSample.State.Position, endpoint)
            : double.NaN;
        return new EvaluationMetrics(
            rms,
            maximum,
            FiniteOrUndefined(finalSample.TimeSeconds),
            FiniteOrUndefined(finalSample.ProgressMeters),
            FiniteOrUndefined(endpointDistance),
            steeringSaturationCount,
            rateLimitCount);
    }

    internal static double Distance(Vector2D first, Vector2D second)
    {
        double x = Math.Abs(first.X - second.X);
        double y = Math.Abs(first.Y - second.Y);
        if (!double.IsFinite(x) || !double.IsFinite(y))
        {
            return double.NaN;
        }

        double scale = Math.Max(x, y);
        if (scale == 0)
        {
            return 0;
        }

        double ratio = Math.Min(x, y) / scale;
        return scale * Math.Sqrt(1 + ratio * ratio);
    }

    private static (double Rms, double Maximum) CrossTrackErrors(IReadOnlyList<SimulationSample> samples)
    {
        double scale = 0;
        double scaledSquares = 0;
        foreach (SimulationSample sample in samples)
        {
            double error = Math.Abs(sample.CrossTrackErrorMeters);
            if (!double.IsFinite(error))
            {
                // A failed observation is retained; the aggregate is undefined rather than filtered.
                return (double.NaN, double.NaN);
            }

            if (error == 0)
            {
                continue;
            }

            if (error > scale)
            {
                double ratio = scale / error;
                scaledSquares = 1 + scaledSquares * ratio * ratio;
                scale = error;
            }
            else
            {
                double ratio = error / scale;
                scaledSquares += ratio * ratio;
            }
        }

        // The mean normalized square cannot exceed one. The clamp prevents final
        // roundoff from overflowing when the largest finite error approaches MaxValue.
        double rms = scale == 0 ? 0 : scale * Math.Sqrt(Math.Min(1, scaledSquares / samples.Count));
        return (rms, scale);
    }

    private static double FiniteOrUndefined(double value) => double.IsFinite(value) ? value : double.NaN;
}
