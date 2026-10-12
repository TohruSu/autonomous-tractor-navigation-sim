using System.Globalization;
using System.Text;
using TractorNavigation;

namespace TractorNavigation.Cli;

internal sealed record ExportedRun(Scenario Scenario, SimulationResult Result);

internal static class CsvExport
{
    public static void Write(string directory, IReadOnlyList<ExportedRun> runs)
    {
        using var summary = Writer(Path.Combine(directory, "summary.csv"));
        using var samples = Writer(Path.Combine(directory, "samples.csv"));
        Row(summary, "scenario", "controller", "status", "failure_reason", "sample_count",
            "rms_cross_track_error_m", "max_abs_cross_track_error_m", "elapsed_s",
            "path_progress_m", "final_endpoint_distance_m", "angle_saturation_steps",
            "rate_limited_steps", "speed_mps", "wheelbase_m", "step_s", "max_duration_s",
            "max_steering_rad", "max_steering_rate_rad_s", "endpoint_tolerance_m", "route_length_m");
        Row(samples, "scenario", "controller", "step", "time_s", "x_m", "y_m", "heading_rad",
            "speed_mps", "requested_steering_rad", "applied_steering_rad",
            "cross_track_error_m", "progress_m");
        foreach (ExportedRun run in runs)
        {
            SimulationResult result = run.Result;
            EvaluationMetrics metrics = result.Metrics;
            SimulationSettings settings = run.Scenario.Settings;
            Row(summary, result.ScenarioName, result.ControllerName, result.Status.ToString(),
                result.FailureReason ?? "", Integer(result.Samples.Count),
                Number(metrics.RmsCrossTrackErrorMeters), Number(metrics.MaxAbsoluteCrossTrackErrorMeters),
                Number(metrics.ElapsedSeconds), Number(metrics.PathProgressMeters),
                Number(metrics.FinalEndpointDistanceMeters), Integer(metrics.SteeringSaturationCount),
                Integer(metrics.RateLimitCount), Number(run.Scenario.InitialState.Speed),
                Number(settings.WheelbaseMeters), Number(settings.TimeStepSeconds),
                Number(settings.MaxDurationSeconds), Number(settings.MaxSteeringRadians),
                Number(settings.MaxSteeringRateRadiansPerSecond), Number(settings.EndpointToleranceMeters),
                Number(run.Scenario.Route.Length));
            foreach (SimulationSample sample in result.Samples)
                Row(samples, result.ScenarioName, result.ControllerName, Integer(sample.Step),
                    Number(sample.TimeSeconds), Number(sample.State.Position.X), Number(sample.State.Position.Y),
                    Number(sample.State.Heading), Number(sample.State.Speed), Number(sample.RequestedSteeringRadians),
                    Number(sample.AppliedSteeringRadians), Number(sample.CrossTrackErrorMeters), Number(sample.ProgressMeters));
        }
    }

    private static StreamWriter Writer(string path) =>
        new(path, false, new UTF8Encoding(false)) { NewLine = "\n" };

    // Round-trip formatting preserves nonfinite values visibly as NaN/Infinity.
    private static string Number(double value) => value.ToString("R", CultureInfo.InvariantCulture);
    private static string Integer(int value) => value.ToString(CultureInfo.InvariantCulture);
    private static void Row(TextWriter writer, params string[] fields) =>
        writer.WriteLine(string.Join(",", fields.Select(Escape)));
    private static string Escape(string value) =>
        value.IndexOfAny([',', '"', '\r', '\n']) >= 0
            ? "\"" + value.Replace("\"", "\"\"", StringComparison.Ordinal) + "\""
            : value;
}
