using System.Globalization;
using TractorNavigation;

namespace TractorNavigation.Cli;

internal static class Program
{
    private const string Help = """
        Standalone navigation algorithm simulation (synthetic data only)

        Commands:
          list
          simulate --output DIRECTORY [--scenario NAME|all]
                   [--controller all|stanley|pure-pursuit]
                   [--speed MPS] [--duration SECONDS] [--step SECONDS]
          help

        Defaults: all seven routes, four comparison configurations, speed 1 m/s,
                  duration 25 s, step 0.02 s. Output directory must be empty/new.
        Files: summary.csv (every run), samples.csv (every recorded state).
        Exit codes: 0 success, 2 any incomplete/failed run, 64 invalid arguments,
                    1 output/system error. Failed runs remain in the CSV files.
        """;

    private static int Main(string[] args)
    {
        try
        {
            if (args.Length == 0 || args is ["help"] or ["--help"] or ["-h"])
            {
                Console.WriteLine(Help);
                return 0;
            }

            if (args is ["list"])
            {
                foreach (Scenario scenario in ScenarioCatalog.Create())
                    Console.WriteLine(scenario.Name);
                foreach (IPathController controller in ScenarioCatalog.Controllers())
                    Console.WriteLine(controller.Name);
                return 0;
            }

            if (args[0] != "simulate")
                throw new ArgumentException($"Unknown command: {args[0]}");

            Options options = Parse(args[1..]);
            Scenario[] scenarios = ScenarioCatalog.Create()
                .Where(s => options.Scenario == "all" || s.Name == options.Scenario)
                .Select(s => s with
                {
                    InitialState = s.InitialState with { Speed = options.Speed },
                    Settings = s.Settings with
                    {
                        TimeStepSeconds = options.Step,
                        MaxDurationSeconds = options.Duration
                    }
                }).ToArray();
            if (scenarios.Length == 0)
                throw new ArgumentException($"Unknown scenario: {options.Scenario}");

            IPathController[] controllers = options.Controller switch
            {
                "all" => ScenarioCatalog.Controllers().ToArray(),
                "stanley" => [new StanleyController()],
                "pure-pursuit" => [new PurePursuitController()],
                _ => throw new ArgumentException($"Unknown controller: {options.Controller}")
            };

            string outputDirectory = Path.GetFullPath(options.Output);
            if (Directory.Exists(outputDirectory) &&
                Directory.EnumerateFileSystemEntries(outputDirectory).Any())
                throw new ArgumentException("Output directory must be empty or new.");
            double integrationSteps = Math.Floor(options.Duration / options.Step);
            if (!double.IsFinite(integrationSteps) ||
                integrationSteps > SimulationSettings.MaximumSupportedSteps)
                throw new ArgumentException("The requested step count exceeds the supported per-run limit.");
            const long maximumComparisonSamples = 2_000_000;
            long totalSamples = ((long)integrationSteps + 1) * scenarios.Length * controllers.Length;
            if (totalSamples > maximumComparisonSamples)
                throw new ArgumentException("The comparison would exceed two million recorded samples; select fewer runs or a larger step.");

            // Validate the full selection before creating any output files.
            var runs = new List<ExportedRun>();
            foreach (Scenario scenario in scenarios)
                foreach (IPathController controller in controllers)
                    runs.Add(new ExportedRun(scenario, SimulationRunner.Run(scenario, controller)));

            Directory.CreateDirectory(outputDirectory);
            CsvExport.Write(outputDirectory, runs);

            foreach (ExportedRun run in runs)
                Console.WriteLine(FormattableString.Invariant(
                    $"{run.Result.ScenarioName} | {run.Result.ControllerName} | {run.Result.Status} | RMS {run.Result.Metrics.RmsCrossTrackErrorMeters:F4} m"));
            int completed = runs.Count(r => r.Result.Status == SimulationStatus.Completed);
            Console.WriteLine($"{completed}/{runs.Count} routes completed. CSV files: {outputDirectory}");
            return completed == runs.Count ? 0 : 2;
        }
        catch (ArgumentException exception)
        {
            Console.Error.WriteLine(exception.Message);
            Console.Error.WriteLine("Run 'help' for usage.");
            return 64;
        }
        catch (Exception exception) when (exception is IOException or UnauthorizedAccessException)
        {
            Console.Error.WriteLine($"Output error: {exception.Message}");
            return 1;
        }
    }

    private static Options Parse(string[] args)
    {
        var values = new Dictionary<string, string>(StringComparer.Ordinal);
        string[] allowed = ["--output", "--scenario", "--controller", "--speed", "--duration", "--step"];
        for (int i = 0; i < args.Length; i += 2)
        {
            string key = args[i];
            if (!allowed.Contains(key) || i + 1 >= args.Length || args[i + 1].StartsWith("--", StringComparison.Ordinal))
                throw new ArgumentException($"Unknown option or missing value: {key}");
            if (!values.TryAdd(key, args[i + 1]))
                throw new ArgumentException($"Duplicate option: {key}");
        }
        if (!values.TryGetValue("--output", out string? output) || string.IsNullOrWhiteSpace(output))
            throw new ArgumentException("--output DIRECTORY is required.");

        double Number(string key, double defaultValue, bool allowZero = false)
        {
            if (!values.TryGetValue(key, out string? raw))
                return defaultValue;
            if (!double.TryParse(raw, NumberStyles.Float, CultureInfo.InvariantCulture, out double value) ||
                !double.IsFinite(value) || (allowZero ? value < 0 : value <= 0))
                throw new ArgumentException($"{key} must be a finite {(allowZero ? "nonnegative" : "positive")} number.");
            return value;
        }

        return new Options(output, values.GetValueOrDefault("--scenario", "all"),
            values.GetValueOrDefault("--controller", "all"), Number("--speed", 1, true),
            Number("--duration", 25), Number("--step", 0.02));
    }

    private sealed record Options(string Output, string Scenario, string Controller,
        double Speed, double Duration, double Step);
}
