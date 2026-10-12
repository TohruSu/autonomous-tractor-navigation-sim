using TractorNavigation;

namespace TractorNavigation.Tests;

internal static class SimulationTests
{
    internal static void Register(List<(string Name, Action Run)> tests)
    {
        tests.Add(("catalog: seven synthetic scenarios and four preserved controllers", Catalog));
        foreach (var scenario in ScenarioCatalog.Create())
        foreach (var controller in DefaultControllers())
        {
            var capturedScenario = scenario;
            var capturedController = controller;
            tests.Add(($"nominal: {scenario.Name} / {controller.Name}", () => Nominal(capturedScenario, capturedController)));
        }
        foreach (var controller in DefaultControllers())
        {
            var captured = controller;
            tests.Add(($"mirror: straight offsets / {controller.Name}", () => Mirror("straight-offset-left", "straight-offset-right", captured)));
            tests.Add(($"mirror: quarter arcs / {controller.Name}", () => Mirror("left-turn", "right-turn", captured)));
            tests.Add(($"mirror: arc offsets / {controller.Name}", () => Mirror("left-turn-offset", "right-turn-offset", captured)));
        }
        tests.Add(("simulation: angle clamp precedes rate limit with independent counts", Limits));
        tests.Add(("simulation: integer clock and partial horizon", Clock));
        tests.Add(("simulation: sub-step horizon retains initial-only timeout", SubStepHorizon));
        tests.Add(("simulation: finite settings reject overflowing final integer clock", OverflowingClock));
        tests.Add(("simulation: endpoint completion without snapping", NoSnap));
        tests.Add(("simulation: initial endpoint completes without invoking controller", InitialCompletion));
        tests.Add(("simulation: progress alone cannot complete", ProgressAlone));
        tests.Add(("simulation: endpoint proximity alone cannot complete", EndpointAlone));
        tests.Add(("timeout: stationary offset retained with samples and metrics", Stationary));
        tests.Add(("timeout: insufficient duration retains final state", TimeLimited));
        tests.Add(("timeout: steering-infeasible route retained", Infeasible));
        tests.Add(("metrics: independent known error series includes initial sample", AnalyticMetrics));
        tests.Add(("metrics: huge finite errors have finite robust RMS", HugeMetrics));
        foreach (var value in new[] { double.NaN, double.PositiveInfinity, double.NegativeInfinity })
        {
            var captured = value;
            tests.Add(($"failure: nonfinite steering {value}", () => NonfiniteController(captured)));
        }
        tests.Add(("failure: throwing controller retained with reason", ThrowingController));
        tests.Add(("failure: integration overflow retained with reason", OverflowIntegration));
        tests.Add(("simulation: exact repeated results on the same runtime", Determinism));
        tests.Add(("simulation: invalid settings and initial state rejected", Invalid));
    }

    private static IPathController[] DefaultControllers() => [new StanleyController(), new PurePursuitController()];

    private static void Catalog()
    {
        var scenarios = ScenarioCatalog.Create();
        Check.Equal(7, scenarios.Count);
        Check.Equal(7, scenarios.Select(s => s.Name).Distinct().Count());
        Check.Equal(4, ScenarioCatalog.Controllers().Count);
        Check.Near(12, scenarios.Single(s => s.Name == "straight").Route.Length);
        foreach (var scenario in scenarios)
        {
            Check.Near(1, scenario.InitialState.Speed);
            Check.Near(0, scenario.InitialState.Heading);
            Check.Near(0.02, scenario.Settings.TimeStepSeconds);
            Check.Near(25, scenario.Settings.MaxDurationSeconds);
            Check.Near(1, scenario.Settings.WheelbaseMeters);
            Check.Near(0.6, scenario.Settings.MaxSteeringRadians);
            Check.Near(1.2, scenario.Settings.MaxSteeringRateRadiansPerSecond);
            Check.Near(0.25, scenario.Settings.EndpointToleranceMeters);
        }
        Check.Near(0.5, scenarios.Single(s => s.Name == "straight-offset-left").InitialState.Position.Y);
        Check.Near(-0.5, scenarios.Single(s => s.Name == "straight-offset-right").InitialState.Position.Y);
        Check.Vector(new(6, 6), scenarios.Single(s => s.Name == "left-turn").Route.Points[^1]);
        Check.Vector(new(6, -6), scenarios.Single(s => s.Name == "right-turn").Route.Points[^1]);
    }

    private static void Nominal(Scenario scenario, IPathController controller)
    {
        var result = SimulationRunner.Run(scenario, controller);
        Check.Equal(SimulationStatus.Completed, result.Status, result.FailureReason ?? "Default nominal route must complete");
        Check.True(result.FailureReason is null, "Successful run has a failure reason.");
        var straight = scenario.Name.StartsWith("straight", StringComparison.Ordinal);
        var rmsBudget = straight ? 0.35 : 0.60;
        var maxBudget = straight ? 0.65 : 0.90;
        Check.True(result.Metrics.RmsCrossTrackErrorMeters <= rmsBudget, $"RMS exceeds predeclared {rmsBudget} m budget: {result.Metrics.RmsCrossTrackErrorMeters:R}.");
        Check.True(result.Metrics.MaxAbsoluteCrossTrackErrorMeters <= maxBudget, $"Peak error exceeds predeclared {maxBudget} m budget: {result.Metrics.MaxAbsoluteCrossTrackErrorMeters:R}.");
        Check.True(result.Metrics.ElapsedSeconds <= 25, "Completion exceeds predeclared 25 s budget.");
        Check.True(result.Metrics.FinalEndpointDistanceMeters <= 0.25, "Endpoint exceeds predeclared 0.25 m budget.");
        Check.True(result.Metrics.PathProgressMeters >= scenario.Route.Length - 0.25, "Completion lacks progress.");
        ValidateRecordedSamples(scenario, result);
        Check.ReadOnly(result.Samples);
    }

    private static void Mirror(string leftName, string rightName, IPathController controller)
    {
        var catalog = ScenarioCatalog.Create();
        var left = SimulationRunner.Run(catalog.Single(s => s.Name == leftName), controller);
        var right = SimulationRunner.Run(catalog.Single(s => s.Name == rightName), controller);
        Check.Equal(left.Status, right.Status);
        Check.Equal(left.Samples.Count, right.Samples.Count);
        for (var i = 0; i < left.Samples.Count; i++)
        {
            var a = left.Samples[i];
            var b = right.Samples[i];
            Check.Equal(a.Step, b.Step);
            Check.Equal(a.TimeSeconds, b.TimeSeconds);
            Check.Vector(new(a.State.Position.X, -a.State.Position.Y), b.State.Position, 1e-10);
            Check.Near(-a.State.Heading, b.State.Heading, 1e-10);
            Check.Near(-a.RequestedSteeringRadians, b.RequestedSteeringRadians, 1e-10);
            Check.Near(-a.AppliedSteeringRadians, b.AppliedSteeringRadians, 1e-10);
            Check.Near(-a.CrossTrackErrorMeters, b.CrossTrackErrorMeters, 1e-10);
            Check.Near(a.ProgressMeters, b.ProgressMeters, 1e-10);
        }
        Check.Near(left.Metrics.RmsCrossTrackErrorMeters, right.Metrics.RmsCrossTrackErrorMeters, 1e-10);
        Check.Near(left.Metrics.MaxAbsoluteCrossTrackErrorMeters, right.Metrics.MaxAbsoluteCrossTrackErrorMeters, 1e-10);
        Check.Near(left.Metrics.FinalEndpointDistanceMeters, right.Metrics.FinalEndpointDistanceMeters, 1e-10);
        Check.Equal(left.Metrics.SteeringSaturationCount, right.Metrics.SteeringSaturationCount);
        Check.Equal(left.Metrics.RateLimitCount, right.Metrics.RateLimitCount);
    }

    private static void Limits()
    {
        var calls = 0;
        var scenario = Make(new(0, 0), 0, 0, new(MaxDurationSeconds: 0.04));
        var result = SimulationRunner.Run(scenario, new Controller("alternating", (_, _, _) => ++calls == 1 ? 1 : -1));
        Check.Equal(SimulationStatus.TimedOut, result.Status);
        Check.Equal(3, result.Samples.Count);
        Check.Near(0, result.Samples[0].AppliedSteeringRadians);
        Check.Near(0.024, result.Samples[1].AppliedSteeringRadians);
        Check.Near(0, result.Samples[2].AppliedSteeringRadians);
        Check.Near(1, result.Samples[1].RequestedSteeringRadians);
        Check.Near(-1, result.Samples[2].RequestedSteeringRadians);
        Check.Equal(2, result.Metrics.SteeringSaturationCount);
        Check.Equal(2, result.Metrics.RateLimitCount);
        ValidateRecordedSamples(scenario, result);
    }

    private static void Clock()
    {
        var scenario = Make(new(0, 0), 0, 0, new(TimeStepSeconds: 0.01, MaxDurationSeconds: 0.025));
        var result = SimulationRunner.Run(scenario, Zero());
        Check.Equal(3, result.Samples.Count);
        Check.Equal(0d, result.Samples[0].TimeSeconds);
        Check.Equal(0.01, result.Samples[1].TimeSeconds);
        Check.Equal(0.02, result.Samples[2].TimeSeconds);
        Check.Equal(0.02, result.Metrics.ElapsedSeconds);
        Check.Equal(SimulationStatus.TimedOut, result.Status);
        ValidateRecordedSamples(scenario, result);
    }

    private static void NoSnap()
    {
        var scenario = new Scenario("no-snap", RouteFactory.Line(1), new(new(0, 0), 0, 1),
            new(TimeStepSeconds: 0.4, MaxDurationSeconds: 1));
        var result = SimulationRunner.Run(scenario, Zero());
        Check.Equal(SimulationStatus.Completed, result.Status);
        Check.Equal(3, result.Samples.Count);
        Check.Vector(new(0.8, 0), result.Samples[^1].State.Position);
        Check.Near(0.8, result.Metrics.ElapsedSeconds);
        Check.Near(0.2, result.Metrics.FinalEndpointDistanceMeters);
        ValidateRecordedSamples(scenario, result);
    }

    private static void SubStepHorizon()
    {
        var scenario = Make(new(0, 0), 0, 1, new(TimeStepSeconds: 0.02, MaxDurationSeconds: 0.01));
        var result = SimulationRunner.Run(scenario,
            new Controller("must-not-call", (_, _, _) => throw new InvalidOperationException("Unexpected controller call")));
        Check.Equal(SimulationStatus.TimedOut, result.Status);
        Check.Equal(1, result.Samples.Count);
        Check.Near(0, result.Metrics.ElapsedSeconds);
        ValidateRecordedSamples(scenario, result);
    }

    private static void OverflowingClock() => Check.Reject(() =>
        new SimulationSettings(TimeStepSeconds: Math.BitIncrement(double.MaxValue / 17), MaxDurationSeconds: double.MaxValue));

    private static void InitialCompletion()
    {
        var scenario = Make(new(10, 0), 0, 1, new());
        var result = SimulationRunner.Run(scenario, new Controller("must-not-call", (_, _, _) => throw new InvalidOperationException("Unexpected controller call")));
        Check.Equal(SimulationStatus.Completed, result.Status);
        Check.Equal(1, result.Samples.Count);
        Check.Near(0, result.Metrics.ElapsedSeconds);
        ValidateRecordedSamples(scenario, result);
    }

    private static void ProgressAlone()
    {
        var scenario = Make(new(9.9, 0.3), 0, 0, new(MaxDurationSeconds: 0.04));
        var result = SimulationRunner.Run(scenario, Zero());
        Check.True(result.Metrics.PathProgressMeters >= 9.75, "Test must establish sufficient progress.");
        Check.True(result.Metrics.FinalEndpointDistanceMeters > 0.25, "Test must establish insufficient endpoint proximity.");
        Check.Equal(SimulationStatus.TimedOut, result.Status);
        ValidateRecordedSamples(scenario, result);
    }

    private static void EndpointAlone()
    {
        var route = new Route(new[] { new Vector2D(0, 0), new Vector2D(10, 0), new Vector2D(0, 0.1) });
        var scenario = new Scenario("endpoint-only", route, new(new(0, 0), 0, 0), new(MaxDurationSeconds: 0.04));
        var result = SimulationRunner.Run(scenario, Zero());
        Check.Near(0.1, result.Metrics.FinalEndpointDistanceMeters);
        Check.Near(0, result.Metrics.PathProgressMeters);
        Check.Equal(SimulationStatus.TimedOut, result.Status);
        ValidateRecordedSamples(scenario, result);
    }

    private static void Stationary()
    {
        var scenario = Make(new(0, 0.5), 0.4, 0, new(MaxDurationSeconds: 0.1));
        var result = SimulationRunner.Run(scenario, new StanleyController());
        Check.Equal(SimulationStatus.TimedOut, result.Status);
        Check.Equal(6, result.Samples.Count);
        Check.True(!string.IsNullOrWhiteSpace(result.FailureReason), "Timeout must retain a reason.");
        foreach (var sample in result.Samples)
        {
            Check.Vector(scenario.InitialState.Position, sample.State.Position);
            Check.Near(scenario.InitialState.Heading, sample.State.Heading);
            Check.Near(0, sample.State.Speed);
        }
        Check.Near(0.5, result.Metrics.RmsCrossTrackErrorMeters);
        ValidateRecordedSamples(scenario, result);
    }

    private static void TimeLimited()
    {
        var scenario = Make(new(0, 0), 0, 1, new(MaxDurationSeconds: 0.1));
        var result = SimulationRunner.Run(scenario, new PurePursuitController());
        Check.Equal(SimulationStatus.TimedOut, result.Status);
        Check.Equal(6, result.Samples.Count);
        Check.Vector(new(0.1, 0), result.Samples[^1].State.Position);
        Check.Near(9.9, result.Metrics.FinalEndpointDistanceMeters);
        Check.True(!string.IsNullOrWhiteSpace(result.FailureReason), "Timeout must retain a reason.");
        ValidateRecordedSamples(scenario, result);
    }

    private static void Infeasible()
    {
        // Even with instant maximum steering, |heading| <= 3*tan(.001),
        // so y <= .5*9*tan(.001) < .0046 m; endpoint y=1 is unreachable.
        var route = new Route(new[] { new Vector2D(0, 0), new Vector2D(1, 1) });
        var scenario = new Scenario("infeasible-steering", route, new(new(0, 0), 0, 1),
            new(MaxDurationSeconds: 3, MaxSteeringRadians: 0.001, EndpointToleranceMeters: 0.01));
        var result = SimulationRunner.Run(scenario, new Controller("maximum-turn", (_, _, _) => 1));
        Check.Equal(SimulationStatus.TimedOut, result.Status);
        Check.Equal(151, result.Samples.Count);
        Check.True(result.Samples[^1].State.Position.Y < 0.0046, "Infeasibility bound was violated.");
        Check.True(result.Metrics.FinalEndpointDistanceMeters > 0.99, "Infeasible endpoint was not retained.");
        Check.True(result.Metrics.SteeringSaturationCount > 0, "Constraint must actually engage.");
        ValidateRecordedSamples(scenario, result);
    }

    private static void AnalyticMetrics()
    {
        var scenario = Make(new(0, 3), -Math.PI / 2, 1, new(TimeStepSeconds: 1, MaxDurationSeconds: 2));
        var result = SimulationRunner.Run(scenario, Zero());
        Check.Equal(3, result.Samples.Count);
        Check.Near(3, result.Samples[0].CrossTrackErrorMeters);
        Check.Near(2, result.Samples[1].CrossTrackErrorMeters);
        Check.Near(1, result.Samples[2].CrossTrackErrorMeters);
        Check.Near(Math.Sqrt(14d / 3), result.Metrics.RmsCrossTrackErrorMeters);
        Check.Near(3, result.Metrics.MaxAbsoluteCrossTrackErrorMeters);
        Check.Near(Math.Sqrt(101), result.Metrics.FinalEndpointDistanceMeters);
        Check.Near(2, result.Metrics.ElapsedSeconds);
        ValidateRecordedSamples(scenario, result);
    }

    private static void HugeMetrics()
    {
        var scenario = Make(new(0, 1e200), 0, 0, new(MaxDurationSeconds: 0.02));
        var result = SimulationRunner.Run(scenario, Zero());
        Check.Equal(SimulationStatus.TimedOut, result.Status);
        Check.Equal(2, result.Samples.Count);
        Check.Near(1e200, result.Metrics.RmsCrossTrackErrorMeters, 1e185);
        Check.Near(1e200, result.Metrics.MaxAbsoluteCrossTrackErrorMeters, 1e185);
        Check.Near(1e200, result.Metrics.FinalEndpointDistanceMeters, 1e185);
        foreach (var sample in result.Samples) Check.Near(1e200, sample.CrossTrackErrorMeters, 1e185);
    }

    private static void NonfiniteController(double value)
    {
        var scenario = Make(new(0, 0.5), 0, 1, new(MaxDurationSeconds: 0.1));
        var result = SimulationRunner.Run(scenario, new Controller("nonfinite", (_, _, _) => value));
        Check.Equal(SimulationStatus.NonFiniteValue, result.Status);
        Check.True(!string.IsNullOrWhiteSpace(result.FailureReason), "Nonfinite request must retain a reason.");
        Check.True(result.FailureReason!.Contains(value.ToString(System.Globalization.CultureInfo.InvariantCulture), StringComparison.OrdinalIgnoreCase),
            "Failure reason must identify the nonfinite request.");
        Check.Equal(1, result.Samples.Count);
        Check.Equal(scenario.InitialState, result.Samples[0].State);
        Check.Near(0.5, result.Metrics.RmsCrossTrackErrorMeters);
        ValidateRecordedSamples(scenario, result);
    }

    private static void ThrowingController()
    {
        var calls = 0;
        var scenario = Make(new(0, 0.5), 0, 1, new(MaxDurationSeconds: 0.1));
        var result = SimulationRunner.Run(scenario, new Controller("throws-later", (_, _, _) => ++calls == 2 ? throw new InvalidOperationException("deliberate-controller-failure") : 0));
        Check.Equal(SimulationStatus.ControllerError, result.Status);
        Check.Equal(2, result.Samples.Count);
        Check.True(result.FailureReason?.Contains("deliberate-controller-failure", StringComparison.Ordinal) == true,
            "Controller failure reason lost its diagnostic message.");
        Check.True(result.FailureReason?.Contains(nameof(InvalidOperationException), StringComparison.Ordinal) == true,
            "Controller failure reason lost its exception type.");
        Check.Near(0.02, result.Metrics.ElapsedSeconds);
        ValidateRecordedSamples(scenario, result);
    }

    private static void OverflowIntegration()
    {
        var scenario = Make(new(0, 0), 0, double.MaxValue, new(TimeStepSeconds: 2, MaxDurationSeconds: 2));
        var result = SimulationRunner.Run(scenario, Zero());
        Check.Equal(SimulationStatus.NonFiniteValue, result.Status);
        Check.True(!string.IsNullOrWhiteSpace(result.FailureReason), "Integration failure must retain a reason.");
        Check.Equal(2, result.Samples.Count, "Obtainable nonfinite candidate must remain visible");
        Check.Equal(scenario.InitialState, result.Samples[0].State);
        Check.Equal(double.PositiveInfinity, result.Samples[^1].State.Position.X);
        Check.Equal(2d, result.Samples[^1].TimeSeconds);
        Check.True(double.IsNaN(result.Samples[^1].CrossTrackErrorMeters), "Nonfinite candidate has undefined cross-track error.");
        Check.True(double.IsNaN(result.Samples[^1].ProgressMeters), "Nonfinite candidate has undefined progress.");
        Check.True(double.IsNaN(result.Metrics.RmsCrossTrackErrorMeters), "Nonfinite samples require explicitly undefined RMS.");
        Check.True(double.IsNaN(result.Metrics.MaxAbsoluteCrossTrackErrorMeters), "Nonfinite samples require explicitly undefined peak error.");
        Check.True(double.IsNaN(result.Metrics.PathProgressMeters), "Nonfinite samples require explicitly undefined progress.");
        Check.True(double.IsNaN(result.Metrics.FinalEndpointDistanceMeters), "Nonfinite samples require explicitly undefined endpoint distance.");
    }

    private static void Determinism()
    {
        foreach (var scenario in ScenarioCatalog.Create())
        foreach (var controller in ScenarioCatalog.Controllers())
        {
            var first = SimulationRunner.Run(scenario, controller);
            var second = SimulationRunner.Run(scenario, controller);
            Check.Equal(first.ScenarioName, second.ScenarioName);
            Check.Equal(first.ControllerName, second.ControllerName);
            Check.Equal(first.Status, second.Status);
            Check.Equal(first.FailureReason, second.FailureReason);
            Check.Equal(first.Metrics, second.Metrics);
            Check.Equal(first.Samples.Count, second.Samples.Count);
            for (var i = 0; i < first.Samples.Count; i++) Check.Equal(first.Samples[i], second.Samples[i]);
        }
    }

    private static void Invalid()
    {
        foreach (var bad in new[] { 0d, -1, double.NaN, double.PositiveInfinity })
        {
            Check.Reject(() => new SimulationSettings(TimeStepSeconds: bad));
            Check.Reject(() => new SimulationSettings(MaxDurationSeconds: bad));
            Check.Reject(() => new SimulationSettings(WheelbaseMeters: bad));
            Check.Reject(() => new SimulationSettings(MaxSteeringRadians: bad));
            Check.Reject(() => new SimulationSettings(MaxSteeringRateRadiansPerSecond: bad));
            Check.Reject(() => new SimulationSettings(EndpointToleranceMeters: bad));
        }
        Check.Reject(() => new SimulationSettings(MaxSteeringRadians: Math.PI / 2));
        Check.Reject(() => new SimulationSettings(MaxDurationSeconds: 1e9, TimeStepSeconds: 1e-9));
        foreach (var state in new[]
        {
            new VehicleState(new(double.NaN, 0), 0, 1), new VehicleState(new(0, double.PositiveInfinity), 0, 1),
            new VehicleState(new(0, 0), double.NaN, 1), new VehicleState(new(0, 0), 0, -1),
            new VehicleState(new(0, 0), 0, double.PositiveInfinity)
        }) Check.Reject(() => SimulationRunner.Run(new("invalid", RouteFactory.Line(10), state, new()), Zero()));
        Check.Reject(() => SimulationRunner.Run(null!, Zero()));
        Check.Reject(() => SimulationRunner.Run(Make(new(0, 0), 0, 1, new()), null!));
        Check.Reject(() => SimulationRunner.Run(new("invalid", null!, new(new(0, 0), 0, 1), new()), Zero()));
        Check.Reject(() => SimulationRunner.Run(new("invalid", RouteFactory.Line(10), new(new(0, 0), 0, 1), null!), Zero()));
    }

    private static Scenario Make(Vector2D position, double heading, double speed, SimulationSettings settings) =>
        new("test", RouteFactory.Line(10), new(position, heading, speed), settings);

    private static IPathController Zero() => new Controller("zero", (_, _, _) => 0);

    private sealed class Controller(string name, Func<VehicleState, Route, double, double> compute) : IPathController
    {
        public string Name => name;
        public double ComputeSteering(VehicleState state, Route route, double wheelbase) => compute(state, route, wheelbase);
    }

    private static void ValidateRecordedSamples(Scenario scenario, SimulationResult result)
    {
        Check.True(result.Samples.Count > 0, "Result must retain the initial sample.");
        Check.Equal(scenario.InitialState, result.Samples[0].State);
        Check.Equal(0, result.Samples[0].Step);
        Check.Equal(0d, result.Samples[0].TimeSeconds);
        Check.Near(0, result.Samples[0].RequestedSteeringRadians);
        Check.Near(0, result.Samples[0].AppliedSteeringRadians);
        var squaredSum = 0d;
        var maxError = 0d;
        var saturationCount = 0;
        var rateCount = 0;
        var previousApplied = 0d;
        var lastProgress = 0d;
        for (var i = 0; i < result.Samples.Count; i++)
        {
            var sample = result.Samples[i];
            Check.Equal(i, sample.Step);
            Check.Equal(i * scenario.Settings.TimeStepSeconds, sample.TimeSeconds);
            Check.True(sample.State.Position.IsFinite && double.IsFinite(sample.State.Heading), "Recorded state must be finite.");
            Check.Near(scenario.InitialState.Speed, sample.State.Speed);
            var projection = IndependentProjection(scenario.Route.Points, sample.State.Position);
            Check.Near(projection.Error, sample.CrossTrackErrorMeters, 1e-10, "Recorded cross-track error");
            Check.Near(projection.Progress, sample.ProgressMeters, 1e-10, "Recorded route progress");
            squaredSum += projection.Error * projection.Error;
            maxError = Math.Max(maxError, Math.Abs(projection.Error));
            lastProgress = projection.Progress;
            if (i == 0) continue;
            var maximum = scenario.Settings.MaxSteeringRadians;
            Check.True(Math.Abs(sample.AppliedSteeringRadians) <= maximum + 1e-14, "Applied steering exceeded angle bound.");
            var angleLimited = Math.Clamp(sample.RequestedSteeringRadians, -maximum, maximum);
            if (angleLimited != sample.RequestedSteeringRadians) saturationCount++;
            var changeLimit = scenario.Settings.MaxSteeringRateRadiansPerSecond * scenario.Settings.TimeStepSeconds;
            var expectedApplied = Math.Clamp(angleLimited, previousApplied - changeLimit, previousApplied + changeLimit);
            if (expectedApplied != angleLimited) rateCount++;
            Check.Near(expectedApplied, sample.AppliedSteeringRadians, 1e-12, "Applied steering sequence");
            Check.True(Math.Abs(sample.AppliedSteeringRadians - previousApplied) <= changeLimit + 1e-14, "Applied steering exceeded rate bound.");
            var prior = result.Samples[i - 1].State;
            Check.Vector(new(prior.Position.X + prior.Speed * Math.Cos(prior.Heading) * scenario.Settings.TimeStepSeconds,
                prior.Position.Y + prior.Speed * Math.Sin(prior.Heading) * scenario.Settings.TimeStepSeconds), sample.State.Position, 1e-11);
            var expectedHeading = WrapReference(prior.Heading + prior.Speed / scenario.Settings.WheelbaseMeters * Math.Tan(sample.AppliedSteeringRadians) * scenario.Settings.TimeStepSeconds);
            Check.Near(expectedHeading, sample.State.Heading, 1e-11, "Recorded bicycle heading");
            previousApplied = sample.AppliedSteeringRadians;
        }
        var last = result.Samples[^1];
        var endpoint = scenario.Route.Points[^1];
        var dx = last.State.Position.X - endpoint.X;
        var dy = last.State.Position.Y - endpoint.Y;
        Check.Near(Math.Sqrt(squaredSum / result.Samples.Count), result.Metrics.RmsCrossTrackErrorMeters, 1e-10, "Independent RMS");
        Check.Near(maxError, result.Metrics.MaxAbsoluteCrossTrackErrorMeters, 1e-10, "Independent peak error");
        Check.Equal(last.TimeSeconds, result.Metrics.ElapsedSeconds);
        Check.Near(lastProgress, result.Metrics.PathProgressMeters, 1e-10, "Independent final progress");
        Check.Near(Math.Sqrt(dx * dx + dy * dy), result.Metrics.FinalEndpointDistanceMeters, 1e-10, "Independent endpoint distance");
        Check.Equal(saturationCount, result.Metrics.SteeringSaturationCount);
        Check.Equal(rateCount, result.Metrics.RateLimitCount);
    }

    // Independent scalar projection: no production Project, AtDistance, vector products,
    // cumulative lengths, angle normalization, or metrics helpers are called here.
    private static (double Progress, double Error) IndependentProjection(IReadOnlyList<Vector2D> points, Vector2D query)
    {
        var bestSquared = double.PositiveInfinity;
        var cumulative = 0d;
        var bestProgress = 0d;
        var bestError = 0d;
        for (var i = 0; i < points.Count - 1; i++)
        {
            var ax = points[i].X;
            var ay = points[i].Y;
            var vx = points[i + 1].X - ax;
            var vy = points[i + 1].Y - ay;
            var squaredLength = vx * vx + vy * vy;
            var length = Math.Sqrt(squaredLength);
            var fraction = Math.Clamp(((query.X - ax) * vx + (query.Y - ay) * vy) / squaredLength, 0, 1);
            var dx = query.X - (ax + fraction * vx);
            var dy = query.Y - (ay + fraction * vy);
            var squaredDistance = dx * dx + dy * dy;
            if (squaredDistance < bestSquared)
            {
                bestSquared = squaredDistance;
                bestProgress = cumulative + fraction * length;
                bestError = (vx * dy - vy * dx) / length;
            }
            cumulative += length;
        }
        return (bestProgress, bestError);
    }

    private static double WrapReference(double value)
    {
        while (value >= Math.PI) value -= 2 * Math.PI;
        while (value < -Math.PI) value += 2 * Math.PI;
        return value;
    }
}
