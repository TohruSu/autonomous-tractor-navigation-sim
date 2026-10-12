using System.Globalization;

namespace TractorNavigation;

/// <summary>Deterministic fixed-step simulation with bounded steering actuators.</summary>
public static class SimulationRunner
{
    public static SimulationResult Run(Scenario scenario, IPathController controller)
    {
        ArgumentNullException.ThrowIfNull(scenario);
        ArgumentNullException.ThrowIfNull(controller);
        ArgumentException.ThrowIfNullOrWhiteSpace(scenario.Name);
        ArgumentNullException.ThrowIfNull(scenario.Route);
        ArgumentNullException.ThrowIfNull(scenario.Settings);
        int maximumSteps = scenario.Settings.ValidateAndGetStepCount();
        KinematicBicycle.ValidateState(scenario.InitialState, nameof(scenario.InitialState));

        Route route = scenario.Route;
        SimulationSettings settings = scenario.Settings;
        Vector2D endpoint = route.Points[^1];
        var samples = new List<SimulationSample>(Math.Min(maximumSteps + 1, 4096));
        VehicleState state = scenario.InitialState;
        int saturationCount = 0;
        int rateLimitCount = 0;
        string controllerName = controller.GetType().Name;

        bool initialObservationValid = TryObserve(
            state, route, endpoint,
            out double crossTrackError, out double progress, out double endpointDistance,
            out string? observationFailure);
        samples.Add(new SimulationSample(0, 0, state, 0, 0, crossTrackError, progress));
        try
        {
            string name = controller.Name;
            if (string.IsNullOrWhiteSpace(name))
            {
                return Finish(SimulationStatus.ControllerError,
                    "Controller name was null or empty at step 0 (t=0 seconds).");
            }

            controllerName = name;
        }
        catch (Exception exception) when (IsOrdinaryException(exception))
        {
            return Finish(SimulationStatus.ControllerError,
                $"Controller name failed at step 0 (t=0 seconds): {exception.GetType().Name}: {exception.Message}");
        }

        if (!initialObservationValid)
        {
            return Finish(SimulationStatus.NonFiniteValue,
                $"Initial observation failed at step 0 (t=0 seconds): {observationFailure}");
        }

        if (HasCompleted(progress, endpointDistance))
        {
            return Finish(SimulationStatus.Completed, null);
        }

        double previousAppliedSteering = 0;
        // The complete reachable steering interval is only 2*maxSteering wide.
        // This cap also handles an overflowing rate*dt product without discarding
        // otherwise valid finite settings.
        double rateAllowance = Math.Min(
            2 * settings.MaxSteeringRadians,
            settings.MaxSteeringRateRadiansPerSecond * settings.TimeStepSeconds);

        for (int step = 1; step <= maximumSteps; step++)
        {
            double commandTime = (step - 1) * settings.TimeStepSeconds;
            double requestedSteering;
            try
            {
                requestedSteering = controller.ComputeSteering(state, route, settings.WheelbaseMeters);
            }
            catch (Exception exception) when (IsOrdinaryException(exception))
            {
                return Finish(SimulationStatus.ControllerError,
                    string.Create(CultureInfo.InvariantCulture,
                        $"Controller failed while preparing step {step} at t={commandTime:R} seconds: {exception.GetType().Name}: {exception.Message}"));
            }

            if (!double.IsFinite(requestedSteering))
            {
                return Finish(SimulationStatus.NonFiniteValue,
                    string.Create(CultureInfo.InvariantCulture,
                        $"Controller returned {requestedSteering:R} while preparing step {step} at t={commandTime:R} seconds."));
            }

            double angleLimitedSteering = Math.Clamp(
                requestedSteering, -settings.MaxSteeringRadians, settings.MaxSteeringRadians);
            if (angleLimitedSteering != requestedSteering)
            {
                saturationCount++;
            }

            double appliedSteering = angleLimitedSteering;
            double requestedChange = angleLimitedSteering - previousAppliedSteering;
            if (Math.Abs(requestedChange) > rateAllowance)
            {
                appliedSteering = previousAppliedSteering + Math.CopySign(rateAllowance, requestedChange);
                rateLimitCount++;
            }

            double time = step * settings.TimeStepSeconds;
            try
            {
                state = KinematicBicycle.Step(state, appliedSteering, settings.WheelbaseMeters, settings.TimeStepSeconds);
            }
            catch (KinematicBicycle.NonFiniteIntegrationException exception)
            {
                // Preserve the actual invalid candidate and explicitly undefined
                // geometric observations instead of dropping the failed step.
                samples.Add(new SimulationSample(
                    step, time, exception.CandidateState,
                    requestedSteering, appliedSteering, double.NaN, double.NaN));
                return Finish(SimulationStatus.NonFiniteValue,
                    string.Create(CultureInfo.InvariantCulture,
                        $"Integration failed at step {step} (t={time:R} seconds): {exception.Message}"));
            }

            bool observationValid = TryObserve(
                state, route, endpoint,
                out crossTrackError, out progress, out endpointDistance,
                out observationFailure);
            samples.Add(new SimulationSample(
                step, time, state,
                requestedSteering, appliedSteering, crossTrackError, progress));
            previousAppliedSteering = appliedSteering;

            if (!observationValid)
            {
                return Finish(SimulationStatus.NonFiniteValue,
                    string.Create(CultureInfo.InvariantCulture,
                        $"Observation failed at step {step} (t={time:R} seconds): {observationFailure}"));
            }

            if (HasCompleted(progress, endpointDistance))
            {
                return Finish(SimulationStatus.Completed, null);
            }
        }

        return Finish(SimulationStatus.TimedOut,
            string.Create(CultureInfo.InvariantCulture,
                $"The endpoint was not reached within {maximumSteps} fixed integration steps (duration limit {settings.MaxDurationSeconds:R} seconds)."));

        bool HasCompleted(double observedProgress, double distanceToEndpoint) =>
            observedProgress >= route.Length - settings.EndpointToleranceMeters
            && distanceToEndpoint <= settings.EndpointToleranceMeters;

        SimulationResult Finish(SimulationStatus status, string? failureReason) => new(
            scenario.Name,
            controllerName,
            status,
            failureReason,
            samples,
            MetricsCalculator.Calculate(samples, endpoint, saturationCount, rateLimitCount));
    }

    private static bool TryObserve(
        VehicleState state,
        Route route,
        Vector2D endpoint,
        out double crossTrackError,
        out double progress,
        out double endpointDistance,
        out string? failureReason)
    {
        crossTrackError = double.NaN;
        progress = double.NaN;
        endpointDistance = double.NaN;
        failureReason = null;
        try
        {
            RouteProjection projection = route.Project(state.Position);
            crossTrackError = projection.SignedCrossTrackError;
            progress = projection.Distance;
            if (!projection.Position.IsFinite || !double.IsFinite(projection.Distance)
                || !double.IsFinite(projection.SignedCrossTrackError) || !double.IsFinite(projection.TangentHeading))
            {
                failureReason = string.Create(CultureInfo.InvariantCulture,
                    $"Route projection produced nonfinite values (x={projection.Position.X:R}, y={projection.Position.Y:R}, progress={projection.Distance:R}, cross-track error={projection.SignedCrossTrackError:R}, heading={projection.TangentHeading:R}).");
                return false;
            }
        }
        catch (ArithmeticException exception)
        {
            failureReason = $"Route projection could not produce finite observations: {exception.Message}";
            return false;
        }

        endpointDistance = MetricsCalculator.Distance(state.Position, endpoint);
        if (!double.IsFinite(endpointDistance))
        {
            failureReason = "The endpoint distance is not representable as a finite value.";
            return false;
        }

        return true;
    }

    private static bool IsOrdinaryException(Exception exception) =>
        exception is not OutOfMemoryException and not StackOverflowException and not AccessViolationException;
}
