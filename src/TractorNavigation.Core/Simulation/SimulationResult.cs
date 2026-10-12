namespace TractorNavigation;

/// <summary>A run outcome whose trajectory is copied into a read-only snapshot.</summary>
public sealed record SimulationResult
{
    public string ScenarioName { get; }
    public string ControllerName { get; }
    public SimulationStatus Status { get; }
    public string? FailureReason { get; }
    public IReadOnlyList<SimulationSample> Samples { get; }
    public EvaluationMetrics Metrics { get; }

    public SimulationResult(
        string ScenarioName,
        string ControllerName,
        SimulationStatus Status,
        string? FailureReason,
        IReadOnlyList<SimulationSample> Samples,
        EvaluationMetrics Metrics)
    {
        ArgumentNullException.ThrowIfNull(ScenarioName);
        ArgumentNullException.ThrowIfNull(ControllerName);
        ArgumentNullException.ThrowIfNull(Samples);
        ArgumentNullException.ThrowIfNull(Metrics);
        var snapshot = new SimulationSample[Samples.Count];
        for (int index = 0; index < snapshot.Length; index++)
        {
            snapshot[index] = Samples[index]
                ?? throw new ArgumentException("A trajectory cannot contain null samples.", nameof(Samples));
        }

        this.ScenarioName = ScenarioName;
        this.ControllerName = ControllerName;
        this.Status = Status;
        this.FailureReason = FailureReason;
        this.Samples = Array.AsReadOnly(snapshot);
        this.Metrics = Metrics;
    }

    public void Deconstruct(
        out string ScenarioName,
        out string ControllerName,
        out SimulationStatus Status,
        out string? FailureReason,
        out IReadOnlyList<SimulationSample> Samples,
        out EvaluationMetrics Metrics)
    {
        ScenarioName = this.ScenarioName;
        ControllerName = this.ControllerName;
        Status = this.Status;
        FailureReason = this.FailureReason;
        Samples = this.Samples;
        Metrics = this.Metrics;
    }
}
