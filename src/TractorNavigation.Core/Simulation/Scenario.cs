namespace TractorNavigation;

public sealed record Scenario(string Name, Route Route, VehicleState InitialState, SimulationSettings Settings);
