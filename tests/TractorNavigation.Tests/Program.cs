using TractorNavigation.Tests;

var tests = new List<(string Name, Action Run)>();
GeometryTests.Register(tests);
ControllerTests.Register(tests);
BicycleTests.Register(tests);
SimulationTests.Register(tests);

var failures = new List<(string Name, Exception Error)>();
foreach (var (name, run) in tests)
{
    try
    {
        run();
        Console.WriteLine($"PASS {name}");
    }
    catch (Exception error)
    {
        failures.Add((name, error));
        Console.WriteLine($"FAIL {name}: {error.Message}");
    }
}

Console.WriteLine($"Cases: {tests.Count}; passed: {tests.Count - failures.Count}; failed: {failures.Count}; skipped: 0.");
foreach (var (name, error) in failures)
    Console.Error.WriteLine($"{name}: {error}");
return failures.Count == 0 ? 0 : 1;
