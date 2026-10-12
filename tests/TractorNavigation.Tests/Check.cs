using TractorNavigation;

namespace TractorNavigation.Tests;

internal static class Check
{
    internal static void True(bool condition, string message)
    {
        if (!condition) throw new InvalidOperationException(message);
    }

    internal static void Equal<T>(T expected, T actual, string message = "Values differ")
    {
        if (!EqualityComparer<T>.Default.Equals(expected, actual))
            throw new InvalidOperationException($"{message}: expected {expected}, actual {actual}.");
    }

    internal static void Near(double expected, double actual, double tolerance = 1e-12, string message = "Values differ")
    {
        if (!double.IsFinite(expected) || !double.IsFinite(actual) || Math.Abs(expected - actual) > tolerance)
            throw new InvalidOperationException($"{message}: expected {expected:R}, actual {actual:R}, tolerance {tolerance:R}.");
    }

    internal static void Vector(Vector2D expected, Vector2D actual, double tolerance = 1e-12)
    {
        Near(expected.X, actual.X, tolerance, "x");
        Near(expected.Y, actual.Y, tolerance, "y");
    }

    internal static void Reject(Action action)
    {
        try { action(); }
        catch (ArgumentException) { return; }
        throw new InvalidOperationException("Invalid input was not rejected with ArgumentException.");
    }

    internal static void ReadOnly<T>(IReadOnlyList<T> values)
    {
        if (values is IList<T> writable)
        {
            True(writable.IsReadOnly, "Collection exposes mutable IList storage.");
            try { writable[0] = writable[0]; }
            catch (NotSupportedException) { return; }
            throw new InvalidOperationException("Collection permits writes.");
        }
        True(values is not T[], "Array exposes mutable storage.");
    }
}
