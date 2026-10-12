namespace TractorNavigation;

public static class Angles
{
    /// <summary>Wraps a finite angle to [-pi, pi), including pi as -pi.</summary>
    public static double Normalize(double radians)
    {
        if (!double.IsFinite(radians))
        {
            throw new ArgumentOutOfRangeException(nameof(radians), "The angle must be finite.");
        }

        var wrapped = radians % (2 * Math.PI);
        if (wrapped >= Math.PI)
        {
            wrapped -= 2 * Math.PI;
        }
        else if (wrapped < -Math.PI)
        {
            wrapped += 2 * Math.PI;
        }

        return wrapped == 0 ? 0 : wrapped;
    }
}
