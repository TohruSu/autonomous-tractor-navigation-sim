namespace TractorNavigation;

/// <summary>Scaled arithmetic for geometric quantities that can overflow when squared.</summary>
internal static class NumericGeometry
{
    internal const int MaximumRoutePoints = 1_000_001;

    internal static double Hypot(double x, double y)
    {
        var largest = Math.Max(Math.Abs(x), Math.Abs(y));
        if (double.IsInfinity(largest))
        {
            return double.PositiveInfinity;
        }

        if (largest == 0 || double.IsNaN(largest))
        {
            return largest;
        }

        x /= largest;
        y /= largest;
        return largest * Math.Sqrt(x * x + y * y);
    }

    internal static void RequireFinite(Vector2D vector, string parameterName)
    {
        if (!vector.IsFinite)
        {
            throw new ArgumentOutOfRangeException(parameterName, "Both coordinates must be finite.");
        }
    }

    internal static void RequirePositive(double value, string parameterName)
    {
        if (!double.IsFinite(value) || value <= 0)
        {
            throw new ArgumentOutOfRangeException(parameterName, "The value must be finite and positive.");
        }
    }

    // Half differences are used only for coordinates whose subtraction overflowed.
    // Preserve each raw component and its exponent: a shared normalized vector can
    // lose a small perpendicular component beside a very large along-track one.
    internal static ScaledVector Difference(Vector2D first, Vector2D second)
    {
        var dx = first.X - second.X;
        var dy = first.Y - second.Y;
        var xExponent = 0;
        var yExponent = 0;
        if (!double.IsFinite(dx))
        {
            dx = first.X * 0.5 - second.X * 0.5;
            xExponent = 1;
        }

        if (!double.IsFinite(dy))
        {
            dy = first.Y * 0.5 - second.Y * 0.5;
            yExponent = 1;
        }

        var exponent = Math.Max(xExponent, yExponent);
        var sharedX = Math.ScaleB(dx, xExponent - exponent);
        var sharedY = Math.ScaleB(dy, yExponent - exponent);
        var scale = Math.Max(Math.Abs(sharedX), Math.Abs(sharedY));
        return scale == 0
            ? new ScaledVector(0, 0, 0, 0, 0, 0, 0, 0)
            : new ScaledVector(sharedX / scale, sharedY / scale, scale, exponent, dx, dy, xExponent, yExponent);
    }

    internal readonly record struct ScaledVector(
        double X, double Y, double Scale, int Exponent,
        double RawX, double RawY, int XExponent, int YExponent)
    {
        internal bool IsZero => Scale == 0;

        internal Magnitude Length => IsZero
            ? Magnitude.Zero
            : Magnitude.FromDouble(Scale).Times(Math.Sqrt(X * X + Y * Y)).Shift(Exponent);

        internal SignedMagnitude DotWithUnit(Vector2D unit) =>
            ProductSum(unit.X, RawX, XExponent, unit.Y, RawY, YExponent);

        internal double CrossWithUnit(Vector2D unit)
        {
            var cross = ProductSum(unit.X, RawY, YExponent, -unit.Y, RawX, XExponent);
            var value = cross.Absolute.ToDouble();
            if (!double.IsFinite(value))
            {
                throw new OverflowException("The cross-track error is too large to represent as a finite number.");
            }

            return Math.CopySign(value, cross.Sign);
        }

        private static SignedMagnitude ProductSum(
            double firstFactor, double firstComponent, int firstExponent,
            double secondFactor, double secondComponent, int secondExponent)
        {
            var first = Magnitude.FromDouble(Math.Abs(firstFactor)).Times(Math.Abs(firstComponent)).Shift(firstExponent);
            var second = Magnitude.FromDouble(Math.Abs(secondFactor)).Times(Math.Abs(secondComponent)).Shift(secondExponent);
            if (first.Significand == 0 && second.Significand == 0)
            {
                return new SignedMagnitude(0, Magnitude.Zero);
            }

            var exponent = first.Significand == 0 ? second.Exponent
                : second.Significand == 0 ? first.Exponent
                : Math.Max(first.Exponent, second.Exponent);
            var sum = Math.CopySign(Math.ScaleB(first.Significand, first.Exponent - exponent), firstFactor * firstComponent)
                + Math.CopySign(Math.ScaleB(second.Significand, second.Exponent - exponent), secondFactor * secondComponent);
            return new SignedMagnitude(Math.Sign(sum), Magnitude.FromDouble(Math.Abs(sum)).Shift(exponent));
        }
    }

    internal readonly record struct SignedMagnitude(double Sign, Magnitude Absolute);

    // A nonnegative value stored as significand * 2^exponent. The significand
    // is in [1,2), or zero. Comparisons and atan2 need no overflowing product.
    internal readonly record struct Magnitude(double Significand, int Exponent) : IComparable<Magnitude>
    {
        internal static Magnitude Zero => new(0, 0);

        internal static Magnitude FromDouble(double value)
        {
            if (value == 0)
            {
                return Zero;
            }

            var exponent = Math.ILogB(value);
            return new Magnitude(Math.ScaleB(value, -exponent), exponent);
        }

        internal Magnitude Times(double factor) => Times(FromDouble(factor));

        internal Magnitude Times(Magnitude other)
        {
            if (Significand == 0 || other.Significand == 0)
            {
                return Zero;
            }

            var significand = Significand * other.Significand;
            var exponent = Exponent + other.Exponent;
            if (significand >= 2)
            {
                significand *= 0.5;
                exponent++;
            }

            return new Magnitude(significand, exponent);
        }

        internal Magnitude Shift(int exponent) => Significand == 0 ? Zero : new(Significand, Exponent + exponent);

        internal double ToDouble() => Math.ScaleB(Significand, Exponent);

        internal static Magnitude Sum(double first, double second)
        {
            var scale = Math.Max(first, second);
            return scale == 0 ? Zero : FromDouble(scale).Times(first / scale + second / scale);
        }

        internal static double Atan2(Magnitude numerator, Magnitude denominator, double numeratorSign)
        {
            if (numerator.Significand == 0)
            {
                return 0;
            }

            var exponent = Math.Max(numerator.Exponent, denominator.Exponent);
            return Math.Atan2(
                Math.CopySign(Math.ScaleB(numerator.Significand, numerator.Exponent - exponent), numeratorSign),
                Math.ScaleB(denominator.Significand, denominator.Exponent - exponent));
        }

        public int CompareTo(Magnitude other)
        {
            if (Significand == 0 || other.Significand == 0)
            {
                return Significand.CompareTo(other.Significand);
            }

            var exponentOrder = Exponent.CompareTo(other.Exponent);
            return exponentOrder == 0 ? Significand.CompareTo(other.Significand) : exponentOrder;
        }
    }
}
