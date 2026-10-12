namespace TractorNavigation;

/// <summary>A Cartesian vector in meters, with counterclockwise-positive orientation.</summary>
public readonly record struct Vector2D(double X, double Y)
{
    public bool IsFinite => double.IsFinite(X) && double.IsFinite(Y);

    public double LengthSquared => X * X + Y * Y;

    public double Length => NumericGeometry.Hypot(X, Y);

    public static Vector2D operator +(Vector2D left, Vector2D right) =>
        new(left.X + right.X, left.Y + right.Y);

    public static Vector2D operator -(Vector2D left, Vector2D right) =>
        new(left.X - right.X, left.Y - right.Y);

    public static Vector2D operator *(Vector2D vector, double scalar) =>
        new(vector.X * scalar, vector.Y * scalar);

    public static Vector2D operator *(double scalar, Vector2D vector) => vector * scalar;

    public static Vector2D operator /(Vector2D vector, double scalar) =>
        new(vector.X / scalar, vector.Y / scalar);

    public static double Dot(Vector2D left, Vector2D right) => left.X * right.X + left.Y * right.Y;

    public static double Cross(Vector2D left, Vector2D right) => left.X * right.Y - left.Y * right.X;
}
