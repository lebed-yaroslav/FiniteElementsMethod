using System.Diagnostics;
using System.Runtime.CompilerServices;
using Telma.Extensions;

namespace Telma;

public enum AngleMeasureUnits { Radians = 0, Degrees = 1 };

public readonly struct Vector1D(double x) : IVectorBase<Vector1D>
{
    public static int Dimensions => 1;
    public static Vector1D Zero { get; } = new(0);
    public static Vector1D XAxis { get; } = new(1);
    public static Vector1D[] Axes { get; } = [XAxis];

    public double X { get; } = x;

    public double Norm => X;
    public double NormSqr => X * X;
    public double MaxNorm => X;
    public Vector1D Round(int digits) => Math.Round(X, digits);

    public Vector1D As1D() => this;
    public Vector2D As2D() => new(X, 0);
    public Vector3D As3D() => new(X, 0, 0);

    public bool Equals(Vector1D b) => X == b.X;
    public override bool Equals(object? obj) => obj is Vector1D vec && Equals(vec);
    public override int GetHashCode() => X.GetHashCode();

    #region Constructors
    public static Vector1D FromSpan(ReadOnlySpan<double> span)
    {
        Debug.Assert(span.Length == Dimensions);
        return span[0];
    }
    #endregion

    #region Static operators
    public static implicit operator Vector1D(double v) => new(v);
    public static implicit operator double(Vector1D v) => v.X;
    public static Vector1D operator -(Vector1D a) => -a.X;
    public static Vector1D operator +(Vector1D a, Vector1D b) => a.X + b.X;
    public static Vector1D operator -(Vector1D a, Vector1D b) => a.X - b.X;
    public static Vector1D operator *(Vector1D a, double v) => a.X * v;
    public static double operator *(Vector1D a, Vector1D b) => a.X * b.X;
    public static Vector1D operator /(Vector1D a, double v) => a.X / v;
    public static bool operator ==(Vector1D a, Vector1D b) => a.X == b.X;
    public static bool operator !=(Vector1D a, Vector1D b) => a.X != b.X;

    public static bool operator >(Vector1D a, Vector1D b) => a.X > b.X;
    public static bool operator >=(Vector1D a, Vector1D b) => a.X >= b.X;
    public static bool operator <(Vector1D a, Vector1D b) => a.X < b.X;
    public static bool operator <=(Vector1D a, Vector1D b) => a.X <= b.X;

    public static Vector1D Min(Vector1D a, Vector1D b) => Math.Min(a.X, b.X);
    public static Vector1D Max(Vector1D a, Vector1D b) => Math.Max(a.X, b.X);
    #endregion
}

public readonly struct Vector2D(double x, double y) : IVectorBase<Vector2D>
{
    public static int Dimensions => 2;
    public static Vector2D Zero { get; } = new(0, 0);
    public static Vector2D XAxis { get; } = new(1, 0);
    public static Vector2D YAxis { get; } = new(0, 1);
    public static Vector2D[] Axes { get; } = [XAxis, YAxis];

    public double X { get; } = x;
    public double Y { get; } = y;

    public double Norm => Math.Sqrt(NormSqr);
    public double NormSqr => X * X + Y * Y;
    public double MaxNorm => Math.Max(Math.Abs(X), Math.Abs(Y));
    public Vector2D Round(int digits) => new(Math.Round(X, digits), Math.Round(Y, digits));

    public Vector1D As1D() => X;
    public Vector2D As2D() => this;
    public Vector3D As3D() => new(X, Y, 0);

    public bool Equals(Vector2D b) => X == b.X && Y == b.Y;
    public override bool Equals(object? obj) => obj is Vector2D vec && Equals(vec);
    public override int GetHashCode() => HashCode.Combine(X, Y);

    public void Deconstruct(out double x, out double y) => (x, y) = (X, Y);

    #region Constructors
    public static Vector2D FromSpan(ReadOnlySpan<double> span)
    {
        Debug.Assert(span.Length == Dimensions);
        return new(span[0], span[1]);
    }
    #endregion

    #region Parsing
    public override string ToString() => $"Vec({X}, {Y})";

    public static bool TryParse(string line, out Vector2D res)
    {
        double x, y;
        var words = line.Split([' ', '\t', ',', '>', '<', '(', ')'], StringSplitOptions.RemoveEmptyEntries);
        if (words[0] == "Vec")
        {
            if (words.Length != 3 || !double.TryParse(words[1], out x) || !double.TryParse(words[2], out y))
            {
                res = Zero;
                return false;
            }
            else { res = new Vector2D(x, y); return true; }
        }
        if (words.Length != 2 || !double.TryParse(words[0], out x) || !double.TryParse(words[1], out y))
        {
            res = Zero;
            return false;
        }
        else { res = new Vector2D(x, y); return true; }
    }

    public static Vector2D Parse(string line)
    {
        if (!TryParse(line, out Vector2D res))
            throw new FormatException($"Can't parse Vector2D from {line}");
        return res;
    }
    #endregion

    #region Static operators
    public static Vector2D operator -(Vector2D a) => new(-a.X, -a.Y);
    public static Vector2D operator +(Vector2D a, Vector2D b) => new(a.X + b.X, a.Y + b.Y);
    public static Vector2D operator -(Vector2D a, Vector2D b) => new(a.X - b.X, a.Y - b.Y);
    public static Vector2D operator /(Vector2D a, double v) => new(a.X / v, a.Y / v);
    public static Vector2D operator *(Vector2D a, double v) => new(a.X * v, a.Y * v);
    public static Vector2D operator *(double v, Vector2D a) => new(v * a.X, v * a.Y);
    public static double operator *(Vector2D a, Vector2D b) => a.X * b.X + a.Y * b.Y;
    public static bool operator ==(Vector2D a, Vector2D b) => a.X == b.X && a.Y == b.Y;
    public static bool operator !=(Vector2D a, Vector2D b) => a.X != b.X || a.Y != b.Y;

    public static bool operator >(Vector2D a, Vector2D b) => a.X > b.X && a.Y > b.Y;
    public static bool operator >=(Vector2D a, Vector2D b) => a.X >= b.X && a.Y >= b.Y;
    public static bool operator <(Vector2D a, Vector2D b) => b > a;
    public static bool operator <=(Vector2D a, Vector2D b) => b >= a;

    public static Vector2D Min(Vector2D a, Vector2D b) => new(Math.Min(a.X, b.X), Math.Min(a.Y, b.Y));
    public static Vector2D Max(Vector2D a, Vector2D b) => new(Math.Max(a.X, b.X), Math.Max(a.Y, b.Y));
    public static Vector2D Cross(Vector2D a) => new(a.Y, -a.X);
    public static double Mixed(Vector2D a, Vector2D b) => a.Y * b.X - a.X * b.Y;
    #endregion
}

public readonly struct Vector3D(double x, double y, double z) : IVectorBase<Vector3D>
{
    public static int Dimensions => 3;
    public static Vector3D Zero { get; } = new(0, 0, 0);
    public static Vector3D XAxis { get; } = new(1, 0, 0);
    public static Vector3D YAxis { get; } = new(0, 1, 0);
    public static Vector3D ZAxis { get; } = new(0, 0, 1);
    public static Vector3D[] Axes { get; } = [XAxis, YAxis, ZAxis];

    public double X { get; } = x;
    public double Y { get; } = y;
    public double Z { get; } = z;


    public double Norm => Math.Sqrt(NormSqr);
    public double NormSqr => X * X + Y * Y + Z * Z;
    public double MaxNorm => Math.Max(Math.Abs(X), Math.Max(Math.Abs(Y), Math.Abs(Z)));
    public Vector3D Round(int digits) => new(Math.Round(X, digits), Math.Round(Y, digits), Math.Round(Z, digits));

    public Vector1D As1D() => X;
    public Vector2D As2D() => new(X, Y);
    public Vector3D As3D() => this;

    public bool Equals(Vector3D b) => b.X == X && b.Y == Y && b.Z == Z;
    public override bool Equals(object? obj) => obj is Vector3D v && Equals(v);
    public override int GetHashCode() => HashCode.Combine(X, Y, Z);

    public void Deconstruct(out double x, out double y, out double z)
        => (x, y, z) = (X, Y, Z);

    #region Constructors
    public Vector3D(Vector2D vec, double z) : this(vec.X, vec.Y, z) { }
    public Vector3D(double x, Vector2D vec) : this(x, vec.X, vec.Y) { }
    public static Vector3D FromSpan(ReadOnlySpan<double> span)
    {
        Debug.Assert(span.Length == Dimensions);
        return new(span[0], span[1], span[2]);
    }
    #endregion

    #region Parsing
    public override string ToString() => $"Vec({X}, {Y}, {Z})";

    public static bool TryParse(string line, out Vector3D res)
    {
        double x, y, z;
        var words = line.Split([' ', '\t', ',', '(', ')', '<', '>'], StringSplitOptions.RemoveEmptyEntries);
        if (words[0] == "Vec")
        {
            if (words.Length != 4 || !double.TryParse(words[1], out x) || !double.TryParse(words[2], out y)
                || !double.TryParse(words[3], out z))
            {
                res = Zero;
                return false;
            }
            res = new Vector3D(x, y, z);
            return true;
        }
        if (words.Length != 3 || !double.TryParse(words[0], out x) || !double.TryParse(words[1], out y)
            || !double.TryParse(words[2], out z))
        {
            res = Zero;
            return false;
        }

        res = new Vector3D(x, y, z);
        return true;
    }

    public static Vector3D Parse(string line)
    {
        if (!TryParse(line, out var res))
            throw new FormatException($"Can't parse {nameof(Vector3D)} from {line}");
        return res;
    }
    #endregion

    #region Static operators
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3D operator -(Vector3D a) => new(-a.X, -a.Y, -a.Z);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3D operator +(Vector3D a) => a;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double operator *(Vector3D a, Vector3D b) => a.X * b.X + a.Y * b.Y + a.Z * b.Z;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3D operator *(double a, Vector3D b) => new(a * b.X, a * b.Y, a * b.Z);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3D operator *(Vector3D b, double a) => new(a * b.X, a * b.Y, a * b.Z);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3D operator /(Vector3D a, double v) => new(a.X / v, a.Y / v, a.Z / v);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3D operator +(Vector3D a, Vector3D b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3D operator -(Vector3D a, Vector3D b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(Vector3D a, Vector3D b) => a.X == b.X && a.Y == b.Y && a.Z == b.Z;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(Vector3D a, Vector3D b) => a.X != b.X || a.Y != b.Y || a.Z != b.Z;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator >(Vector3D a, Vector3D b) => a.X > b.X && a.Y > b.Y && a.Z > b.Z;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator >=(Vector3D a, Vector3D b) => a.X >= b.X && a.Y >= b.Y && a.Z >= b.Z;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator <(Vector3D a, Vector3D b) => b > a;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator <=(Vector3D a, Vector3D b) => b >= a;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3D Cross(Vector3D v1, Vector3D v2) =>
        new(v1.Y * v2.Z - v2.Y * v1.Z, v1.Z * v2.X - v1.X * v2.Z, v1.X * v2.Y - v1.Y * v2.X);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double Mixed(Vector3D v1, Vector3D v2, Vector3D v3) =>
        (v1.Y * v2.Z - v2.Y * v1.Z) * v3.X + (v1.Z * v2.X - v1.X * v2.Z) * v3.Y + (v1.X * v2.Y - v1.Y * v2.X) * v3.Z;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3D Min(Vector3D a, Vector3D b) =>
        new(Math.Min(a.X, b.X), Math.Min(a.Y, b.Y), Math.Min(a.Z, b.Z));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3D Max(Vector3D a, Vector3D b) =>
        new(Math.Max(a.X, b.X), Math.Max(a.Y, b.Y), Math.Max(a.Z, b.Z));
    #endregion
}
