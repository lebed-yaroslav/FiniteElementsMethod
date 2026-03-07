using System.Diagnostics.CodeAnalysis;
using System.Globalization;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Telma.Extensions;

namespace Telma;

public enum AngleMeasureUnits { amuRadians = 0, amuDegrees = 1 };

public readonly struct Vector1D(double x) : IVectorTraits<Vector1D>
{
    public static int Dimensions => 1;
    public static Vector1D Zero { get; } = new(0);
    public static Vector1D XAxis { get; } = new(1);
    public static Vector1D[] Axes { get; } = [XAxis];

    public double X { get; } = x;

    public double Norm => X;
    public double NormSqr => X * X;

    public ReadOnlySpan<double> AsSpan() =>
         MemoryMarshal.Cast<Vector1D, double>(MemoryMarshal.CreateReadOnlySpan(in this, 1));

    public static implicit operator Vector1D(double v) => new(v);
    public static implicit operator double(Vector1D v) => v.X;

}

public readonly struct Vector2D : IVectorTraits<Vector2D>, IEquatable<Vector2D>
{
    public static int Dimensions => 2;
    public static Vector2D Zero { get; } = new(0, 0);
    public static Vector2D XAxis { get; } = new(1, 0);
    public static Vector2D YAxis { get; } = new(0, 1);
    public static Vector2D[] Axes { get; } = [XAxis, YAxis];

    public double X { get; }
    public double Y { get; }
    public ReadOnlySpan<double> AsSpan() => MemoryMarshal.Cast<Vector2D, double>(MemoryMarshal.CreateReadOnlySpan(in this, 1));

    public Vector2D(double x, double y)
    {
        X = x;
        Y = y;
    }

    public void Deconstruct(out double x, out double y) => (x, y) = (X, Y);

    /// <summary>
    ///  из полярных координат в декартовы
    /// </summary>
    public Vector2D(double a, double b, AngleMeasureUnits measure) : this()
    {
        if (measure == AngleMeasureUnits.amuRadians)
        {
            X = a * Math.Cos(b);
            Y = a * Math.Sin(b);
        }
        else
        {
            double c = b * Math.PI / 180;
            X = a * Math.Cos(c);
            Y = a * Math.Sin(c);
        }
    }
    public Vector2D(ReadOnlySpan<double> arr)
    {
        X = arr[0];
        Y = arr[1];
    }
    public static double Distance(Vector2D a, Vector2D b) => (a - b).Norm;

    public static double SqrDistance(Vector2D a, Vector2D b)
    {
        Vector2D diff = a - b;
        return diff * diff;
    }
    public double Distance(Vector2D b) => Distance(this, b);

    public double SqrDistance(Vector2D b) => SqrDistance(this, b);

    public double Norm => Math.Sqrt(NormSqr);
    public double NormSqr => X * X + Y * Y;

    public Vector2D Normalize() => this / Norm;

    public override string ToString() => $"Vec({X}, {Y})";

    public override bool Equals(object? obj) => obj is Vector2D v && Equals(v);

    public override int GetHashCode() => HashCode.Combine(X, Y);

    public bool Equals(Vector2D a) => a.X == X && a.Y == Y;
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
    private Vector2D Round(int digits) => new Vector2D(Math.Round(X, digits), Math.Round(Y, digits));

    public static Vector2D Vec(double x, double y) => new Vector2D(x, y);
    public Vector3D As3D() => new Vector3D(X, Y, 0);
    #region Static operators

    public static Vector2D operator -(Vector2D a) => new Vector2D(-a.X, -a.Y);

    public static Vector2D operator +(Vector2D a, Vector2D b) => new Vector2D(a.X + b.X, a.Y + b.Y);

    public static Vector2D operator -(Vector2D a, Vector2D b) => new Vector2D(a.X - b.X, a.Y - b.Y);

    public static Vector2D operator /(Vector2D a, double v) => new Vector2D(a.X / v, a.Y / v);

    public static Vector2D operator *(Vector2D a, double v) => new Vector2D(a.X * v, a.Y * v);

    public static Vector2D operator *(double v, Vector2D a) => new Vector2D(v * a.X, v * a.Y);

    public static double operator *(Vector2D a, Vector2D b) => a.X * b.X + a.Y * b.Y;

    public static bool operator ==(Vector2D a, Vector2D b) => a.X == b.X && a.Y == b.Y;

    public static bool operator !=(Vector2D a, Vector2D b) => a.X != b.X || a.Y != b.Y;

    public static Vector2D Cross(Vector2D v1) => new Vector2D(v1.Y, -v1.X);

    public static double Mixed(Vector2D v1, Vector2D v2) => v1.Y * v2.X - v1.X * v2.Y;

    public static Vector2D Sum(Vector2D a, Vector2D b) => new Vector2D(a.X + b.X, a.Y + b.Y);

    #endregion
    #region EqualityComparer

    private class EqualityComparer : IEqualityComparer<Vector2D>
    {
        public int Digits { get; set; }

        public bool Equals(Vector2D v1, Vector2D v2)
        {
            return v1.Round(Digits) == v2.Round(Digits);
        }

        public int GetHashCode(Vector2D obj)
        {
            return obj.Round(Digits).GetHashCode();
        }
    }

    public static IEqualityComparer<Vector2D> CreateComparer(int digits = 7) => new EqualityComparer { Digits = digits };

    #endregion
}

public readonly struct Vector3D : IVectorTraits<Vector3D>, INumberBase<Vector3D>, IMultiplyOperators<Vector3D, double, Vector3D>
{
    public static int Dimensions => 3;
    public static Vector3D Zero { get; } = new(0, 0, 0);
    public static Vector3D XAxis { get; } = new(1, 0, 0);
    public static Vector3D YAxis { get; } = new(0, 1, 0);
    public static Vector3D ZAxis { get; } = new(0, 0, 1);
    public static Vector3D[] Axes { get; } = [XAxis, YAxis, ZAxis];

    public double X { get; }
    public double Y { get; }
    public double Z { get; }

    public Vector3D(double x, double y, double z)
    {
        X = x;
        Y = y;
        Z = z;
    }
    public Vector3D(Vector2D vec, double z)
    {
        X = vec.X;
        Y = vec.Y;
        Z = z;
    }
    public Vector3D(ReadOnlySpan<double> arr)
    {
#if DEBUG
        if (arr.Length != 3) throw new ArgumentException();
#endif
        X = arr[0];
        Y = arr[1];
        Z = arr[2];
    }
    public double Distance(Vector3D b) => Distance(this, b);

    public double SqrDistance(Vector3D b) => SqrDistance(this, b);
    public void Deconstruct(out double x, out double y, out double z)
        => (x, y, z) = (X, Y, Z);

    public double this[int k]
    {
        get
        {
            return k switch
            {
                0 => X,
                1 => Y,
                2 => Z,
                _ => throw new Exception("get: Vector3D out of range"),
            };
        }
    }
    public ReadOnlySpan<double> AsSpan() => MemoryMarshal.Cast<Vector3D, double>(MemoryMarshal.CreateReadOnlySpan(in this, 1));

    public Vector2D As2D() => new(X, Y);
    public Vector3D As3D() => this;

    public double Norm => Math.Sqrt(NormSqr);
    public double NormSqr => X * X + Y * Y + Z * Z;

    public double MaxNorm => Math.Max(Math.Abs(X), Math.Max(Math.Abs(Y), Math.Abs(Z)));

    public Vector3D Projection(Vector3D p) => (this * p) * p;

    public Vector3D Normalize() => this / Norm;

    public Vector3D Round(int digits) => new Vector3D(Math.Round(X, digits), Math.Round(Y, digits), Math.Round(Z, digits));

    public override string ToString() => $"Vec({X}, {Y}, {Z})";

    public override bool Equals(object? obj) => obj is Vector3D v && Equals(v);

    public override int GetHashCode() => HashCode.Combine(X, Y, Z);

    public bool Equals(Vector3D a) => a.X == X && a.Y == Y && a.Z == Z;

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
    public static Vector3D Vec(double x, double y, double z) => new Vector3D(x, y, z);

    public static Vector3D Parse(string line)
    {
        Vector3D res;
        if (!TryParse(line, out res))
            throw new FormatException($"Can't parse Vector3D from {line}");
        return res;
    }

    #region Static operators

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3D operator -(Vector3D a) => new Vector3D(-a.X, -a.Y, -a.Z);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3D operator +(Vector3D a) => a;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double operator *(Vector3D a, Vector3D b) => a.X * b.X + a.Y * b.Y + a.Z * b.Z;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3D operator *(double a, Vector3D b) => new Vector3D(a * b.X, a * b.Y, a * b.Z);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3D operator *(Vector3D b, double a) => new Vector3D(a * b.X, a * b.Y, a * b.Z);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3D operator /(Vector3D a, double v) => new Vector3D(a.X / v, a.Y / v, a.Z / v);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3D operator +(Vector3D a, Vector3D b) => new Vector3D(a.X + b.X, a.Y + b.Y, a.Z + b.Z);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3D operator -(Vector3D a, Vector3D b) => new Vector3D(a.X - b.X, a.Y - b.Y, a.Z - b.Z);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(Vector3D a, Vector3D b) => a.X == b.X && a.Y == b.Y && a.Z == b.Z;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(Vector3D a, Vector3D b) => a.X != b.X || a.Y != b.Y || a.Z != b.Z;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3D Cross(Vector3D v1, Vector3D v2) =>
        new Vector3D(v1.Y * v2.Z - v2.Y * v1.Z, v1.Z * v2.X - v1.X * v2.Z, v1.X * v2.Y - v1.Y * v2.X);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double Mixed(Vector3D v1, Vector3D v2, Vector3D v3) =>
        (v1.Y * v2.Z - v2.Y * v1.Z) * v3.X + (v1.Z * v2.X - v1.X * v2.Z) * v3.Y + (v1.X * v2.Y - v1.Y * v2.X) * v3.Z;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3D Sum(Vector3D a, Vector3D b) => a + b;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3D Min(Vector3D a, Vector3D b) =>
        new Vector3D(Math.Min(a.X, b.X), Math.Min(a.Y, b.Y), Math.Min(a.Z, b.Z));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3D Max(Vector3D a, Vector3D b) =>
        new Vector3D(Math.Max(a.X, b.X), Math.Max(a.Y, b.Y), Math.Max(a.Z, b.Z));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double Distance(Vector3D a, Vector3D b) => (a - b).Norm;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double SqrDistance(Vector3D a, Vector3D b)
    {
        var diff = a - b;
        return diff * diff;
    }
    #endregion

    #region EqualityComparer

    private class EqualityComparer : IEqualityComparer<Vector3D>
    {
        public int Digits { get; set; }

        public bool Equals(Vector3D v1, Vector3D v2)
        {
            return v1.Round(Digits) == v2.Round(Digits);
        }

        public int GetHashCode(Vector3D obj)
        {
            return obj.Round(Digits).GetHashCode();
        }
    }

    public static IEqualityComparer<Vector3D> CreateComparer(int digits = 7)
    {
        return new EqualityComparer { Digits = digits };
    }
    #endregion

    public static Vector3D Abs(Vector3D value) => throw new NotSupportedException();
    public static bool IsCanonical(Vector3D value) => true;
    public static bool IsComplexNumber(Vector3D value) => false;
    public static bool IsEvenInteger(Vector3D value) => false;
    public static bool IsFinite(Vector3D value) => double.IsFinite(value.X) && double.IsFinite(value.Y) && double.IsFinite(value.Z);
    public static bool IsImaginaryNumber(Vector3D value) => false;
    public static bool IsInfinity(Vector3D value) => double.IsInfinity(value.X) || double.IsInfinity(value.Y) || double.IsInfinity(value.Z);
    public static bool IsInteger(Vector3D value) => false;
    public static bool IsNaN(Vector3D value) => double.IsNaN(value.X) || double.IsNaN(value.Y) || double.IsNaN(value.Z);
    public static bool IsNegative(Vector3D value) => false;
    public static bool IsNegativeInfinity(Vector3D value) => false;
    public static bool IsNormal(Vector3D value) => value.Norm == 1.0;
    public static bool IsOddInteger(Vector3D value) => false;
    public static bool IsPositive(Vector3D value) => false;
    public static bool IsPositiveInfinity(Vector3D value) => false;
    public static bool IsRealNumber(Vector3D value) => false;
    public static bool IsSubnormal(Vector3D value) => false;
    public static bool IsZero(Vector3D value) => value == default;
    public static Vector3D MaxMagnitude(Vector3D x, Vector3D y) => MaxMagnitudeNumber(x, y);
    public static Vector3D MaxMagnitudeNumber(Vector3D x, Vector3D y) => x.X > y.X || (x.X == y.X && x.Y > y.Y) || (x.X == y.X && x.Y == y.Y && x.Z > y.Z) ? x : y;
    public static Vector3D MinMagnitude(Vector3D x, Vector3D y) => MinMagnitudeNumber(x, y);
    public static Vector3D MinMagnitudeNumber(Vector3D x, Vector3D y) => x.X < y.X || (x.X == y.X && x.Y < y.Y) || (x.X == y.X && x.Y == y.Y && x.Z < y.Z) ? x : y;
    public static Vector3D Parse(ReadOnlySpan<char> s, NumberStyles style, IFormatProvider? provider) => Parse(s.ToString(), style, provider);
    public static Vector3D Parse(string s, NumberStyles style, IFormatProvider? provider) => Parse(s);
    public static bool TryParse(ReadOnlySpan<char> s, NumberStyles style, IFormatProvider? provider, [MaybeNullWhen(false)] out Vector3D result) => TryParse(s.ToString(), style, provider, out result);
    public static bool TryParse([NotNullWhen(true)] string? s, NumberStyles style, IFormatProvider? provider, [MaybeNullWhen(false)] out Vector3D result) => TryParse(s!, out result);

    public static Vector3D One => throw new NotSupportedException();
    public static int Radix => throw new NotSupportedException();

    public bool TryFormat(Span<char> destination, out int charsWritten, ReadOnlySpan<char> format, IFormatProvider? provider)
    {
        var s = ToString();
        charsWritten = s.Length;
        return s.AsSpan().TryCopyTo(destination);
    }
    public string ToString(string? format, IFormatProvider? formatProvider) => ToString();
    public static Vector3D Parse(ReadOnlySpan<char> s, IFormatProvider? provider) => Parse(s.ToString());
    public static bool TryParse(ReadOnlySpan<char> s, IFormatProvider? provider, [MaybeNullWhen(false)] out Vector3D result) => TryParse(s.ToString(), out result);
    public static Vector3D Parse(string s, IFormatProvider? provider) => Parse(s);
    public static bool TryParse([NotNullWhen(true)] string? s, IFormatProvider? provider, [MaybeNullWhen(false)] out Vector3D result) => TryParse(s!, out result);
    public static Vector3D AdditiveIdentity => Zero;
    public static Vector3D operator --(Vector3D value) => throw new NotSupportedException();
    public static Vector3D operator /(Vector3D left, Vector3D right) => throw new NotSupportedException();
    public static Vector3D operator ++(Vector3D value) => throw new NotSupportedException();

    public static Vector3D MultiplicativeIdentity => throw new NotSupportedException();

    static Vector3D IMultiplyOperators<Vector3D, Vector3D, Vector3D>.operator *(Vector3D left, Vector3D right) => throw new NotSupportedException();

    static bool INumberBase<Vector3D>.TryConvertFromChecked<TOther>(TOther value, out Vector3D result)
    {
        if (value is Vector3D other)
        {
            result = other;
            return true;
        }
        result = default;
        return false;
    }
    static bool INumberBase<Vector3D>.TryConvertFromTruncating<TOther>(TOther value, out Vector3D result)
    {
        if (value is Vector3D other)
        {
            result = other;
            return true;
        }
        result = Vector3D.Zero;
        return false;

    }
    static bool INumberBase<Vector3D>.TryConvertToChecked<TOther>(Vector3D value, out TOther result)
    {
        if (typeof(TOther) == typeof(Vector3D))
        {
            result = (TOther)(object)value;
            return true;
        }
        result = TOther.Zero;
        return false;
    }
    static bool INumberBase<Vector3D>.TryConvertToTruncating<TOther>(Vector3D value, out TOther result)
    {
        if (typeof(TOther) == typeof(Vector3D))
        {
            result = (TOther)(object)value;
            return true;
        }
        result = TOther.Zero;
        return false;

    }
    public static bool TryConvertFromSaturating<TOther>(TOther value, [MaybeNullWhen(false)] out Vector3D result) where TOther : INumberBase<TOther>
    {
        switch (value)
        {
            case Vector3D v3d:
                result = v3d;
                return true;
            case Vector2D v2d:
                result = v2d.As3D();
                return true;
            case double v1d:
                result = new(v1d, 0, 0);
                return true;
            default:
                result = default;
                return false;
        }
    }
    public static bool TryConvertToSaturating<TOther>(Vector3D value, [MaybeNullWhen(false)] out TOther result) where TOther : INumberBase<TOther>
    {
        switch (TOther.Zero)
        {
            case Vector3D:
                result = (TOther)(object)value;
                return true;
            case Vector2D:
                result = (TOther)(object)value.As2D();
                return true;
            case double:
                result = (TOther)(object)value.X;
                return true;
            default:
                result = TOther.Zero;
                return false;
        }
    }

}
public readonly struct ComplexVector3D : IEquatable<ComplexVector3D>
{
    public static readonly ComplexVector3D Zero = new ComplexVector3D(0, 0, 0);

    public Complex X { get; }
    public Complex Y { get; }
    public Complex Z { get; }

    public Vector3D Real => new Vector3D(X.Real, Y.Real, Z.Real);
    public Vector3D Imaginary => new Vector3D(X.Imaginary, Y.Imaginary, Z.Imaginary);


    public ComplexVector3D(Complex x, Complex y, Complex z)
        => (X, Y, Z) = (x, y, z);

    public static implicit operator ComplexVector3D(in Vector3D v) => new ComplexVector3D(v.X, v.Y, v.Z);

    public override string ToString() => $"({X}, {Y}, {Z})";

    public override bool Equals(object? obj) => obj is ComplexVector3D v && Equals(v);

    public override int GetHashCode() => HashCode.Combine(X, Y, Z);

    public bool Equals(ComplexVector3D a) => a.X == X && a.Y == Y && a.Z == Z;

    #region Static operators

    public static ComplexVector3D operator -(in ComplexVector3D a) => new ComplexVector3D(-a.X, -a.Y, -a.Z);

    public static ComplexVector3D operator +(in ComplexVector3D a) => a;

    public static ComplexVector3D operator *(Complex a, in ComplexVector3D b) => new ComplexVector3D(a * b.X, a * b.Y, a * b.Z);

    public static ComplexVector3D operator *(in ComplexVector3D b, Complex a) => new ComplexVector3D(a * b.X, a * b.Y, a * b.Z);

    public static ComplexVector3D operator /(in ComplexVector3D a, Complex v) => new ComplexVector3D(a.X / v, a.Y / v, a.Z / v);

    public static ComplexVector3D operator +(in ComplexVector3D a, in ComplexVector3D b) => new ComplexVector3D(a.X + b.X, a.Y + b.Y, a.Z + b.Z);

    public static ComplexVector3D operator -(in ComplexVector3D a, in ComplexVector3D b) => new ComplexVector3D(a.X - b.X, a.Y - b.Y, a.Z - b.Z);

    public static Complex operator *(in ComplexVector3D a, in ComplexVector3D b)
        => a.X * Complex.Conjugate(b.X) + a.Y * Complex.Conjugate(b.Y) + a.Z * Complex.Conjugate(b.Z);

    public static ComplexVector3D Cross(Vector3D v1, Vector3D v2) =>
        new ComplexVector3D(v1.Y * v2.Z - v2.Y * v1.Z, v1.Z * v2.X - v1.X * v2.Z, v1.X * v2.Y - v1.Y * v2.X);

    public static ComplexVector3D Cross(ComplexVector3D v1, Vector3D v2) =>
        new ComplexVector3D(v1.Y * v2.Z - v2.Y * v1.Z, v1.Z * v2.X - v1.X * v2.Z, v1.X * v2.Y - v1.Y * v2.X);

    public static ComplexVector3D Cross(Vector3D v1, ComplexVector3D v2) =>
        new ComplexVector3D(v1.Y * v2.Z - v2.Y * v1.Z, v1.Z * v2.X - v1.X * v2.Z, v1.X * v2.Y - v1.Y * v2.X);

    public static ComplexVector3D Cross(ComplexVector3D v1, ComplexVector3D v2) =>
        new ComplexVector3D(v1.Y * v2.Z - v2.Y * v1.Z, v1.Z * v2.X - v1.X * v2.Z, v1.X * v2.Y - v1.Y * v2.X);


    #endregion
}
public static class VectorExtensions
{
    public static Vector3D Sum(this IEnumerable<Vector3D> vectors)
    {
        try
        {
            return vectors.Aggregate(Vector3D.Sum);
        }
        catch (InvalidOperationException)
        {
            return default;
        }
    }
    public static Vector3D WeightedEnumerableSum(this IEnumerable<Vector3D> vectors, IEnumerable<double> weights)
    {
        return vectors.Zip(weights, (a, b) => a * b).Aggregate(Vector3D.Sum);
    }
    public static Vector3D WeightedSpanSum(this ReadOnlySpan<Vector3D> vectors, ReadOnlySpan<double> weights)
    {
        var v = Vector3D.Zero;
        for (int i = 0; i < weights.Length; i++)
            v += vectors[i] * weights[i];
        return v;
    }
    public static Vector3D WeightedSpanSum(this Span<Vector3D> vectors, ReadOnlySpan<double> weights)
    {
        var v = Vector3D.Zero;
        for (int i = 0; i < weights.Length; i++)
            v += vectors[i] * weights[i];
        return v;
    }
    public static Vector3D CenterMass(this IEnumerable<Vector3D> vectors)
    {
        return vectors.Aggregate(Vector3D.Sum) / vectors.Count();
    }
    public static Vector2D Center(this IEnumerable<Vector2D> vectors)
    {
        return (vectors.Aggregate(Vector2D.Sum) / vectors.Count());
    }
    public static Vector3D CenterBox(this IEnumerable<Vector3D> vectors)
    {
        var min = vectors.Aggregate(vectors.First(), Vector3D.Min);
        var max = vectors.Aggregate(vectors.First(), Vector3D.Max);
        return (min + max) / 2;
    }

    public static Vector3D Center(this IEnumerable<Vector3D> vectors) => vectors.CenterMass();
    public static double Radius(this IEnumerable<Vector3D> vectors)
    {
        var c = vectors.Center();
        return Math.Sqrt(vectors.Max(v => (v - c).NormSqr));
    }

    public static Func<Vector3D, int> SplitBox(this IEnumerable<Vector3D> vectors)
    {
        var min = vectors.Aggregate(Vector3D.Min);
        var max = vectors.Aggregate(Vector3D.Max);
        var center = (min + max) / 2;
        double diameter = Vector3D.Axes.Max(a => (max - min) * a);
        var v = Vector3D.Axes.Where(a => (max - min) * a > diameter / 2).ToArray();
        return p => v.Select((x, i) => (p - center) * x >= 0 ? 1 << i : 0).Sum();
    }
}
