using System.Numerics;
using System.Runtime.InteropServices;

namespace Telma.Extensions;

/// <summary>Defines common n-dimensional vector traits</summary>
/// <typeparam name="TSelf">
/// One of <see cref="Vector1D"/>, <see cref="Vector2D"/>, <see cref="Vector3D"/>
/// </typeparam>
public interface IVectorBase<TSelf> :
    IUnaryNegationOperators<TSelf, TSelf>,
    IAdditionOperators<TSelf, TSelf, TSelf>,
    ISubtractionOperators<TSelf, TSelf, TSelf>,
    IMultiplyOperators<TSelf, double, TSelf>,
    IDivisionOperators<TSelf, double, TSelf>,
    IEqualityOperators<TSelf, TSelf, bool>,
    IComparisonOperators<TSelf, TSelf, bool>,
    IEquatable<TSelf>
    where TSelf : IVectorBase<TSelf>
{
    static abstract int Dimensions { get; }
    static abstract TSelf Zero { get; }
    static abstract TSelf[] Axes { get; }

    static abstract TSelf FromSpan(ReadOnlySpan<double> span);

    double Norm { get; }
    double NormSqr { get; }
    double MaxNorm { get; }
    TSelf Round(int digits);

    Vector1D As1D();
    Vector2D As2D();
    Vector3D As3D();

    static abstract double operator *(TSelf a, TSelf b);
    static abstract TSelf Min(TSelf a, TSelf b);
    static abstract TSelf Max(TSelf a, TSelf b);
}

public static class VectorExtensions
{
    extension<TVector>(TVector self) where TVector : IVectorBase<TVector>
    {
        public double Distance(TVector b) => (b - self).Norm;
        public double DistanceSqr(TVector b) => (b - self).NormSqr;
        public TVector Normalize() => self / self.Norm;

        public static IEqualityComparer<TVector> CreateComparer(int digits) => new EqualityComparer<TVector>(digits);

        public bool AreNearlyEquals(TVector other, double epsilon = 1e-12)
            => (self - other).MaxNorm <= epsilon;
    }

    extension<TVector>(TVector self) where TVector : struct, IVectorBase<TVector>
    {
        public ReadOnlySpan<double> AsSpan() =>
            MemoryMarshal.Cast<TVector, double>(MemoryMarshal.CreateReadOnlySpan(in self, 1));
    }


    private class EqualityComparer<TVector>(int digits) : IEqualityComparer<TVector> where TVector : IVectorBase<TVector>
    {
        private readonly int _digits = digits;

#pragma warning disable CS8767 // Nullability of reference types in type of parameter doesn't match implicitly implemented member (possibly because of nullability attributes).
        public bool Equals(TVector a, TVector b) => a.Round(_digits) == b.Round(_digits);
#pragma warning restore CS8767 // Nullability of reference types in type of parameter doesn't match implicitly implemented member (possibly because of nullability attributes).
        public int GetHashCode(TVector obj) => obj.Round(_digits).GetHashCode();
    }

    #region Enumerables

    public static TVector Sum<TVector>(this IEnumerable<TVector> vectors)
        where TVector : IVectorBase<TVector>
        => vectors.Aggregate((a, b) => a + b);

    public static TVector WeightedSum<TVector>(this IEnumerable<TVector> vectors, IEnumerable<double> weights)
        where TVector : IVectorBase<TVector>
        => vectors.Zip(weights, (a, b) => a * b).Aggregate((a, b) => a + b);

    public static TVector WeightedSum<TVector>(this ReadOnlySpan<TVector> vectors, ReadOnlySpan<double> weights)
        where TVector : IVectorBase<TVector>
    {
        var v = TVector.Zero;
        for (int i = 0; i < weights.Length; i++)
            v += vectors[i] * weights[i];
        return v;
    }

    public static TVector WeightedSum<TVector>(this Span<TVector> vectors, ReadOnlySpan<double> weights)
        where TVector : IVectorBase<TVector>
    {
        var v = TVector.Zero;
        for (int i = 0; i < weights.Length; i++)
            v += vectors[i] * weights[i];
        return v;
    }

    public static TVector Center<TVector>(this IEnumerable<TVector> vectors)
        where TVector : IVectorBase<TVector>
        => vectors.Sum() / vectors.Count();

    public static TVector CenterBox<TVector>(this IEnumerable<TVector> vectors)
        where TVector : IVectorBase<TVector>
    {
        var min = vectors.Aggregate(vectors.First(), TVector.Min);
        var max = vectors.Aggregate(vectors.First(), TVector.Max);
        return (min + max) / 2;
    }

    public static double Radius<TVector>(this IEnumerable<TVector> vectors)
        where TVector : IVectorBase<TVector>
    {
        var c = vectors.Center();
        return Math.Sqrt(vectors.Max(v => (v - c).NormSqr));
    }

    public static Func<TVector, int> SplitBox<TVector>(this IEnumerable<TVector> vectors)
        where TVector : IVectorBase<TVector>
    {
        var min = vectors.Aggregate(TVector.Min);
        var max = vectors.Aggregate(TVector.Max);
        var center = (min + max) / 2;
        double diameter = TVector.Axes.Max(a => (max - min) * a);
        var v = TVector.Axes.Where(a => (max - min) * a > diameter / 2).ToArray();
        return p => v.Select((x, i) => (p - center) * x >= 0 ? 1 << i : 0).Sum();
    }
    #endregion
}
