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
    IEquatable<TSelf>
    where TSelf : IVectorBase<TSelf>
{
    static abstract int Dimensions { get; }
    static abstract TSelf Zero { get; }

    double Norm { get; }
    double NormSqr { get; }
    double MaxNorm { get; }
    TSelf Round(int digits);

    Vector1D As1D();
    Vector2D As2D();
    Vector3D As3D();

    static abstract double operator *(TSelf a, TSelf b);
}

public static class VectorExtensions
{
    extension<TVector>(TVector self) where TVector : IVectorBase<TVector>
    {
        public double Distance(TVector b) => (b - self).Norm;
        public double DistanceSqr(TVector b) => (b - self).NormSqr;
        public TVector Normalize() => self / self.Norm;

        public static IEqualityComparer<TVector> CreateComparer(int digits) => new EqualityComparer<TVector>(digits);
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
}
