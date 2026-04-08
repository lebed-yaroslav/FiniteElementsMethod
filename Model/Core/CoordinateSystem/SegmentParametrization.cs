using System.Diagnostics;
using Telma;
using Telma.Extensions;

namespace Model.Core.CoordinateSystem;


/// <summary>
/// Parametrization: γ(t) = A + V * t
/// </summary>
/// <typeparam name="TSource">Source vector dimensions</typeparam>
/// <param name="a">Start point of the segment (t = 0)</param>
/// <param name="b">End point of the segment (t = 1)</param>
public sealed class SegmentParametrization<TSource>(TSource a, TSource b) :
    ICoordinateTransform<TSource, Vector1D>
    where TSource : struct, IVectorBase<TSource>
{
    public static bool IsLinear => true;
    private readonly InverseSegmentJacobyMatrix<TSource> _j = new(b - a);
    public TSource Origin { get; } = a;
    public TSource Offset => _j.Offset;


    public Vector1D Transform(TSource sourcePoint)
        => ((sourcePoint - Origin) * Offset) / Offset.NormSqr;

    public TSource InverseTransform(Vector1D targetPoint)
        => Origin + Offset * targetPoint;

    public double Jacobian(Vector1D targetPoint) => Offset.Norm;

    public IJacobyMatrix<Vector1D, TSource> InverseJacoby()
        => _j;
}


public sealed record InverseSegmentJacobyMatrix<TSource>(
    TSource Offset
) : IJacobyMatrix<Vector1D, TSource>
    where TSource : struct, IVectorBase<TSource>
{
    public static bool IsConstant => true;
    public double this[int i, int j]
    {
        get
        {
            Debug.Assert(0 <= i && i < IJacobyMatrix<Vector1D, TSource>.Rows);
            Debug.Assert(0 <= j && j < IJacobyMatrix<Vector1D, TSource>.Columns);
            return Offset.AsSpan()[j];
        }
    }
    public double this[int i, int j, Vector1D _] => this[i, j];
    public double Det(Vector1D _) => 1.0 / Offset.Norm;
}
