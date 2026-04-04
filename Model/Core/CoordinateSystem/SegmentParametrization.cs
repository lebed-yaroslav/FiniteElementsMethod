using System.Diagnostics;
using Telma;
using Telma.Extensions;

namespace Model.Core.CoordinateSystem;


/// <summary>
/// Parametrization: γ(t) = A + V * t  (t∈[-1; 1])
/// </summary>
/// <typeparam name="TSource">Source vector dimensions</typeparam>
/// <param name="a">Start point of the segment (t = -1)</param>
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
    {
        var projection = ((sourcePoint - Origin) * Offset) / Offset.NormSqr;
        return 2 * projection - 1;
    }

    public TSource InverseTransform(Vector1D targetPoint)
    {
        double t = 0.5 * (targetPoint + 1);
        return Origin + Offset * t;
    }

    public double Jacobian(Vector1D targetPoint) => 2.0 / Offset.Norm;

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
            return Offset.AsSpan()[j] / 2.0;
        }
    }
    public double this[int i, int j, Vector1D _] => this[i, j];
    public double Det(Vector1D _) => Offset.Norm / 2.0;
}
