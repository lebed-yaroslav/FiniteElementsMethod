using System.Diagnostics;
using Telma.Extensions;

namespace Model.Core.CoordinateSystem;

public sealed class IdentityTransform<TSource> : ICoordinateTransform<TSource, TSource>
    where TSource : IVectorBase<TSource>
{
    public static IdentityTransform<TSource> Instance { get; } = new();
    private IdentityTransform() {}

    public static bool IsLinear => true;

    public TSource Transform(TSource sourcePoint) => sourcePoint;
    public TSource InverseTransform(TSource targetPoint) => targetPoint;
    public double Jacobian(TSource targetPoint) => 1.0;
    public IJacobyMatrix<TSource, TSource> InverseJacoby()
        => IdentityJacobyMatrix<TSource>.Instance;
}

public sealed class IdentityJacobyMatrix<TSource> : IJacobyMatrix<TSource, TSource>
    where TSource : IVectorBase<TSource>
{
    public static IdentityJacobyMatrix<TSource> Instance { get; } = new();
    private IdentityJacobyMatrix() {}

    public static bool IsConstant => true;
    public double this[int i, int j]
    {
        get
        {
            Debug.Assert(0 <= i && i < TSource.Dimensions);
            Debug.Assert(0 <= j && j < TSource.Dimensions);
            return (i == j) ? 1 : 0;
        }
    }

    public double this[int i, int j, TSource _] => this[i, j];


    public double Det(TSource targetPoint) => 1.0;
}
