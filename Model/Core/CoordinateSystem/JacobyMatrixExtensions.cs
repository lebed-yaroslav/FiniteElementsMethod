using Telma.Extensions;

namespace Model.Core.CoordinateSystem;


public static class JacobyMatrixMatrixMultiplication
{
    public static ConstantJacobyMatrix<TSource, TOther, TOps> MulAt<TSource, TTarget, TOther, TOps>(
        this IJacobyMatrix<TSource, TTarget> lhs,
        TSource lp,
        IJacobyMatrix<TTarget, TOther> rhs,
        TTarget rp
    )
        where TSource : IVectorBase<TSource>
        where TTarget : IVectorBase<TTarget>
        where TOther : IVectorBase<TOther>
        where TOps : IMatrixOperations<TSource, TOther, TOps>
    {
        var res = new double[TSource.Dimensions, TOther.Dimensions];
        for (var i = 0; i < TSource.Dimensions; ++i)
            for (var k = 0; k < TTarget.Dimensions; ++k)
                for (var j = 0; j < TOther.Dimensions; ++j)
                    res[i, j] += lhs[i, k, lp] * rhs[k, j, rp];
        return new(res);
    }
}
