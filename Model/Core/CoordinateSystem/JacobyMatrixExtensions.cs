using Telma.Extensions;

namespace Model.Core.CoordinateSystem;


public static class JacobyMatrixMatrixMultiplication
{
    extension<TSource, TTarget>(IJacobyMatrix<TSource, TTarget> self)
        where TSource : IVectorBase<TSource>
        where TTarget : IVectorBase<TTarget>
    {
        public ConstantJacobyMatrix<TSource, TOther, TOps> MulAt<TOther, TOps>(
            TSource lp,
            IJacobyMatrix<TTarget, TOther> rhs,
            TTarget rp
        )
            where TOther : IVectorBase<TOther>
            where TOps : IMatrixOperations<TSource, TOther, TOps>
        {
            var res = new double[TSource.Dimensions, TOther.Dimensions];
            for (var i = 0; i < TSource.Dimensions; ++i)
                for (var k = 0; k < TTarget.Dimensions; ++k)
                    for (var j = 0; j < TOther.Dimensions; ++j)
                        res[i, j] += self[i, k, lp] * rhs[k, j, rp];
            return new(res);
        }
    }
}
