using System.Runtime.CompilerServices;
using Telma;
using Telma.Extensions;

namespace Model.Core.CoordinateSystem;


public static class JacobyMatrixExtensions
{
    extension<TSource, TTarget>(IJacobyMatrix<TSource, TTarget> self)
        where TSource : IVectorBase<TSource>
        where TTarget : IVectorBase<TTarget>
    {
        public ConstantJacobyMatrix<TSource, TTarget> ComputeAt(TSource sourcePoint)
        {
            var matrix = new double[TSource.Dimensions, TTarget.Dimensions];
            for (var i = 0; i < TSource.Dimensions; ++i)
                for (var j = 0; j < TTarget.Dimensions; ++j)
                    matrix[i, j] = self[i, j, sourcePoint];
            return new(matrix);
        }
    }
}

public static class JacobyMatrixMatrixMultiplication
{
    [OverloadResolutionPriority(0)]
    public static ConstantJacobyMatrix<TSource, TOther> MulAt<TSource, TTarget, TOther>(
    this IJacobyMatrix<TSource, TTarget> lhs,
    TSource lp,
    IJacobyMatrix<TTarget, TOther> rhs,
    TTarget rp
)
    where TSource : IVectorBase<TSource>
    where TTarget : IVectorBase<TTarget>
    where TOther : IVectorBase<TOther>
    {

        var res = new double[TSource.Dimensions, TOther.Dimensions];
        for (var i = 0; i < TSource.Dimensions; ++i)
            for (var k = 0; k < TTarget.Dimensions; ++k)
                for (var j = 0; j < TOther.Dimensions; ++j)
                    res[i, j] += lhs[i, k, lp] * rhs[k, j, rp];
        return new(res);
    }

    #region Specialization

    [OverloadResolutionPriority(1)]
    public static ConstantJacobyMatrix1x1 MulAt(
        this IJacobyMatrix1x1 lhs,
        Vector1D lp,
        IJacobyMatrix1x1 rhs,
        Vector1D rp
    )
        => new(new[,] { { lhs.Det(lp) * rhs.Det(lp) } });


    [OverloadResolutionPriority(1)]
    public static ConstantJacobyMatrix2x2 MulAt(
        this IJacobyMatrix2x2 lhs,
        Vector2D lp,
        IJacobyMatrix2x2 rhs,
        Vector2D rp
    ) => new(new[,] {
        {
            lhs[0, 0, lp] * rhs[0, 0, rp] + lhs[0, 1, lp] * rhs[1, 0, rp],
            lhs[0, 0, lp] * rhs[0, 1, rp] + lhs[0, 1, lp] * rhs[1, 1, rp]
        },
        {
            lhs[1, 0, lp] * rhs[0, 0, rp] + lhs[1, 1, lp] * rhs[1, 0, rp],
            lhs[1, 0, lp] * rhs[0, 1, rp] + lhs[1, 1, lp] * rhs[1, 1, rp]
        }
    });

    #endregion
}

public static class ConstantJacobyMatrixVectorMultiplication
{
    [OverloadResolutionPriority(0)]
    public static TTarget Mul<TSource, TTarget>(
        this TSource lhs,
        ConstantJacobyMatrix<TSource, TTarget> rhs
    )
        where TSource : IVectorBase<TSource>
        where TTarget : IVectorBase<TTarget>
    => throw new NotImplementedException(
        $"No specialization found for Mul(Vector{TSource.Dimensions}D, " +
        $"ConstantJacobyMatrix{TSource.Dimensions}x{TTarget.Dimensions})"
    );

    #region Specialization

    [OverloadResolutionPriority(2)]
    public static Vector1D Mul(this Vector1D lhs, ConstantJacobyMatrix1x1 rhs)
        => new(lhs.X * rhs[0, 0]);

    [OverloadResolutionPriority(1)]
    public static Vector2D Mul(this Vector1D lhs, ConstantJacobyMatrix1x2 rhs)
        => new(lhs.X * rhs[0, 0], lhs.X * rhs[0, 1]);

    [OverloadResolutionPriority(2)]
    public static Vector2D Mul(this Vector2D lhs, ConstantJacobyMatrix2x2 rhs)
        => new(
            lhs.X * rhs[0, 0] + lhs.Y * rhs[1, 0],
            lhs.X * rhs[0, 1] + lhs.Y * rhs[1, 1]
        );

    [OverloadResolutionPriority(1)]
    public static Vector3D Mul(this Vector2D lhs, ConstantJacobyMatrix2x3 rhs)
        => new(
            lhs.X * rhs[0, 0] + lhs.Y * rhs[1, 0],
            lhs.X * rhs[0, 1] + lhs.Y * rhs[1, 1],
            lhs.X * rhs[0, 2] + lhs.Y * rhs[1, 2]
        );

    [OverloadResolutionPriority(2)]
    public static Vector3D Mul(this Vector3D lhs, ConstantJacobyMatrix3x3 rhs)
    => new(
        lhs.X * rhs[0, 0] + lhs.Y * rhs[1, 0] + lhs.Z * rhs[2, 0],
        lhs.X * rhs[0, 1] + lhs.Y * rhs[1, 1] + lhs.Z * rhs[2, 1],
        lhs.X * rhs[0, 2] + lhs.Y * rhs[1, 2] + lhs.Z * rhs[2, 2]
    );

    #endregion

    [OverloadResolutionPriority(0)]
    public static TSource Mul<TSource, TTarget>(
        this ConstantJacobyMatrix<TSource, TTarget> rhs,
        TTarget lhs
    )
        where TSource : IVectorBase<TSource>
        where TTarget : IVectorBase<TTarget>
    => throw new NotImplementedException(
        $"No specialization found for Mul(ConstantJacobyMatrix{TSource.Dimensions}x{TTarget.Dimensions}), " +
        $"Vector{TTarget.Dimensions}D)"
    );

    #region Specialization

    [OverloadResolutionPriority(2)]
    public static Vector1D Mul(this ConstantJacobyMatrix1x1 rhs, Vector1D lhs)
        => lhs.Mul(rhs);

    [OverloadResolutionPriority(1)]
    public static Vector1D Mul(this ConstantJacobyMatrix1x2 rhs, Vector2D lhs)
        => new(rhs[0, 0] * lhs.X + rhs[0, 1] * lhs.Y);

    [OverloadResolutionPriority(2)]
    public static Vector2D Mul(this ConstantJacobyMatrix2x2 rhs, Vector2D lhs)
        => new(
            rhs[0, 0] * lhs.X + rhs[0, 1] * lhs.Y,
            rhs[1, 0] * lhs.X + rhs[1, 1] * lhs.Y
        );

    [OverloadResolutionPriority(1)]
    public static Vector2D Mul(this ConstantJacobyMatrix2x3 rhs, Vector3D lhs)
    => new(
        rhs[0, 0] * lhs.X + rhs[0, 1] * lhs.Y + rhs[0, 2] * lhs.Z,
        rhs[1, 0] * lhs.X + rhs[1, 1] * lhs.Y + rhs[1, 2] * lhs.Z
    );

    [OverloadResolutionPriority(2)]
    public static Vector3D Mul(this ConstantJacobyMatrix3x3 rhs, Vector3D lhs)
    => new(
        rhs[0, 0] * lhs.X + rhs[0, 1] * lhs.Y + rhs[0, 2] * lhs.Z,
        rhs[1, 0] * lhs.X + rhs[1, 1] * lhs.Y + rhs[1, 2] * lhs.Z,
        rhs[2, 0] * lhs.X + rhs[2, 1] * lhs.Y + rhs[2, 2] * lhs.Z
    );

    #endregion
}


public static class ConstantJacobyMatrixDeterminant
{
    [OverloadResolutionPriority(0)]
    public static double Det<TSource, TTarget>(
        this ConstantJacobyMatrix<TSource, TTarget> self
    )
        where TSource : IVectorBase<TSource>
        where TTarget : IVectorBase<TTarget>
    => throw new NotImplementedException(
        $"No specialization found for Det(ConstantJacobyMatrix{TSource.Dimensions}x{TTarget.Dimensions}))"
    );

    #region Specialization

    [OverloadResolutionPriority(1)]
    public static double Det(this ConstantJacobyMatrix1x1 self) => self[0, 0];

    [OverloadResolutionPriority(1)]
    public static double Det(this ConstantJacobyMatrix2x2 self)
        => self[0, 0] * self[1, 1] - self[0, 1] * self[1, 0];

    [OverloadResolutionPriority(1)]
    public static double Det(this ConstantJacobyMatrix3x3 self)
        => self[0, 0] * (self[1, 1] * self[2, 2] - self[1, 2] * self[2, 1])
        - self[0, 1] * (self[1, 0] * self[2, 2] - self[1, 2] * self[2, 0])
        + self[0, 2] * (self[1, 0] * self[2, 1] - self[1, 1] * self[2, 0]);

    #endregion
}
