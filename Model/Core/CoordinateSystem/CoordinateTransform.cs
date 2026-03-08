using System.Diagnostics;
using Telma.Extensions;

namespace Model.Core.CoordinateSystem;

public interface ICoordinateTransform<TSource, TTarget>
    where TSource : IVectorBase<TSource>
    where TTarget : IVectorBase<TTarget>
{
    abstract bool IsLinear { get; }

    TTarget Transform(TSource sourcePoint);
    TSource InverseTransform(TTarget targetPoint);
    double Jacobian(TTarget targetPoint);
    IJacobyMatrix<TTarget, TSource> InverseJacoby();
}

public interface IJacobyMatrix<TSource, TTarget>
    where TSource : IVectorBase<TSource>
    where TTarget : IVectorBase<TTarget>
{
    static int Rows => TTarget.Dimensions;
    static int Columns => TSource.Dimensions;
    static abstract bool IsConstant { get; }

    double Det(TSource targetPoint);
    double this[int i, int j, TSource targetPoint] { get; }
    double this[int i, int j] { get; }
}

public static class IJacobyMatrixExtensions
{
    extension<TJacobyMatrix, TSource, TTarget>(TJacobyMatrix self)
        where TJacobyMatrix : IJacobyMatrix<TSource, TTarget>
        where TSource : IVectorBase<TSource>
        where TTarget: IVectorBase<TTarget>
    {
        public double Det()
        {
            Debug.Assert(TJacobyMatrix.IsConstant);
            return self.Det(TSource.Zero);
        }
    }
}
