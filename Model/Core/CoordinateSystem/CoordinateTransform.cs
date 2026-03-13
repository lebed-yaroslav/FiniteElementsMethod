using Telma.Extensions;

namespace Model.Core.CoordinateSystem;


public interface ICoordinateTransform<TSource, TTarget>
    where TSource : IVectorBase<TSource>
    where TTarget : IVectorBase<TTarget>
{
    static abstract bool IsLinear { get; }

    TTarget Transform(TSource sourcePoint);
    TSource InverseTransform(TTarget targetPoint);
    double Jacobian(TTarget targetPoint);
    IJacobyMatrix<TTarget, TSource> InverseJacoby();
}
