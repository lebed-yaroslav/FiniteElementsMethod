using Model.Core.CoordinateSystem;
using Model.Core.Matrix;
using Model.Model.Elements;
using Telma.Extensions;

namespace Model.Model.Integrator;

public interface IIntegrator<TSpace, TBoundary, TOps>
    where TSpace : IVectorBase<TSpace>
    where TBoundary : IVectorBase<TBoundary>
    where TOps : IMatrixOperations<TSpace, TSpace, TOps>
{
    LocalMatrix CalculateLocalStiffness(
        IFiniteElementBase<TSpace, TSpace> element,
        Func<TSpace, double> lambda
    );

    LocalMatrix CalculateLocalMass(
        IFiniteElementBase<TSpace, TBoundary> element,
        Func<TSpace, double> gamma
    );

    public void CalculateLocalLoad(
        IFiniteElementBase<TSpace, TBoundary> element,
        Func<TSpace, double> source,
        Span<double> load
    );

    public void CalculateLocalLoad(
        IFiniteElementBase<TSpace, TSpace> element,
        Func<TSpace, double> source,
        Span<double> load
    );
}
