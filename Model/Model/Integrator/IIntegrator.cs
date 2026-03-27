using Model.Core.Matrix;
using Model.Model.Elements;
using Telma.Extensions;

namespace Model.Model.Integrator;

public interface IIntegrator<TSpace, TBoundary>
    where TSpace : IVectorBase<TSpace>
    where TBoundary : IVectorBase<TBoundary>
{
    LocalMatrix CalculateLocalMass(
        IFiniteElementBase<TSpace, TBoundary> element,
        Func<TSpace, double> gamma
    );

    public void CalculateLocalLoad(
        IFiniteElementBase<TSpace, TBoundary> element,
        Func<TSpace, double> source,
        Span<double> load
    );
}
