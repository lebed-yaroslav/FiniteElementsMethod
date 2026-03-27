using Model.Core.Matrix;
using Model.Model.Elements;
using Telma.Extensions;

namespace Model.Model.Integrator;

public class NumericIntegrator<TSpace, TBoundary> : IIntegrator<TSpace, TBoundary>
    where TSpace : IVectorBase<TSpace>
    where TBoundary : IVectorBase<TBoundary>
{
    public static readonly NumericIntegrator<TSpace, TBoundary> Instance = new();
    private NumericIntegrator() { }

    public LocalMatrix CalculateLocalMass(
        IFiniteElementBase<TSpace, TBoundary> element,
        Func<TSpace, double> gamma
    )
    {
        int n = element.DOF.Count;
        var mass = new LocalMatrix(n);

        var masterCs = element.MasterElementCoordinateSystem;
        var meshCs = element.Mesh.CoordinateSystem;

        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j <= i; j++)
            {
                var value = 0.0;
                foreach (var q in element.Quadratures)
                {
                    var ep = q.Point; // master element-space point
                    var mp = masterCs.InverseTransform(ep); // mesh-space point
                    var phiI = element.Basis[i].Value(ep);
                    var phiJ = element.Basis[j].Value(ep);
                    var jacobian = Math.Abs(masterCs.Jacobian(ep) * meshCs.Jacobian(mp)); // FIXME: may be optimized (by IsConstant)
                    value += gamma(mp) * phiI * phiJ * jacobian * q.Weight;
                }
                mass[i, j] = value;
                mass[j, i] = value;
            }
        }
        return mass;
    }

    public void CalculateLocalLoad(
        IFiniteElementBase<TSpace, TBoundary> element,
        Func<TSpace, double> source,
        Span<double> load
    )
    {
        int n = element.DOF.Count;
        var masterCs = element.MasterElementCoordinateSystem;
        var meshCs = element.Mesh.CoordinateSystem;

        for (int j = 0; j < n; j++)
        {
            var value = 0.0;
            foreach (var q in element.Quadratures)
            {
                var ep = q.Point; // master element-space point
                var mp = masterCs.InverseTransform(q.Point); // mesh-space point
                var psiJ = element.Basis[j].Value(ep);
                var jacobian = Math.Abs(masterCs.Jacobian(ep) * meshCs.Jacobian(mp));  // FIXME: may be optimized (by IsConstant)
                value += source(mp) * psiJ * q.Weight * jacobian;
            }
            load[j] = value;
        }
    }
}
