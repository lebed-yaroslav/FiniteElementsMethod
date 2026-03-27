using Model.Core.CoordinateSystem;
using Model.Core.Matrix;
using Model.Model.Elements;
using Telma;
using Telma.Extensions;

namespace Model.Model;


[Obsolete("This feature is moving into Integrator folder")]
public static class NumericIntegrator<TSpace, TBoundary>
    where TSpace : IVectorBase<TSpace>
    where TBoundary : IVectorBase<TBoundary>
{
    public static LocalMatrix CalculateLocalStiffness(
        IFiniteElement2D element,
        Func<Vector2D, double> lambda
    )
    {
        int n = element.DOF.Count;
        var stiffness = new LocalMatrix(n);

        var masterCs = element.MasterElementCoordinateSystem;
        var meshCs = element.Mesh.CoordinateSystem;

        var masterInvJ = masterCs.InverseJacoby();
        var meshInvJ = meshCs.InverseJacoby();

        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j <= i; j++)
            {
                var value = 0.0;
                foreach (var q in element.Quadratures)
                {
                    var ep = q.Point; // master element-space point
                    var mp = masterCs.InverseTransform(ep); // mesh-space point
                    var invJ = masterInvJ.MulAt(ep, meshInvJ, mp); // Jacoby from master to physical
                    var gradPhiI = element.Basis[i].Derivatives(ep);
                    var gradPhiJ = element.Basis[j].Derivatives(ep);

                    // product = grad(phi_i)^T * J^(-T) * J(-1) * grad(phi_j)
                    var product = (
                        (invJ[0, 0] * invJ[0, 0] + invJ[1, 0] * invJ[1, 0]) * gradPhiI.X +
                        (invJ[0, 0] * invJ[0, 1] + invJ[1, 0] * invJ[1, 1]) * gradPhiI.Y
                    ) * gradPhiJ.X + (
                        (invJ[0, 1] * invJ[0, 1] + invJ[1, 1] * invJ[1, 1]) * gradPhiI.Y +
                        (invJ[0, 0] * invJ[0, 1] + invJ[1, 0] * invJ[1, 1]) * gradPhiI.X
                    ) * gradPhiJ.Y;

                    var jacobian = Math.Abs(1 / invJ.Det(Vector2D.Zero));

                    value += lambda(mp) * product * jacobian * q.Weight;
                }
                stiffness[i, j] = value;
                stiffness[j, i] = value;
            }
        }
        return stiffness;
    }

    public static LocalMatrix CalculateLocalMass(
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

    public static void CalculateLocalLoad(
        IFiniteElement2D element,
        Func<Vector2D, double> source,
        Span<double> load
    )
    {
        int n = element.DOF.Count;
        var masterCs = element.Geometry.MasterElementCoordinateSystem;
        var meshCs = element.Mesh.CoordinateSystem;

        for (int j = 0; j < n; j++)
        {
            var value = 0.0;
            foreach (var q in element.Quadratures)
            {
                var ep = q.Point; // master element-space point
                var mp = masterCs.InverseTransform(q.Point); // mesh-space point
                var psiJ = element.Basis[j].Value(ep);
                var jacobian = Math.Abs(masterCs.Jacobian(ep) * meshCs.Jacobian(mp));
                value += source(mp) * psiJ * q.Weight * jacobian;
            }
            load[j] = value;
        }
    }
}
