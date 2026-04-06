using System.Diagnostics;
using Model.Core.CoordinateSystem;
using Model.Model.Basis;
using Telma;

namespace Model.Model.Elements.Quadrangle;

public sealed class PhysicalHermiteBasis2D(
    IBasisFunction2D[] refBasis, // Все 16 функций элемента
    double[] coefficients       // Столбец матрицы трансформации T
) : IBasisFunction2D
{
    public double Value(Vector2D p)
    {
        double sum = 0;
        for (int i = 0; i < 16; i++)
            if (coefficients[i] != 0) sum += coefficients[i] * refBasis[i].Value(p);
        return sum;
    }

    public Vector2D Derivatives(Vector2D p)
    {
        Vector2D sum = Vector2D.Zero;
        for (int i = 0; i < 16; i++)
            if (coefficients[i] != 0) sum += refBasis[i].Derivatives(p) * coefficients[i];
        return sum;
    }
}
public sealed class HermiteQuadrangleFactory : IFiniteElementFactory2D
{
    public IFiniteElement2D CreateElement(IMesh2D mesh, int[] vertices, int materialIndex)
    {
        var geometry = new QuadrangleGeometry(vertices) { Mesh = mesh };
        var refBasis = QuadrangleBasis.Q3_Hermite;

        var transform = (QuadrangleCoordinateSystem)geometry.MasterElementCoordinateSystem;
        var jMatrix = transform.J_Matrix;

        // Смешанная производная отображения x_xi_eta, y_xi_eta
        var d2P = jMatrix.MixedDerivative;

        var physicalBasis = new IBasisFunction2D[16];
        Vector2D[] corners = [new(0, 0), new(1, 0), new(1, 1), new(0, 1)];

        for (int k = 0; k < 4; k++)
        {
            var J = jMatrix.At(corners[k]);
            double x_xi = J[0, 0], x_eta = J[0, 1];
            double y_xi = J[1, 0], y_eta = J[1, 1];

            // 1. Физическая функция для u
            var cU = new double[16]; cU[k * 4 + 0] = 1.0;
            physicalBasis[k * 4 + 0] = new PhysicalHermiteBasis2D(refBasis, cU);

            // 2. Физическая функция для ux 
            var cUx = new double[16];
            cUx[k * 4 + 1] = x_xi;
            cUx[k * 4 + 2] = x_eta;
            cUx[k * 4 + 3] = d2P.X;
            physicalBasis[k * 4 + 1] = new PhysicalHermiteBasis2D(refBasis, cUx);

            // 3. Физическая функция для uy
            var cUy = new double[16];
            cUy[k * 4 + 1] = y_xi;
            cUy[k * 4 + 2] = y_eta;
            cUy[k * 4 + 3] = d2P.Y;
            physicalBasis[k * 4 + 2] = new PhysicalHermiteBasis2D(refBasis, cUy);

            // 4. Физическая функция для uxy
            var cUxy = new double[16];
            cUxy[k * 4 + 3] = (x_xi * y_eta + x_eta * y_xi);
            physicalBasis[k * 4 + 3] = new PhysicalHermiteBasis2D(refBasis, cUxy);
        }

        return new FiniteElement2D(
            Geometry: geometry,
            DOF: new Dof(),
            BasisSet: new BasisSet<Vector2D>(Quadratures.QuadrangleOrder9, physicalBasis),
            MaterialIndex: materialIndex
        );
    }

    public static readonly IBasisSet2D Basis = new BasisSet2D(
        Quadratures.QuadrangleOrder9,
        QuadrangleBasis.Q3_Hermite
    );

    public sealed class Dof() : DofManager(dofCount: 16)
    {
        public override int NumberOfDofOnVertex => 4;
        public override int NumberOfDofOnEdge => 0;
        public override int NumberOfDofOnElement => 0;

        public override void SetVertexDof(int localVertexIndex, int n, int dofIndex)
        {
            AssertIsValidVertexDofNumber(n);
            Debug.Assert(0 <= localVertexIndex && localVertexIndex < 4);
            _dof[localVertexIndex * 4 + n] = dofIndex;
        }

        public override void SetEdgeDof(int localEdgeIndex, bool isOrientationFlipped, int n, int dofIndex)
            => throw new NotSupportedException();


        public override void SetElementDof(int n, int dofIndex)
            => throw new NotSupportedException();
    }
}
