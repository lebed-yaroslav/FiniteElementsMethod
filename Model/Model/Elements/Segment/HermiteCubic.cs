using System.Diagnostics;
using Model.Core.CoordinateSystem;
using Model.Model.Basis;
using Telma;

namespace Model.Model.Elements.Segment;

public sealed class PhysicalHermiteBasis1D(
    IBasisFunction1D[] refBasis,
    double[] coefficients
) : IBasisFunction1D
{
    public double Value(Vector1D p)
    {
        double sum = 0;
        for (int i = 0; i < 4; i++)
            if (coefficients[i] != 0) sum += coefficients[i] * refBasis[i].Value(p);
        return sum;
    }

    public Vector1D Derivatives(Vector1D p)
    {
        double sum = 0;
        for (int i = 0; i < 4; i++)
            if (coefficients[i] != 0) sum += refBasis[i].Derivatives(p).X * coefficients[i];
        return sum;
    }
}
public sealed class HermiteSegmentFactory : IBoundaryElementFactory2D
{
    public IBoundaryElement2D CreateBoundary(IMesh2D mesh, int[] vertices, int boundaryIndex)
    {
        var geom = new SegmentGeometry<Vector2D>(vertices) { Mesh = mesh };
        var refBasis = new IBasisFunction1D[] {
            new SegmentBasis.Hermite1D(0), new SegmentBasis.Hermite1D(1),
            new SegmentBasis.Hermite1D(2), new SegmentBasis.Hermite1D(3)
        };

        var transform = (SegmentParametrization<Vector2D>)geom.MasterElementCoordinateSystem;
        double dx_dxi = transform.Offset.X;
        double dy_dxi = transform.Offset.Y;

        var physicalBasis = new IBasisFunction1D[8]; // 2 узла * 4 DOF

        for (int k = 0; k < 2; k++)
        {
            // n=0: физическое u
            var cU = new double[4]; cU[k * 2 + 0] = 1.0;
            physicalBasis[k * 4 + 0] = new PhysicalHermiteBasis1D(refBasis, cU);

            // n=1: физическое ux (влияет на локальную производную u_xi через dx/dxi)
            var cUx = new double[4]; cUx[k * 2 + 1] = dx_dxi;
            physicalBasis[k * 4 + 1] = new PhysicalHermiteBasis1D(refBasis, cUx);

            // n=2: физическое uy (влияет на локальную производную u_xi через dy/dxi)
            var cUy = new double[4]; cUy[k * 2 + 1] = dy_dxi;
            physicalBasis[k * 4 + 2] = new PhysicalHermiteBasis1D(refBasis, cUy);

            // n=3: физическое uxy (фиксируем в 0 для 1D элемента)
            var cUxy = new double[4];
            physicalBasis[k * 4 + 3] = new PhysicalHermiteBasis1D(refBasis, cUxy);
        }

        return new BoundaryElement2D(
            Geometry: geom,
            DOF: new Dof(),
            BasisSet: new BasisSet<Vector1D>(() => Quadratures.SegmentGaussOrder5(), physicalBasis),
            BoundaryIndex: boundaryIndex
        );
    }

    public static readonly IBasisSet1D Basis = new BasisSet1D(
        Quadratures.SegmentGaussOrder5,
        new SegmentBasis.Hermite1D(0), // Значение в узле 0
        new SegmentBasis.Hermite1D(1), // Производная в узле 0
        new SegmentBasis.Hermite1D(2), // Значение в узле 1
        new SegmentBasis.Hermite1D(3)  // Производная в узле 1

       /* new SegmentBasis.Hermite1D(0), // Значение в узле 0
        new SegmentBasis.Hermite1D(1), // Производная в узле 0
        new SegmentBasis.Hermite1D(2), // Значение в узле 1
        new SegmentBasis.Hermite1D(3)  // Производная в узле 1*/
    );

    public sealed class Dof() : DofManager(dofCount: 8) // 2 узла * 4 DOF
    {
        public override int NumberOfDofOnVertex => 4;
        public override int NumberOfDofOnEdge => 0;
        public override int NumberOfDofOnElement => 0;

        public override void SetVertexDof(int localVertexIndex, int n, int dofIndex)
        {
            AssertIsValidVertexDofNumber(n);
            Debug.Assert(0 <= localVertexIndex && localVertexIndex < 2);
            // n=0: u, n=1: ux, n=2: uy, n=3: uxy
            _dof[localVertexIndex * 4 + n] = dofIndex;
        }

        public override void SetEdgeDof(int localEdgeIndex, bool isOrientationFlipped, int n, int dofIndex) =>
            throw new NotSupportedException();

        public override void SetElementDof(int n, int dofIndex) =>
            throw new NotSupportedException();
    }
}
