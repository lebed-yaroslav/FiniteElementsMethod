using System.Diagnostics;
using Model.Core.CoordinateSystem;
using Model.Fem.Basis;
using Model.Fem.Elements;
using Model.Fem.Elements.Segment;
using Telma;

namespace Model.Fem.Elements.Segment;

public sealed class PhysicalHermiteBasis1D(
    IBasisFunction1D[] refBasis,
    double[] coefficients
) : IBasisFunction1D
{
    public double Value(Vector1D p)
    {
        double sum = 0;
        for (int i = 0; i < 4; i++)
            sum += coefficients[i] * refBasis[i].Value(p);
        return sum;
    }

    public Vector1D Derivatives(Vector1D p)
    {
        double sum = 0;
        for (int i = 0; i < 4; i++)
            sum += coefficients[i] * refBasis[i].Derivatives(p).X;
        return new Vector1D(sum);
    }
}
public sealed class HermiteSegmentFactory : IBoundaryElementFactory2D
{
    private const double Eps = 1e-15;

    public enum BoundaryMode
    {
        Horizontal, //  u + ux
        Vertical,   //  u + uy
        Oblique     //  только u (косое ребро, где угол не 90)
    }

    public IBoundaryElement2D CreateBoundary(IMesh2D mesh, int[] vertices, int boundaryIndex)
    {
        var geom = new SegmentGeometry<Vector2D>(vertices) { Mesh = mesh };
        var transform = (SegmentParametrization<Vector2D>)geom.MasterElementCoordinateSystem;
        var v = transform.Offset;

        var mode = DetectMode(v);

        var refBasis = new IBasisFunction1D[]
        {
            new SegmentBasis.Hermite1D(0), new SegmentBasis.Hermite1D(1),
            new SegmentBasis.Hermite1D(2), new SegmentBasis.Hermite1D(3)
        };

        IBasisFunction1D[] physicalBasis = mode switch
        {
            BoundaryMode.Horizontal => CreateHorizontalBasis(refBasis, v.X),
            BoundaryMode.Vertical => CreateVerticalBasis(refBasis, v.Y),
            BoundaryMode.Oblique => CreateHorizontalBasis(refBasis, v.X ) ,
            _ => throw new UnreachableException()
        };

        return new BoundaryElement2D(
            Geometry: geom,
            DOF: new Dof(mode),
            BasisSet: new BasisSet<Vector1D>(() => Quadratures.SegmentGaussOrder7(), physicalBasis),
            BoundaryIndex: boundaryIndex
        );
    }

    private static BoundaryMode DetectMode(Vector2D v)
    {
        bool isHorizontal = Math.Abs(v.Y) < Eps;
        bool isVertical = Math.Abs(v.X) < Eps;

        if (isHorizontal) return BoundaryMode.Horizontal;
        if (isVertical) return BoundaryMode.Vertical;
        return BoundaryMode.Oblique;
    }

    private static IBasisFunction1D[] CreateHorizontalBasis(IBasisFunction1D[] H, double dx)
    {
        var basis = new IBasisFunction1D[4];

        // vertex 0: u
        var u0 = new double[4];
        u0[0] = 1.0;
        basis[0] = new PhysicalHermiteBasis1D(H, u0);

        // vertex 0: ux
        var ux0 = new double[4];
        ux0[1] = dx;
        basis[1] = new PhysicalHermiteBasis1D(H, ux0);

        // vertex 1: u
        var u1 = new double[4];
        u1[2] = 1.0;
        basis[2] = new PhysicalHermiteBasis1D(H, u1);

        // vertex 1: ux
        var ux1 = new double[4];
        ux1[3] = dx;
        basis[3] = new PhysicalHermiteBasis1D(H, ux1);

        return basis;
    }

    private static IBasisFunction1D[] CreateVerticalBasis(IBasisFunction1D[] H, double dy)
    {
        var basis = new IBasisFunction1D[4];

        // vertex 0: u
        var u0 = new double[4];
        u0[0] = 1.0;
        basis[0] = new PhysicalHermiteBasis1D(H, u0);

        // vertex 0: uy
        var uy0 = new double[4];
        uy0[1] = dy;
        basis[1] = new PhysicalHermiteBasis1D(H, uy0);

        // vertex 1: u
        var u1 = new double[4];
        u1[2] = 1.0;
        basis[2] = new PhysicalHermiteBasis1D(H, u1);

        // vertex 1: uy
        var c3 = new double[4];
        c3[3] = dy;
        basis[3] = new PhysicalHermiteBasis1D(H, c3);

        return basis;
    }

    public sealed class Dof(BoundaryMode mode) : DofManager(dofCount: 4)
    {
        private readonly BoundaryMode _mode = mode;

        public override int NumberOfDofOnVertex => 2;
        public override int NumberOfDofOnEdge => 0;
        public override int NumberOfDofOnElement => 0;

        public override void SetVertexDof(int localVertexIndex, int n, int dofIndex)
        {
            Debug.Assert(0 <= localVertexIndex && localVertexIndex < 2);
            Debug.Assert(n == 0 || n == 1);

            int pos = localVertexIndex * 2 + n;

            if (n == 0)
            {
                // u
                _dof[pos] = dofIndex;
                return;
            }

            // 0:u, 1:ux, 2:uy, 3:uxy
            // Для горизонтали ux => base+1.
            // Для вертикали   uy => base+2.
            _dof[pos] = _mode == BoundaryMode.Vertical ? dofIndex + 1 : dofIndex;
        }

        public override void SetEdgeDof(int localEdgeIndex, bool isOrientationFlipped, int n, int dofIndex)
            => throw new NotSupportedException();

        public override void SetElementDof(int n, int dofIndex)
            => throw new NotSupportedException();
    }
}
