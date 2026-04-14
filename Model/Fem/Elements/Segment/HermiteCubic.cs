using System.Diagnostics;
using Model.Core.CoordinateSystem;
using Model.Fem.Basis;
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
    public IBoundaryElement2D CreateBoundary(IMesh2D mesh, int[] vertices, int boundaryIndex)
{
    var geom = new SegmentGeometry<Vector2D>(vertices) { Mesh = mesh };
    var refBasis = new IBasisFunction1D[] {
        new SegmentBasis.Hermite1D(0), new SegmentBasis.Hermite1D(1),
        new SegmentBasis.Hermite1D(2), new SegmentBasis.Hermite1D(3)
    };

    var transform = (SegmentParametrization<Vector2D>)geom.MasterElementCoordinateSystem;
    
    // Это вектор нашего отрезка (его длина и направление)
    double dx = transform.Offset.X; 
    double dy = transform.Offset.Y;

    var physicalBasis = new IBasisFunction1D[8]; 

    for (int k = 0; k < 2; k++)
    {
        int baseIdx = k * 4;
        int refValIdx = k * 2;     
        int refDerIdx = k * 2 + 1; 

        // n=0: u (значение не зависит от угла)
        var cU = new double[4]; cU[refValIdx] = 1.0;
        physicalBasis[baseIdx + 0] = new PhysicalHermiteBasis1D(refBasis, cU);

        // n=1: ux. Вклад в локальный наклон пропорционален dx
        var cUx = new double[4]; cUx[refDerIdx] = dx; 
        physicalBasis[baseIdx + 1] = new PhysicalHermiteBasis1D(refBasis, cUx);

        // n=2: uy. Вклад в локальный наклон пропорционален dy
        var cUy = new double[4]; cUy[refDerIdx] = dy; 
        physicalBasis[baseIdx + 2] = new PhysicalHermiteBasis1D(refBasis, cUy);

        // n=3: uxy. Смешанная производная на 1D линии 
        var cUxy = new double[4]; 
        cUxy[refDerIdx] = 0.0; 
        physicalBasis[baseIdx + 3] = new PhysicalHermiteBasis1D(refBasis, cUxy);
    }

        return new BoundaryElement2D(
            Geometry: geom,
            DOF: new Dof(),
            BasisSet: new BasisSet<Vector1D>(() => Quadratures.SegmentGaussOrder7(), physicalBasis),
            BoundaryIndex: boundaryIndex
        );
    }

    public static readonly IBasisSet1D Basis = new BasisSet1D(
        Quadratures.SegmentGaussOrder7,
        new SegmentBasis.Hermite1D(0), // Значение в узле 0
        new SegmentBasis.Hermite1D(1), // Производная в узле 0
        new SegmentBasis.Hermite1D(2), // Значение в узле 1
        new SegmentBasis.Hermite1D(3)  // Производная в узле 1

        /*new SegmentBasis.Hermite1D(0), // Значение в узле 0
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
