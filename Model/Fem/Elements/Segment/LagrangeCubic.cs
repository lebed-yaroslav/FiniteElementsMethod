using System.Diagnostics;
using Model.Fem.Basis;
using Telma;

namespace Model.Fem.Elements.Segment;


public sealed class LagrangeCubicSegmentFactory : IBoundaryElementFactory2D
{
    public IBoundaryElement2D CreateBoundary(IMesh2D mesh, int[] vertices, int boundaryIndex)
    {
        return new BoundaryElement2D(
            Geometry: new SegmentGeometry<Vector2D>(vertices) { Mesh = mesh },
            DOF: new Dof(),
            BasisSet: Basis,
            BoundaryIndex: boundaryIndex);
    }

    public static IBasisSet1D Basis => new BasisSet1D(
        Quadratures.SegmentGaussOrder7,
        SegmentBasis.Lagrange1D.Create(3, 0),
        SegmentBasis.Lagrange1D.Create(3, 3),
        SegmentBasis.Lagrange1D.Create(3, 1),
        SegmentBasis.Lagrange1D.Create(3, 2)
    );

    public sealed class Dof() : ElementDof(dofCount: 4)
    {
        public override int NumberOfDofOnVertex => 1;
        public override int NumberOfDofOnEdge => 2;
        public override int NumberOfDofOnElement => 0;

        public override void SetVertexDof(int localVertexIndex, int n, int dofIndex)
        {
            AssertIsValidVertexDofNumber(n);
            Debug.Assert(0 <= localVertexIndex && localVertexIndex < 2);
            _dof[localVertexIndex] = dofIndex;
        }

        public override void SetEdgeDof(int localEdgeIndex, bool isOrientationFlipped, int n, int dofIndex)
        {
            AssertIsValidEdgeDofNumber(n);
            Debug.Assert(localEdgeIndex == 0);
            if (isOrientationFlipped) n = 1 - n;
            _dof[2 + n] = dofIndex;
        }

        public override void SetElementDof(int n, int dofIndex) =>
            throw new NotSupportedException();
    }
}
