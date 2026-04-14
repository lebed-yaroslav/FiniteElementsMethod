using System.Diagnostics;
using Model.Fem.Basis;
using Telma;

namespace Model.Fem.Elements.Segment;


public sealed class LagrangeQuadraticSegmentFactory : IBoundaryElementFactory2D
{
    public IBoundaryElement2D CreateBoundary(IMesh2D mesh, int[] vertices, int boundaryIndex)
        => new BoundaryElement2D(
            Geometry: new SegmentGeometry<Vector2D>(vertices) { Mesh = mesh },
            DOF: new Dof(),
            BasisSet: Basis,
            BoundaryIndex: boundaryIndex
    );

    public static readonly IBasisSet1D Basis = new BasisSet1D(
        Quadratures.SegmentGaussOrder5,
        SegmentBasis.Lagrange1D.Create(2, 0),
        SegmentBasis.Lagrange1D.Create(2, 2),
        SegmentBasis.Lagrange1D.Create(2, 1)
    );

    public sealed class Dof() : DofManager(dofCount: 3)
    {
        //|-----|-----|
        public override int NumberOfDofOnVertex => 1;
        public override int NumberOfDofOnEdge => 1;
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
            _dof[localEdgeIndex + 2] = dofIndex;
        }

        public override void SetElementDof(int n, int dofIndex) =>
            throw new NotSupportedException();
    }
}
