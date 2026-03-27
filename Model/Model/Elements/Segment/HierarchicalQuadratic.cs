using System.Diagnostics;
using Model.Model.Basis;
using Model.Model.Mesh;
using Telma;

namespace Model.Model.Elements.Segment;


public sealed class HierarchicalQuadraticSegmentFactory : IBoundaryElementFactory2D
{
    public IBoundaryElement2D CreateBoundary(IMesh2D mesh, int[] vertices, int boundaryIndex)
        => new BoundaryElement2D(
            Geometry: new SegmentGeometry<Vector2D>.Boundary(vertices) { Mesh = mesh },
            DOF: new Dof(),
            BasisSet: Basis,
            BoundaryIndex: boundaryIndex
    );

    public static readonly IBasisSet1D Basis = new BasisSet1D(
        Quadratures.SegmentGaussOrder1,
        SegmentBasis.N0,
        SegmentBasis.N1,
        SegmentBasis.N0N1
   );

    public sealed class Dof() : DofManager(dofCount: 3)
    {
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
            AssertIsValidElementDofNumber(n);
            Debug.Assert(localEdgeIndex == 0);
            _dof[2] = dofIndex;
        }

        public override void SetElementDof(int n, int dofIndex) =>
            throw new NotSupportedException();
    }
}
