using Model.Model.Basis;
using Model.Model.Mesh;
using Telma;

namespace Model.Model.Elements.Segment;


public sealed class QuadraticSegmentFactory : IBoundaryElementFactory<Vector2D, Vector1D>
{
    public IBoundaryElement<Vector2D, Vector1D> CreateBoundary(IMesh<Vector2D> mesh, int[] vertices, int boundaryIndex)
        => new BoundaryElement<Vector2D, Vector1D>(
            Geometry: new SegmentGeometry<Vector2D>.Boundary(vertices) { Mesh = mesh },
            DOF: new Dof(),
            BasisSet: Basis,
            BoundaryIndex: boundaryIndex
    );

    public static readonly IBasisSet<Vector1D> Basis = new BasisSet<Vector1D>(
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
            if (n != 0) throw new NotSupportedException();
            if (localVertexIndex <= 2) throw new NotSupportedException();
            _dof[localVertexIndex] = dofIndex;
        }

        public override void SetEdgeDof(int localEdgeIndex, bool isOrientationFlipped, int n, int dofIndex)
        {
            if (n != 0) throw new NotSupportedException();
            if (localEdgeIndex != 0) throw new NotSupportedException();
            _dof[2] = dofIndex;
        }

        public override void SetElementDof(int n, int dofIndex) =>
            throw new NotSupportedException();
    }
}
