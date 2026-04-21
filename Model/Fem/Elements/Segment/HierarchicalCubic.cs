using System.Diagnostics;
using Model.Fem.Basis;
using Telma;

namespace Model.Fem.Elements.Segment;


public sealed class HierarchicalCubicSegmentFactory : IBoundaryElementFactory2D
{
    public IBoundaryElement2D CreateBoundary(IMesh2D mesh, int[] vertices, int boundaryIndex)
    {
        var basis = DefaultBasis();
        return new BoundaryElement2D(
            Geometry: new SegmentGeometry<Vector2D>(vertices) { Mesh = mesh },
            DOF: new Dof(basis),
            BasisSet: basis,
            BoundaryIndex: boundaryIndex
        );
    }

    public static IBasisSet1D DefaultBasis() => new BasisSet1D(
        Quadratures.SegmentGaussOrder9,
        SegmentBasis.N0,
        SegmentBasis.N1,
        SegmentBasis.N0N1,
        new OrientedBasisFunction1D(SegmentBasis.N0N1N0SubN1)
    );

    public sealed class Dof(IBasisSet1D basis) : ElementDof(dofCount: 4)
    {
        private readonly IBasisSet1D _basis = basis;
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
            int basisIndex = 2 + n;
            _dof[basisIndex] = dofIndex;
            if (n == 1)
                ((OrientedBasisFunction1D)_basis.Basis[basisIndex]).IsOrientationFlipped = isOrientationFlipped;
        }

        public override void SetElementDof(int n, int dofIndex)
            => throw new NotSupportedException();
    }
}
