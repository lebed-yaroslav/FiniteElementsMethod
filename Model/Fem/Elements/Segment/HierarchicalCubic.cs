using System.Diagnostics;
using Model.Fem.Basis;
using Model.Fem.Integrator;
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

    public static MutableBasisSet<Vector1D> DefaultBasis() => new(
        Quadratures.SegmentGaussOrder5,
        SegmentBasis.N0,
        SegmentBasis.N1,
        SegmentBasis.N0N1,
        SegmentBasis.N0N1N0SubN1
    );

    public sealed class Dof(MutableBasisSet<Vector1D> basis) : ElementDof(dofCount: 4)
    {
        private readonly MutableBasisSet<Vector1D> _basis = basis;
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
            {
                _basis.MutableBasis[basisIndex] = isOrientationFlipped
                    ? Polynomial1D.ScalMult(SegmentBasis.N0N1N0SubN1, -1.0)
                    : SegmentBasis.N0N1N0SubN1;
            }
        }

        public override void SetElementDof(int n, int dofIndex)
            => throw new NotSupportedException();
    }
}
