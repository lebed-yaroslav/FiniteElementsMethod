using System.Diagnostics;
using Model.Model.Basis;
using Telma;

namespace Model.Model.Elements.Segment;


public sealed class LagrangeCubicSegmentFactory : IBoundaryElementFactory2D
{
    public IBoundaryElement2D CreateBoundary(IMesh2D mesh, int[] vertices, int boundaryIndex)
    {
        var basis = (MutableBasisSet<Vector1D>)DefaultBasis();
        return new BoundaryElement2D(
            Geometry: new SegmentGeometry<Vector2D>(vertices) { Mesh = mesh },
            DOF: new Dof(basis),
            BasisSet: basis,
            BoundaryIndex: boundaryIndex);
    }


    public static IBasisSet1D DefaultBasis() => new MutableBasisSet<Vector1D>(
        Quadratures.SegmentGaussOrder7,
        SegmentBasis.Lagrange1D.Create(3, 0),
        SegmentBasis.Lagrange1D.Create(3, 3),
        SegmentBasis.Lagrange1D.Create(3, 1),
        SegmentBasis.Lagrange1D.Create(3, 2)
    );

    public sealed class Dof(MutableBasisSet<Vector1D> mutableBasis) : DofManager(dofCount: 4)
    {
        private readonly MutableBasisSet<Vector1D> _mutableBasisSet = mutableBasis;
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
            Debug.Assert(0 == localEdgeIndex);
            _dof[n + 2] = dofIndex;

            if (isOrientationFlipped)
            {
                _mutableBasisSet.MutableBasis[2] = SegmentBasis.Lagrange1D.Create(3, 2);
                _mutableBasisSet.MutableBasis[3] = SegmentBasis.Lagrange1D.Create(3, 1);
            }
        }

        public override void SetElementDof(int n, int dofIndex) =>
            throw new NotSupportedException();
    }
}
