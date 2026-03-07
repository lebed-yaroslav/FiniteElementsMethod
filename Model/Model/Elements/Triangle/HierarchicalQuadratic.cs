using Model.Model.Basis;
using Telma;

namespace Model.Model.Elements.Triangle;

public sealed class HierarchicalQuadraticTriangleFactory : IFiniteElementFactory
{
    public IFiniteElement Create(IMesh2D mesh, int[] vertices, int materialIndex) =>
        new FiniteElement(
            Geometry: new TriangleGeometry(vertices) { Mesh = mesh },
            DOF: new Dof(),
            BasisSet: Basis,
            MaterialIndex: materialIndex
     );

    public static readonly IBasisSet<Vector2D> Basis = new BasisSet<Vector2D>(
        Quadratures.TriangleOrder6,
        TriangleBasis.L3,
        TriangleBasis.L1,
        TriangleBasis.L2,
        TriangleBasis.L3L1,
        TriangleBasis.L1L2,
        TriangleBasis.L2L3
    );

    public sealed class Dof() : DofManager(dofCount: 6)
    {
        public override int NumberOfDofOnVertex => 1;
        public override int NumberOfDofOnEdge => 1;
        public override int NumberOfDofOnElement => 0;


        public override void SetEdgeDof(int localEdgeIndex, bool isOrientationFlipped, int n, int dofIndex)
        {
            if (n != 0) throw new NotSupportedException();
            if (localEdgeIndex >= 3) throw new NotSupportedException();
            _dof[3 + localEdgeIndex] = dofIndex;
        }


        public override void SetVertexDof(int localVertexIndex, int n, int dofIndex)
        {
            if (n != 0) throw new NotSupportedException();
            if (localVertexIndex >= 3) throw new NotSupportedException();
            _dof[localVertexIndex] = dofIndex;
        }

        public override void SetElementDof(int n, int dofIndex)
            => throw new NotSupportedException();
    }
}
