using Model.Model.Basis;
using Telma;

namespace Model.Model.Elements.Triangle;

public sealed class HierarchicalCubicTriangleFactory : IFiniteElementFactory
{
    public IFiniteElement Create(IMesh2D mesh, int[] vertices) =>
        new FiniteElement(
            Geometry: new TriangleGeometry(vertices) { Mesh = mesh },
            DOF: new Dof(),
            BasisSet: Basis
     );

    public static readonly IBasisSet Basis = new MutableBasisSet(
        Quadratures.TriangleOrder9,
        TriangleBasis.L1,
        TriangleBasis.L2,
        TriangleBasis.L3,
        TriangleBasis.L1L2,
        TriangleBasis.L2L3,
        TriangleBasis.L3L1,
        TriangleBasis.L3L1L3DivL1,
        TriangleBasis.L1L2L1DivL2,
        TriangleBasis.L3L2L3DivL2
    );

    public sealed class Dof() : DofManager(dofCount: 10)
    {
        public override int NumberOfDofOnVertex => 1;
        public override int NumberOfDofOnEdge => 2;
        public override int NumberOfDofOnElement => 1;

        // нужно знать в каком направлении нам дают ребро, чтобы изменить базисную функцию
        public override void SetEdgeDof(int localEdgeIndex, int n, int dofIndex)
        {
            throw new NotSupportedException();
        }


        public override void SetVertexDof(int localVertexIndex, int n, int dofIndex)
        {
            if (n != 0) throw new NotSupportedException();
            if (localVertexIndex >= 3) throw new NotSupportedException();
            _dof[localVertexIndex] = dofIndex;
        }

        public override void SetElementDof(int n, int dofIndex)
        {
            if (n != 0) throw new NotSupportedException();
            _dof[9] = dofIndex;
        }
    }
}
