using Model.Model.Basis;
using Telma;

namespace Model.Model.Elements.Triangle;

public sealed class LagrangeCubicTriangleFactory : IFiniteElementFactory
{
    public IFiniteElement Create(IMesh2D mesh, int[] vertices, int materialIndex)
    {
        var basis = (MutableBasisSet)DefaultBasis();
        return new FiniteElement(
            Geometry: new TriangleGeometry(vertices) { Mesh = mesh },
            DOF: new Dof(basis),
            BasisSet: basis,
            MaterialIndex: materialIndex
        );
    }

    // базис должен быть изменяемым как и в иерархическом (функции нужно менять местами)
    public static IBasisSet DefaultBasis() => new MutableBasisSet(
        Quadratures.TriangleOrder18,
        TriangleBasis.Lagrange.L3L3L3,
        TriangleBasis.Lagrange.L1L1L1,
        TriangleBasis.Lagrange.L2L2L2,
        TriangleBasis.Lagrange.L3L1L3,
        TriangleBasis.Lagrange.L3L1L1,
        TriangleBasis.Lagrange.L1L2L1,
        TriangleBasis.Lagrange.L1L2L2,
        TriangleBasis.Lagrange.L2L3L2,
        TriangleBasis.Lagrange.L2L3L3,
        TriangleBasis.Lagrange.L1L2L3
    );

    public sealed class Dof(MutableBasisSet mutableBasis) : DofManager(dofCount: 10)
    {
        private readonly MutableBasisSet _mutableBasisSet = mutableBasis;

        public override int NumberOfDofOnVertex => 1;
        public override int NumberOfDofOnEdge => 2;
        public override int NumberOfDofOnElement => 1;

        public override void SetVertexDof(int localVertexIndex, int n, int dofIndex)
        {
            if (n != 0) throw new NotSupportedException();
            if (localVertexIndex >= 3) throw new NotSupportedException();
            _dof[localVertexIndex] = dofIndex;
        }

        // TODO: Добавил изменяемость базису (ты можешь изменить его в Dof через _mutableBasisSet.MutableBasis[i] = TriangleBasis.Lagrange.LXLYLZ)
        public override void SetEdgeDof(int localEdgeIndex, bool isOrientationFlipped, int n, int dofIndex)
        {
            if (n != 0) throw new NotSupportedException();
            if (localEdgeIndex >= 6) throw new NotSupportedException();
            _dof[3 + localEdgeIndex] = dofIndex;
        }

        public override void SetElementDof(int n, int dofIndex)
        {
            _dof[9] = dofIndex;
        }  
    }
}
