using Model.Model.Basis;
using Telma;

namespace Model.Model.Elements.Triangle;

public sealed class HierarchicalCubicTriangleFactory : IFiniteElementFactory
{
    public IFiniteElement Create(IMesh2D mesh, int[] vertices, int materialIndex)
    {
        var basis = (BasisSet)Basis;
        return new FiniteElement(
          Geometry: new TriangleGeometry(vertices) { Mesh = mesh },
          DOF: new Dof(basis),
          BasisSet: basis,
          MaterialIndex: materialIndex
        );
    }

    public static IBasisSet Basis => new BasisSet(
      Quadratures.TriangleOrder6,
      TriangleBasis.L3,
      TriangleBasis.L1,
      TriangleBasis.L2,
      TriangleBasis.L3L1,
      TriangleBasis.L1L2,
      TriangleBasis.L2L3,
      new OrientedBasisFunction(TriangleBasis.L1L3L1SubL3),
      new OrientedBasisFunction(TriangleBasis.L1L2L2SubL1),
      new OrientedBasisFunction(TriangleBasis.L2L3L2SubL3),
      TriangleBasis.L1L2L3
    );

    public sealed class Dof(IBasisSet basis) : DofManager(dofCount: 10)
    {
        private IBasisSet _basis = basis;

        public override int NumberOfDofOnVertex => 1;
        public override int NumberOfDofOnEdge => 2;
        public override int NumberOfDofOnElement => 1;

        public override void SetEdgeDof(int localEdgeIndex, bool isOrientationFlipped, int n, int dofIndex)
        {
            _dof[3 + 3 * n + localEdgeIndex] = dofIndex;

            if (n == 1 && isOrientationFlipped)
            {
                switch (localEdgeIndex)
                {
                    case 0:
                        ((OrientedBasisFunction)_basis.Basis[6]).IsOrientationFlipped = true;
                        break;
                    case 1:
                        ((OrientedBasisFunction)_basis.Basis[7]).IsOrientationFlipped = true;
                        break;
                    case 2:
                        ((OrientedBasisFunction)_basis.Basis[8]).IsOrientationFlipped = true;
                        break;
                    default:
                        break;
                }
            }
        }

        public override void SetVertexDof(int localVertexIndex, int n, int dofIndex)
        {
            _dof[localVertexIndex] = dofIndex;
        }

        public override void SetElementDof(int n, int dofIndex)
        {
            _dof[9] = dofIndex;
        }
    }
}
