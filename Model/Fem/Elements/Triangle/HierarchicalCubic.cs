using System.Diagnostics;
using Model.Fem.Basis;
using Model.Fem.Integrator;
using Telma;

namespace Model.Fem.Elements.Triangle;


public sealed class HierarchicalCubicTriangleFactory : IFiniteElementFactory2D
{
    public IFiniteElement2D CreateElement(IMesh2D mesh, int[] vertices, int materialIndex)
    {
        var basis = DefaultBasis();
        return new FiniteElement2D(
            Geometry: new TriangleGeometry(vertices) { Mesh = mesh },
            DOF: new Dof(basis),
            BasisSet: basis,
            MaterialIndex: materialIndex
        );
    }

    public static MutableBasisSet<Vector2D> DefaultBasis() => new(
        Quadratures.TriangleOrder6,
        TriangleBasis.L3,
        TriangleBasis.L1,
        TriangleBasis.L2,
        TriangleBasis.L3L1,
        TriangleBasis.L1L2,
        TriangleBasis.L2L3,
        TriangleBasis.L1L3L1SubL3,
        TriangleBasis.L1L2L2SubL1,
        TriangleBasis.L2L3L2SubL3,
        TriangleBasis.L1L2L3
    );

    public sealed class Dof(MutableBasisSet<Vector2D> basis) : ElementDof(dofCount: 10)
    {
        private readonly MutableBasisSet<Vector2D> _basis = basis;

        public override int NumberOfDofOnVertex => 1;
        public override int NumberOfDofOnEdge => 2;
        public override int NumberOfDofOnElement => 1;

        public override void SetVertexDof(int localVertexIndex, int n, int dofIndex)
        {
            AssertIsValidVertexDofNumber(n);
            Debug.Assert(0 <= localVertexIndex && localVertexIndex < 3);
            _dof[localVertexIndex] = dofIndex;
        }

        public override void SetEdgeDof(int localEdgeIndex, bool isOrientationFlipped, int n, int dofIndex)
        {
            AssertIsValidEdgeDofNumber(n);
            Debug.Assert(0 <= localEdgeIndex && localEdgeIndex < 3);

            int basisIndex = 3 + 3 * n + localEdgeIndex;
            _dof[basisIndex] = dofIndex;
            if (n == 1)
            {
                Polynomial2D basis_func = localEdgeIndex switch
                {
                    0 => TriangleBasis.L1L3L1SubL3,
                    1 => TriangleBasis.L1L2L2SubL1,
                    2 => TriangleBasis.L2L3L2SubL3,
                    _ => throw new UnreachableException()
                };
                _basis.MutableBasis[basisIndex] = isOrientationFlipped ? Polynomial2D.ScalMult(basis_func, -1.0) : basis_func;
            }
        }

        public override void SetElementDof(int n, int dofIndex)
        {
            AssertIsValidElementDofNumber(n);
            _dof[9] = dofIndex;
        }
    }
}
