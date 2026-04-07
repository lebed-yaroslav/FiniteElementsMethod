using Model.Model.Basis;
using Telma;

namespace Model.Model.Elements.Quadrangle;


public sealed class BiQuadraticLagrangeQuadrangleFactory : IFiniteElementFactory2D
{
    public IFiniteElement2D CreateElement(IMesh2D mesh, int[] vertices, int materialIndex) =>
        new FiniteElement2D(
            Geometry: new QuadrangleGeometry(vertices) { Mesh = mesh },
            DOF: new Dof(),
            BasisSet: Basis,
            MaterialIndex: materialIndex
     );

    public static readonly IBasisSet2D Basis = new BasisSet2D(
        Quadratures.QuadrangleOrder7,
        QuadrangleBasis.Q2_Lagrange
    );

    public sealed class Dof() : DofManager(dofCount: 9)
    {
        public override int NumberOfDofOnVertex => 1;
        public override int NumberOfDofOnEdge => 1;
        public override int NumberOfDofOnElement => 1;

        public override void SetVertexDof(int localVertexIndex, int n, int dofIndex)
        {
            AssertIsValidVertexDofNumber(n);
            var basisIndex = localVertexIndex switch
            {
                0 => 0,
                1 => 2,
                2 => 8,
                3 => 6,
                _ => throw new NotSupportedException()
            };
            _dof[basisIndex] = dofIndex;
        }

        public override void SetEdgeDof(int localEdgeIndex, bool isOrientationFlipped, int n, int dofIndex)
        {
            AssertIsValidEdgeDofNumber(n);
            var basisIndex = localEdgeIndex switch
            {
                0 => 1,
                1 => 5,
                2 => 7,
                3 => 3,
                _ => throw new NotSupportedException()
            };
            _dof[basisIndex] = dofIndex;
        }

        public override void SetElementDof(int n, int dofIndex)
        {
            AssertIsValidElementDofNumber(n);
            _dof[4] = dofIndex;
        }
    }
}
