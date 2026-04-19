using Model.Fem.Basis;
using Telma;

namespace Model.Fem.Elements.Quadrangle;


public sealed class BicubicLagrangeQuadrangleFactory : IFiniteElementFactory2D
{
    public IFiniteElement2D CreateElement(IMesh2D mesh, int[] vertices, int materialIndex) =>
        new FiniteElement2D(
            Geometry: new QuadrangleGeometry(vertices) { Mesh = mesh },
            DOF: new Dof(),
            BasisSet: Basis,
            MaterialIndex: materialIndex
     );

    public static readonly IBasisSet2D Basis = new BasisSet2D(
        Quadratures.QuadrangleOrder9,
        QuadrangleBasis.Q3_Lagrange
    );

    public sealed class Dof() : ElementDof(dofCount: 16)
    {
        public override int NumberOfDofOnVertex => 1;
        public override int NumberOfDofOnEdge => 2;
        public override int NumberOfDofOnElement => 4;

        public override void SetVertexDof(int localVertexIndex, int n, int dofIndex)
        {
            AssertIsValidVertexDofNumber(n);
            var basisIndex = localVertexIndex switch
            {
                0 => 0,
                1 => 3,
                2 => 15,
                3 => 12,
                _ => throw new NotSupportedException()
            };
            _dof[basisIndex] = dofIndex;
        }

        // Each edge has two nodes that numbered in clockwise order (corresponds to QuadrangleGeometry edge numeration)
        public override void SetEdgeDof(int localEdgeIndex, bool isOrientationFlipped, int n, int dofIndex)
        {
            AssertIsValidEdgeDofNumber(n);
            if (isOrientationFlipped) n = 1 - n;
            var basisIndex = localEdgeIndex switch
            {
                0 => 1 + n,
                1 => 7 + n * 4,
                2 => 14 - n,
                3 => 8 - n * 4,
                _ => throw new NotSupportedException()
            };
            _dof[basisIndex] = dofIndex;
        }

        // Local numeration corresponds to vertex node numerations
        public override void SetElementDof(int n, int dofIndex)
        {
            AssertIsValidElementDofNumber(n);
            var basisIndex = n switch
            {
                0 => 5,
                1 => 6,
                2 => 10,
                3 => 9,
                _ => throw new NotSupportedException()
            };
            _dof[basisIndex] = dofIndex;
        }
    }
}
