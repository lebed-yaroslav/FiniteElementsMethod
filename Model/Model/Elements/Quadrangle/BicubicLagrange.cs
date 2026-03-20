using Model.Model.Basis;
using Model.Model.Mesh;
using Telma;

namespace Model.Model.Elements.Quadrangle;


public sealed class BicubicLagrangeQuadrangleFactory : IFiniteElementFactory<Vector2D>
{
    public IFiniteElement<Vector2D> CreateElement(IMesh<Vector2D> mesh, int[] vertices, int materialIndex) =>
        new FiniteElement<Vector2D>(
            Geometry: new QuadrangleGeometry(vertices) { Mesh = mesh },
            DOF: new Dof(),
            BasisSet: Basis,
            MaterialIndex: materialIndex
     );

    public static readonly IBasisSet<Vector2D> Basis = new BasisSet<Vector2D>(
        Quadratures.QuadrangleOrder7,
        QuadrangleBasis.Q3_Lagrange
    );

    public sealed class Dof() : DofManager(dofCount: 16)
    {
        public override int NumberOfDofOnVertex => 1;
        public override int NumberOfDofOnEdge => 2;
        public override int NumberOfDofOnElement => 4;


        public override void SetVertexDof(int localVertexIndex, int n, int dofIndex)
        {
            if (n != 0) throw new NotSupportedException();
            var basisIndex = localVertexIndex switch {
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
            if (n < 0 || n >= 2) throw new NotSupportedException();
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
            var basisIndex = n switch
            {
                0 => 5,
                1 => 6,
                2 => 9,
                3 => 10,
                _ => throw new NotSupportedException()
            };
            _dof[basisIndex] = dofIndex;
        }
    }
}
