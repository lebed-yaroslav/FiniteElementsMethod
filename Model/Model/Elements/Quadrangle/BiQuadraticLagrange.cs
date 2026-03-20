using Model.Model.Basis;
using Model.Model.Elements;
using Model.Model.Elements.Quadrangle;
using Model.Model.Mesh;
using Telma;

internal class BiQuadraticLagrangeQuadrangleFactory : IFiniteElementFactory<Vector2D>
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
        QuadrangleBasis.Q2_Lagrange
    );

    public sealed class Dof() : DofManager(dofCount: 9)
    {
        public override int NumberOfDofOnVertex => 1;
        public override int NumberOfDofOnEdge => 1;
        public override int NumberOfDofOnElement => 1;

        public override void SetVertexDof(int localVertexIndex, int n, int dofIndex)
        {
            if (n != 0) throw new NotSupportedException();
            if (localVertexIndex >= 4) throw new NotSupportedException();
            var basisIndex = localVertexIndex switch
            {
                0 => 0,
                1 => 2,
                2 => 6,
                3 => 8,
                _ => throw new NotSupportedException()
            };
            _dof[basisIndex] = dofIndex;
        }

        public override void SetEdgeDof(int localEdgeIndex, bool isOrientationFlipped, int n, int dofIndex)
        {
            if (n != 0) throw new NotSupportedException();
            if (localEdgeIndex >= 4) throw new NotSupportedException();
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
            if (n != 0) throw new NotSupportedException();
            _dof[4] = dofIndex;
        }
    }
}
