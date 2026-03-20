using Model.Model.Basis;
using Model.Model.Mesh;
using Telma;

namespace Model.Model.Elements.Quadrangle;

public sealed class HermiteQuadrangleFactory : IFiniteElementFactory<Vector2D>
{
    public IFiniteElement<Vector2D> CreateElement(IMesh<Vector2D> mesh, int[] vertices, int materialIndex) =>
        new FiniteElement<Vector2D>(
            Geometry: new QuadrangleGeometry(vertices) { Mesh = mesh },
            DOF: new Dof(),
            BasisSet: Basis,
            MaterialIndex: materialIndex
     );

    public static readonly IBasisSet<Vector2D> Basis = new BasisSet<Vector2D>(
        Quadratures.QuadrangleOrder9,
        QuadrangleBasis.Q3_Hermite
    );

    public sealed class Dof() : DofManager(dofCount: 16)
    {
        public override int NumberOfDofOnVertex => 4;
        public override int NumberOfDofOnEdge => 0;
        public override int NumberOfDofOnElement => 0;


        public override void SetEdgeDof(int localEdgeIndex, bool isOrientationFlipped, int n, int dofIndex) => throw new NotSupportedException();

        public override void SetVertexDof(int localVertexIndex, int n, int dofIndex)
        {
            if (n != 0) throw new NotSupportedException();
            ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(localVertexIndex, 4);
            _dof[localVertexIndex * 4 + n] = dofIndex;
        }
        
        public override void SetElementDof(int n, int dofIndex) => throw new NotSupportedException();
    }
}
