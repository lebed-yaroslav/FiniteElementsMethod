using Model.Model.Basis;
using Model.Model.Elements.Rectangle;
using Telma;

namespace Model.Model.Elements.Quadrangle;

public sealed class HermiteQuadraticFactory : IFiniteElementFactory
{
    public IFiniteElement Create(IMesh2D mesh, int[] vertices) =>
        new FiniteElement(
            Geometry: new RectangleGeometry(vertices) { Mesh = mesh },
            DOF: new Dof(),
            BasisSet: Basis
     );

    public static readonly IBasisSet Basis = new BasisSet(
        Quadratures.QuadrangleOrder9,
        QuadrangleBasis.Q3_Hermite
    );

    public sealed class Dof() : DofManager(dofCount: 16)
    {
        public override int NumberOfDofOnVertex => 4;
        public override int NumberOfDofOnEdge => 0;
        public override int NumberOfDofOnElement => 0;


        public override void SetEdgeDof(int localEdgeIndex, int n, int dofIndex) => throw new NotSupportedException();

        public override void SetVertexDof(int localVertexIndex, int n, int dofIndex)
        {
            if (n != 0) throw new NotSupportedException();

            if (localVertexIndex >= 4) throw new ArgumentOutOfRangeException(nameof(localVertexIndex));
            _dof[localVertexIndex * 4 + n] = dofIndex;
        }
        
        public override void SetElementDof(int n, int dofIndex) => throw new NotSupportedException();
    }
}