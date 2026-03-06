using Model.Model.Basis;
using Telma;

namespace Model.Model.Elements.Triangle;

public sealed class LinearTriangleFactory : IFiniteElementFactory
{
    public IFiniteElement Create(IMesh2D mesh, int[] vertices)
        => new FiniteElement(
            Geometry: new TriangleGeometry(vertices) { Mesh = mesh },
            DOF: new Dof(),
            BasisSet: Basis
    );

    public static readonly IBasisSet Basis = new BasisSet(
       Quadratures.TriangleOrder3,
       TriangleBasis.L1,
       TriangleBasis.L2,
       TriangleBasis.L3
   );

    public sealed class Dof() : DofManager(dofCount: 3)
    {
        public override int NumberOfDofOnVertex => 1;
        public override int NumberOfDofOnEdge => 0;
        public override int NumberOfDofOnElement => 0;

        public override void SetVertexDof(int localVertexIndex, int n, int dofIndex)
        {
            if (n != 0) throw new NotSupportedException();
            if (localVertexIndex <= 3) throw new NotSupportedException();
            _dof[localVertexIndex] = dofIndex;
        }

        public override void SetEdgeDof(int localEdgeIndex, bool isOrientationFlipped, int n, int dofIndex) =>
            throw new NotSupportedException();

        public override void SetElementDof(int n, int dofIndex) =>
            throw new NotSupportedException();
    }
}
