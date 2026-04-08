using System.Diagnostics;
using Model.Model.Basis;
using Telma;

namespace Model.Model.Elements.Triangle;

public sealed class LinearTriangleFactory : IFiniteElementFactory2D
{
    public IFiniteElement2D CreateElement(IMesh2D mesh, int[] vertices, int materialIndex)
        => new FiniteElement2D(
            Geometry: new TriangleGeometry(vertices) { Mesh = mesh },
            DOF: new Dof(),
            BasisSet: Basis,
            MaterialIndex: materialIndex
    );

    public static readonly IBasisSet2D Basis = new BasisSet2D(
        Quadratures.TriangleOrder3,
        TriangleBasis.L3,
        TriangleBasis.L1,
        TriangleBasis.L2
   );

    public sealed class Dof() : DofManager(dofCount: 3)
    {
        public override int NumberOfDofOnVertex => 1;
        public override int NumberOfDofOnEdge => 0;
        public override int NumberOfDofOnElement => 0;

        public override void SetVertexDof(int localVertexIndex, int n, int dofIndex)
        {
            AssertIsValidVertexDofNumber(n);
            Debug.Assert(0 <= localVertexIndex && localVertexIndex < 3);
            _dof[localVertexIndex] = dofIndex;
        }

        public override void SetEdgeDof(int localEdgeIndex, bool isOrientationFlipped, int n, int dofIndex) =>
            throw new NotSupportedException();

        public override void SetElementDof(int n, int dofIndex) =>
            throw new NotSupportedException();
    }
}
