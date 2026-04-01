using System.Diagnostics;
using Model.Model.Basis;
using Telma;

namespace Model.Model.Elements.Quadrangle;

public sealed class HermiteQuadrangleFactory : IFiniteElementFactory2D
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
        QuadrangleBasis.Q3_Hermite
    );

    public sealed class Dof() : DofManager(dofCount: 16)
    {
        public override int NumberOfDofOnVertex => 4;
        public override int NumberOfDofOnEdge => 0;
        public override int NumberOfDofOnElement => 0;

        public override void SetVertexDof(int localVertexIndex, int n, int dofIndex)
        {
            AssertIsValidVertexDofNumber(n);
            Debug.Assert(0 <= localVertexIndex && localVertexIndex < 4);
            _dof[localVertexIndex * 4 + n] = dofIndex;
        }

        public override void SetEdgeDof(int localEdgeIndex, bool isOrientationFlipped, int n, int dofIndex)
            => throw new NotSupportedException();


        public override void SetElementDof(int n, int dofIndex)
            => throw new NotSupportedException();
    }
}
