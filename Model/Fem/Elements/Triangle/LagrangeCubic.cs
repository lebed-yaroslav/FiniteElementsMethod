using System.Diagnostics;
using Model.Fem.Basis;
using Telma;

namespace Model.Fem.Elements.Triangle;


public sealed class LagrangeCubicTriangleFactory : IFiniteElementFactory2D
{
    public IFiniteElement2D CreateElement(IMesh2D mesh, int[] vertices, int materialIndex)
    {
        return new FiniteElement2D(
            Geometry: new TriangleGeometry(vertices) { Mesh = mesh },
            DOF: new Dof(),
            BasisSet: Basis,
            MaterialIndex: materialIndex
        );
    }

    public static IBasisSet2D Basis => new BasisSet2D(
        Quadratures.TriangleOrder18,
        TriangleBasis.Lagrange.L3L3L3,
        TriangleBasis.Lagrange.L1L1L1,
        TriangleBasis.Lagrange.L2L2L2,
        TriangleBasis.Lagrange.L3L1L3,
        TriangleBasis.Lagrange.L3L1L1,
        TriangleBasis.Lagrange.L1L2L1,
        TriangleBasis.Lagrange.L1L2L2,
        TriangleBasis.Lagrange.L2L3L2,
        TriangleBasis.Lagrange.L2L3L3,
        TriangleBasis.Lagrange.L1L2L3
    );

    public sealed class Dof() : ElementDof(dofCount: 10)
    {
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
            if (isOrientationFlipped) n = 1 - n;
            _dof[3 + 2 * localEdgeIndex + n] = dofIndex;
        }

        public override void SetElementDof(int n, int dofIndex)
        {
            AssertIsValidElementDofNumber(n);
            _dof[9] = dofIndex;
        }  
    }
}
