using System;
using Model.Model.Basis;
using Telma;


namespace Model.Model.Elements.Quadrangle
{
    public sealed class BilinearQuadraticFactory : IFiniteElementFactory
    {

        public IFiniteElement Create(IMesh2D mesh, int[] vertices) =>
            new FiniteElement(
                Geometry: new QuadrangleGeometry(vertices) { Mesh = mesh },
                DOF: new Dof(),
                BasisSet: Basis
         );

        public static readonly IBasisSet Basis = new BasisSet(
            Quadratures.QuadrangleOrder7,
            QuadrangleBasis.Q1
        );

        public sealed class Dof() : DofManager(dofCount: 4)
        {
            public override int NumberOfDofOnVertex => 1;
            public override int NumberOfDofOnEdge => 0;
            public override int NumberOfDofOnElement => 0;


            public override void SetEdgeDof(int localEdgeIndex, int n, int dofIndex)
            {
                if (n != 0) throw new NotSupportedException();
                if (localEdgeIndex >= 4) throw new NotSupportedException();
                _dof[4 + localEdgeIndex] = dofIndex;
            }

            public override void SetVertexDof(int localVertexIndex, int n, int dofIndex)
            {
                if (n != 0) throw new NotSupportedException();
                if (localVertexIndex >= 4) throw new NotSupportedException();
                _dof[localVertexIndex] = dofIndex;
            }

            public override void SetElementDof(int n, int dofIndex)
                => throw new NotSupportedException();
        }
    }
}
