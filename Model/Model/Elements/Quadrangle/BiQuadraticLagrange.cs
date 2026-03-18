using System;
using System.Collections.Generic;
using System.Text;
using Model.Model;
using Model.Model.Basis;
using Model.Model.Elements;
using Model.Model.Elements.Quadrangle;
using Telma;

internal class BiQuadraticLagrangeQuadrangleFactory : IFiniteElementFactory
{
    public IFiniteElement Create(IMesh2D mesh, int[] vertices, int materialIndex) =>
        new FiniteElement(
            Geometry: new QuadrangleGeometry(vertices) { Mesh = mesh },
            DOF: new Dof(),
            BasisSet: Basis,
            MaterialIndex: materialIndex
     );

    public static readonly IBasisSet Basis = new BasisSet(
        Quadratures.QuadrangleOrder7,
        QuadrangleBasis.Q2_Lagrange
    );

    public sealed class Dof() : DofManager(dofCount: 9)
    {
        public override int NumberOfDofOnVertex => 1;
        public override int NumberOfDofOnEdge => 1;
        public override int NumberOfDofOnElement => 1;

        /// <summary>
        /// Ребра нумеруются с нижнего против часовой {0, 1, 2, 3}, на ребре 2 узла, которые имеют индексы 0 и 1 и нумеруются опять же по часовой
        /// </summary>
        public override void SetEdgeDof(int localEdgeIndex, bool isOrientationFlipped, int n, int dofIndex)
        {
            if (n != 0) throw new NotSupportedException();
            if (localEdgeIndex >= 4) throw new NotSupportedException();
            switch (localEdgeIndex)
            {
                case 0:
                    _dof[1] = dofIndex;
                    break;
                case 1:
                    _dof[5] = dofIndex;
                    break;
                case 2:
                    _dof[7] = dofIndex;
                    break;
                case 3:
                    _dof[3] = dofIndex;
                    break;
            }
        }


        public override void SetVertexDof(int localVertexIndex, int n, int dofIndex)
        {
            if (n != 0) throw new NotSupportedException();
            if (localVertexIndex >= 4) throw new NotSupportedException();
            switch (localVertexIndex)
            {
                case 0:
                    _dof[0] = dofIndex;
                    break;
                case 1:
                    _dof[2] = dofIndex;
                    break;
                case 2:
                    _dof[6] = dofIndex;
                    break;
                case 3:
                    _dof[8] = dofIndex;
                    break;
            }
        }

        /// <summary>
        /// локальная нумерация соответствует нумерации вершин(узлов на углах)
        /// 3---2
        /// |   |
        /// 0---1
        /// </summary>
        public override void SetElementDof(int n, int dofIndex)
        {
            if (n != 0) throw new NotSupportedException();
            _dof[4] = dofIndex;
        }
    }
}
