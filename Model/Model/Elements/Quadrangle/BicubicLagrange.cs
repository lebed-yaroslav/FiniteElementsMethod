using System.Xml.Linq;
using Model.Model.Basis;
using Telma;

namespace Model.Model.Elements.Quadrangle;

public sealed class BicubicLagrangeQuadrangleFactory : IFiniteElementFactory
{
    public IFiniteElement Create(IMesh2D mesh, int[] vertices) =>
        new FiniteElement(
            Geometry: new QuadrangleGeometry(vertices) { Mesh = mesh },
            DOF: new Dof(),
            BasisSet: Basis
     );

    public static readonly IBasisSet Basis = new BasisSet(
        Quadratures.QuadrangleOrder7,
        QuadrangleBasis.Q3_Lagrange
    );

    public sealed class Dof() : DofManager(dofCount: 16)
    {
        public override int NumberOfDofOnVertex => 1;
        public override int NumberOfDofOnEdge => 2;
        public override int NumberOfDofOnElement => 4;

        /// <summary>
        /// Ребра нумеруются с нижнего против часовой {0, 1, 2, 3}, на ребре 2 ущла, которые имеют индексы 0 и 1 и нумеруются опять же по часовой
        /// </summary>
        public override void SetEdgeDof(int localEdgeIndex, int n, int dofIndex)
        {
            if (n < 0 || n >= 2) throw new NotSupportedException();
            if (localEdgeIndex >= 4) throw new NotSupportedException();
            switch (localEdgeIndex)
            {
                case 0:
                    _dof[1 + n] = dofIndex;
                    break;
                case 1:
                    _dof[7 + n * 4] = dofIndex;
                    break;
                case 2:
                    _dof[14 - n] = dofIndex;
                    break;
                case 3:
                    _dof[8 - n * 4] = dofIndex;
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
                    _dof[3] = dofIndex;
                    break;
                case 2:
                    _dof[15] = dofIndex;
                    break;
                case 3:
                    _dof[12] = dofIndex;
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
            if (n < 0 || n >= 4) throw new NotSupportedException();
            switch (n)
            {
                case 0:
                    _dof[5] = dofIndex;
                    break;
                case 1:
                    _dof[6] = dofIndex;
                    break;
                case 2:
                    _dof[9] = dofIndex;
                    break;
                case 3:
                    _dof[10] = dofIndex;
                    break;
            }
        }
    }
}