using System.Diagnostics;
using Model.Model.Basis;
using Telma;

namespace Model.Model.Elements.Triangle;


public sealed class LagrangeCubicTriangleFactory : IFiniteElementFactory2D
{
    public IFiniteElement2D CreateElement(IMesh2D mesh, int[] vertices, int materialIndex)
    {
        var basis = (MutableBasisSet<Vector2D>)DefaultBasis();
        return new FiniteElement2D(
            Geometry: new TriangleGeometry(vertices) { Mesh = mesh },
            DOF: new Dof(basis),
            BasisSet: basis,
            MaterialIndex: materialIndex
        );
    }

    // базис должен быть изменяемым как и в иерархическом (функции нужно менять местами)
    public static IBasisSet2D DefaultBasis() => new MutableBasisSet<Vector2D>(
        Quadratures.TriangleOrder18,
        TriangleBasis.Lagrange.L3L3L3,  //0
        TriangleBasis.Lagrange.L1L1L1,  //1
        TriangleBasis.Lagrange.L2L2L2,  //2
        TriangleBasis.Lagrange.L3L1L3,  //3
        TriangleBasis.Lagrange.L3L1L1,  //4
        TriangleBasis.Lagrange.L1L2L1,  //5
        TriangleBasis.Lagrange.L1L2L2,  //6
        TriangleBasis.Lagrange.L2L3L2,  //7
        TriangleBasis.Lagrange.L2L3L3,  //8
        TriangleBasis.Lagrange.L1L2L3   //9
    );

public sealed class Dof(MutableBasisSet<Vector2D> mutableBasis) : DofManager(dofCount: 10)
    {
        private readonly MutableBasisSet<Vector2D> _mutableBasisSet = mutableBasis;

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

            int index = 3 + 2 * localEdgeIndex;
            if ( isOrientationFlipped )
            {
                switch (localEdgeIndex)
                {
                    case 0:
                        _mutableBasisSet.MutableBasis[index] = TriangleBasis.Lagrange.L3L1L1;
                        _mutableBasisSet.MutableBasis[index + 1] = TriangleBasis.Lagrange.L3L1L3;
                        break;
                    case 1:
                        _mutableBasisSet.MutableBasis[index] = TriangleBasis.Lagrange.L1L2L2;
                        _mutableBasisSet.MutableBasis[index + 1] = TriangleBasis.Lagrange.L1L2L1;
                        break;
                    case 2:
                        _mutableBasisSet.MutableBasis[index] = TriangleBasis.Lagrange.L2L3L3;
                        _mutableBasisSet.MutableBasis[index + 1] = TriangleBasis.Lagrange.L2L3L2;
                        break;
                }
            }
            _dof[index + n] = dofIndex;
        }

        public override void SetElementDof(int n, int dofIndex)
        {
            AssertIsValidElementDofNumber(n);
            _dof[9] = dofIndex;
        }  
    }
}
