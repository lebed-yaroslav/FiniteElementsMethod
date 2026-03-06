using Model.Model.Basis;
using Telma;

namespace Model.Model.Elements.Triangle;

public sealed class LagrangeCubicTriangleFactory : IFiniteElementFactory
{
   public IFiniteElement Create(IMesh2D mesh, int[] vertices, int materialIndex) =>
        new FiniteElement(
            Geometry: new TriangleGeometry(vertices) { Mesh = mesh },
            DOF: new Dof(),
            BasisSet: Basis,
            MaterialIndex: materialIndex
     );

   // базис должен быть изменяемым как и в иерархическом (функции нужно менять местами)
   public static readonly IBasisSet Basis = new BasisSet(
       Quadratures.TriangleOrder18,
       TriangleBasis.LagrangeL3L3L3,
       TriangleBasis.LagrangeL1L1L1,
       TriangleBasis.LagrangeL2L2L2,
       TriangleBasis.LagrangeL3L1L3,
       TriangleBasis.LagrangeL3L1L1,
       TriangleBasis.LagrangeL1L2L1,
       TriangleBasis.LagrangeL1L2L2,
       TriangleBasis.LagrangeL2L3L2,
       TriangleBasis.LagrangeL2L3L3,
       TriangleBasis.LagrangeL1L2L3
   );

   public sealed class Dof() : DofManager(dofCount: 10)
   {
      public override int NumberOfDofOnVertex => 1;
      public override int NumberOfDofOnEdge => 2;
      public override int NumberOfDofOnElement => 1;


      public override void SetEdgeDof(int localEdgeIndex, bool isOrientationFlipped, int n, int dofIndex)
      {
         if (n != 0) throw new NotSupportedException();
         if (localEdgeIndex >= 6) throw new NotSupportedException();
         _dof[3 + localEdgeIndex] = dofIndex;
      }


      public override void SetVertexDof(int localVertexIndex, int n, int dofIndex)
      {
         if (n != 0) throw new NotSupportedException();
         if (localVertexIndex >= 3) throw new NotSupportedException();
         _dof[localVertexIndex] = dofIndex;
      }

      public override void SetElementDof(int n, int dofIndex)
      {
         _dof[9] = dofIndex;
      }  
   }
}
