
using System.Diagnostics;
using Model.Core.CoordinateSystem;
using Model.Core.Matrix;
using Model.Model;
using Model.Model.Assembly;
using Model.Model.Basis;
//using Model.Model.Elements;
using Model.Model.Elements.Segment;
using Model.Model.Elements.Triangle;
using Model.Model.Integrator;
using Model.Model.Mesh;
using Model.Model.Problem;
using Telma;
using static Model.Core.CoordinateSystem.MatrixOperations;

namespace UnitTests.Elements.Triangle
{
    public class HierarchicalTriangle
    {
        [Fact]
        public void HierarchicalInit() 
        {
            string path = @"C:\Users\Lossky\Documents\GitHub\FiniteElementsMethod\test.txt";

            Mesh2D init = MeshInput.ReadMesh(File.OpenText(path), PolarCoordinateSystem.Instance,new HierarchicalQuadraticTriangleFactory(),new HierarchicalQuadraticSegmentFactory());
            var a = init.BoundaryElements;

            BoundaryCondition<Vector2D>[] boundaryCondition = new BoundaryCondition<Vector2D>[2];
            Func<Vector2D, double, double> x = (y,d) => y.X;

            boundaryCondition[0] = new BoundaryCondition<Vector2D>.Dirichlet(x);
            boundaryCondition[1] = new BoundaryCondition<Vector2D>.Dirichlet(x);

            //Нумератор + Перенумератор работает корректно
            int DOFcount = DofNumerator<Vector2D, Vector1D>.PreNumerateDof(init);

            NumericItegrator2D Integrator = NumericIntegrator<Vector2D, Vector1D, Ops2X2>.Instance;

            var element = init.FiniteElements.First();
            System.Func<Telma.Vector2D, double> lambda = x => 1;
            var stiffness = Integrator.CalculateLocalStiffness(element, lambda);
            var mass = Integrator.CalculateLocalMass(element, lambda);


            double[,] trueMass = new double[6, 6]
{
                {0.40000000000000169,0.19999999999999993,0.23333333333333361,0.077777777777777918,0.044444444444444189,0.088888888888889184 } ,
                {0.19999999999999993,0.40000000000000108,0.23333333333333331,0.077777777777777848,0.088888888888889,0.044444444444444189 } ,
                {0.23333333333333361,0.23333333333333331,0.53333333333333532,0.044444444444444189,0.10000000000000012,0.10000000000000024} ,
                {0.077777777777777918,0.077777777777777848,0.044444444444444189,0.02539682539682549,0.014285714285714195,0.014285714285714221} ,
                {0.044444444444444189,0.088888888888889,0.10000000000000012,0.014285714285714195,0.031746031746031841,0.015873015873015778} ,
                {0.088888888888889184,0.044444444444444189,0.10000000000000024,0.014285714285714221,0.015873015873015778,0.031746031746031911}
};
            for (int i = 0; i < 6; ++i)
            {
                for (int j = 0; j < 6; ++j)
                {
                    Assert.Equal(trueMass[i, j], mass[i, j], 1e-12);
                }
            }


            double[,] trueStiffnes = new double[6, 6] 
            {
                {2.8333333333333384,-0.16666666666666696,-2.6666666666666714,0.83333333333333437,-0.89583333333333448,0.22916666666666696 } ,
                {-0.16666666666666696,0.16666666666666696,0,5.8980598183211441E-17,0.062500000000000139,-0.062500000000000139 } ,
                {-2.6666666666666714,0,2.6666666666666714,-0.83333333333333448,0.83333333333333448,-0.16666666666666693} ,
                {0.83333333333333437,5.8980598183211441E-17,-0.83333333333333448,0.42500000000000132,-0.40000000000000108,0.033333333333333326 } ,
                {-0.89583333333333448,0.062500000000000139,0.83333333333333448,-0.40000000000000108,0.43333333333333451,-0.0666666666666668 } ,
                {0.22916666666666696,-0.062500000000000139,-0.16666666666666693,0.033333333333333326,-0.0666666666666668,0.50000000000000322 }
            };

            for (int i = 0; i < 6; ++i) 
            {
                for (int j = 0; j < 6; ++j) 
                {
                    Assert.Equal(trueStiffnes[i, j], stiffness[i,j],1e-12);
                }
            }





            var DOF = DofManager.NumerateDof(init,boundaryCondition);
 
            Assert.Equal(DOF.TotalDofCount, 12);

            //int FreeDOFCount = DofNumerator<Vector2D, Vector1D>.RenumberFixedDof(init, boundaryCondition, DOFcount);

            Assert.Equal(DOF.FreeDofCount, 6);

            List<HashSet<int>> trueAdjacency = new();
            trueAdjacency.Add([]);
            trueAdjacency.Add([0]);
            trueAdjacency.Add([0,1]);
            trueAdjacency.Add([0,1]);
            trueAdjacency.Add([0,3,1]);
            trueAdjacency.Add([0,4]);

            //Списки смежности не собираются, выход за пределы  PS: Исправлено
            var adjacency = PortraitGenerator.CreateAdjacencyList(init.AllElementsDof,0, DOF.FreeDofCount);

            Assert.Equal(adjacency, trueAdjacency);

            List<HashSet<int>> trueAdjacencyDirichlet = new();
            trueAdjacencyDirichlet.Add([]);
            trueAdjacencyDirichlet.Add([]);
            trueAdjacencyDirichlet.Add([1]);
            trueAdjacencyDirichlet.Add([0]);
            trueAdjacencyDirichlet.Add([3,0]);
            trueAdjacencyDirichlet.Add([2, 1]);

            //Лишний элемент в портрете
            var adjacencyDirichlet = PortraitGenerator.CreateAdjacencyList(init.BoundaryElements
            .Where(e => boundaryCondition[e.BoundaryIndex] is BoundaryCondition<Vector2D>.Dirichlet)
            .Select(e => e.DOF), DOF.FreeDofCount, DOF.TotalDofCount);
            Assert.Equal(adjacencyDirichlet, trueAdjacencyDirichlet);

            var matrix = (CsrMatrix)(new CsrMatrixFactory().Create(adjacency));

      
            //var matrix = CsrMatrix.Portrait.Create(adjacency);

            int[] trueMatrixIG = [0,0,1,3,5,8,10];
            int[] trueMatrixJG = [0,0,1,0,1,0,1,3,0,4];

            Assert.Equal(matrix.Ig, trueMatrixIG);
            Assert.Equal(matrix.Jg, trueMatrixJG);
            
            var matrixDirichlet = (CsrMatrix)(new CsrMatrixFactory().Create(adjacencyDirichlet));

            int[] trueMatrixDirichletIG = [0,0,0,1,2,4,6];
            int[] trueMatrixDirichletJG = [1,0,0,3,1,2];

            Assert.Equal(matrixDirichlet.Ig, trueMatrixDirichletIG);
            Assert.Equal(matrixDirichlet.Jg, trueMatrixDirichletJG);





        }
    }
}
