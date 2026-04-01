
using System.Diagnostics;
using Model.Core.CoordinateSystem;
using Model.Core.Matrix;
using Model.Model;
using Model.Model.Assembly;
using Model.Model.Basis;
using Model.Model.Elements;
using Model.Model.Elements.Segment;
using Model.Model.Elements.Triangle;
using Model.Model.Mesh;
using Telma;

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
            int DOFcount = DofNumerator<Vector2D, Vector1D>.NumerateDof(init);

            Assert.Equal(DOFcount, 12);

            int FreeDOFCount = DofNumerator<Vector2D, Vector1D>.RenumberFixedDof(init, boundaryCondition, DOFcount);

            Assert.Equal(FreeDOFCount, 6);

            List<HashSet<int>> trueAdjacency = new();
            trueAdjacency.Add([]);
            trueAdjacency.Add([0]);
            trueAdjacency.Add([0,1]);
            trueAdjacency.Add([0,1]);
            trueAdjacency.Add([0,3,1]);
            trueAdjacency.Add([0,4]);

            //Списки смежности не собираются, выход за пределы  PS: Исправлено
            var adjacency = PortraitGenerator.CreateAdjacencyList(init.AllElementsDof,0,FreeDOFCount);

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
            .Select(e => e.DOF), FreeDOFCount, DOFcount);
            Assert.Equal(adjacencyDirichlet, trueAdjacencyDirichlet);


            var matrix = CsrMatrix.Portrait.Create(adjacency);

            int[] trueMatrixIG = [0,0,1,3,5,8,10];
            int[] trueMatrixJG = [0,0,1,0,1,0,1,3,0,4];

            Assert.Equal(matrix.Ig, trueMatrixIG);
            Assert.Equal(matrix.Jg, trueMatrixJG);
            
            var matrixDirichlet = CsrMatrix.Portrait.Create(adjacencyDirichlet);

            int[] trueMatrixDirichletIG = [0,0,0,1,2,4,6];
            int[] trueMatrixDirichletJG = [1,0,0,3,1,2];

            Assert.Equal(matrixDirichlet.Ig, trueMatrixDirichletIG);
            Assert.Equal(matrixDirichlet.Jg, trueMatrixDirichletJG);

        }
    }
}
