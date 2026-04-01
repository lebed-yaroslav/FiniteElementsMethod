
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
            int FreeDOFCount = DofNumerator<Vector2D, Vector1D>.RenumberFixedDof(init, boundaryCondition, DOFcount);


            //Списки смежности не собираются, выход за пределы
            var adjacency = PortraitGenerator.CreateAdjacencyList(init.AllElementsDof,0,FreeDOFCount);
            var adjacencyDirichlet = PortraitGenerator.CreateAdjacencyList(init.AllElementsDof, DOFcount, FreeDOFCount);
            var matrix = CsrMatrix.Portrait.Create(adjacency);
            var matrixDirichlet = CsrMatrix.Portrait.Create(adjacencyDirichlet);
        }
    }
}
