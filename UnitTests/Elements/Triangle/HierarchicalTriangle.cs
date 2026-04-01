
using Model.Core.CoordinateSystem;
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

            var init = MeshInput.ReadMesh(File.OpenText(path), PolarCoordinateSystem.Instance,new HierarchicalCubicTriangleFactory(),new HierarchicalQuadraticSegmentFactory());
           
        }
    }
}
