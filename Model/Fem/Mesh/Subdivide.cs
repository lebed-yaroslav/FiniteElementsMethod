using Model.Fem.Elements;
using Model.Fem.Elements.Quadrangle;
using Model.Fem.Elements.Segment;
using Model.Fem.Elements.Triangle;
using Telma;

namespace Model.Fem.Mesh;

// FIXME: Heavily depends on internal Geometry ordering
public static class SubdivideMesh
{
    extension(Mesh2D self)
    {
        public Mesh2D Subdivide(
            IFiniteElementFactory2D elementFactory,
            IBoundaryElementFactory2D boundaryFactory
        )
        {
            var mesh = new Mesh2D(self.CoordinateSystem);
            // 1. Copy initial vertices
            for (int i = 0; i < self.VertexCount; ++i)
                mesh.AddVertex(self[i]);

            // 2. Subdivide elements
            Dictionary<Edge, int> edgeMidpoints = [];
            foreach (var element in self.FiniteElements)
            {
                var midpoints = new int[element.Geometry.EdgeCount];
                int index = 0;
                foreach (var unsortedEdge in element.Edges)
                {

                    var edge = unsortedEdge.Sorted();
                    int midpointId = -1;
                    if (!edgeMidpoints.TryGetValue(edge, out midpointId))
                    {
                        var midpoint = 0.5 * (mesh[edge.I] + mesh[edge.J]);
                        midpointId = mesh.AddVertex(midpoint);
                        edgeMidpoints.Add(edge, midpointId);
                    }
                    midpoints[index++] = midpointId;
                }

                switch (element.Geometry)
                {
                    case TriangleGeometry:
                        mesh.AddSubdividedTriangle(element, midpoints, elementFactory);
                        break;
                    case QuadrangleGeometry:
                        mesh.AddSubdividedQuadrangle(element, midpoints, elementFactory);
                        break;
                    default:
                        throw new NotSupportedException();
                }
            }

            // 3. Subdivide boundaries
            foreach (var element in self.BoundaryElements)
            {
                switch (element.Geometry)
                {
                    case SegmentGeometry<Vector2D>:
                        mesh.AddSubdividedSegment(element, edgeMidpoints, boundaryFactory);
                        break;
                    default: throw new NotSupportedException();
                }
            }

            return mesh;
        }

        private void AddSubdividedTriangle(
            IFiniteElement2D element,
            ReadOnlySpan<int> midpoints,
            IFiniteElementFactory2D factory
        )
        {
            var v1 = element.Vertices[0];
            var v2 = element.Vertices[1];
            var v3 = element.Vertices[2];
            var m1 = midpoints[0];
            var m2 = midpoints[1];
            var m3 = midpoints[2];

            self.AddElement(factory, [v1, m1, m3], element.MaterialIndex);
            self.AddElement(factory, [m1, m2, m3], element.MaterialIndex);
            self.AddElement(factory, [m1, v2, m2], element.MaterialIndex);
            self.AddElement(factory, [m3, m2, v3], element.MaterialIndex);
        }

        private void AddSubdividedQuadrangle(
            IFiniteElement2D element,
            ReadOnlySpan<int> midpoints,
            IFiniteElementFactory2D factory
        )
        {
            var v1 = element.Vertices[0];
            var v2 = element.Vertices[1];
            var v3 = element.Vertices[2];
            var v4 = element.Vertices[3];
            var m1 = midpoints[0];
            var m2 = midpoints[1];
            var m3 = midpoints[2];
            var m4 = midpoints[3];

            var center = self.AddVertex(0.25 * (self[v1] + self[v2] + self[v3] + self[v4]));

            self.AddElement(factory, [v1, m1, center, m4], element.MaterialIndex);
            self.AddElement(factory, [m1, v2, m2, center], element.MaterialIndex);
            self.AddElement(factory, [center, m2, v3, m3], element.MaterialIndex);
            self.AddElement(factory, [m4, center, m3, v4], element.MaterialIndex);
        }

        private void AddSubdividedSegment(
            IBoundaryElement2D element,
            Dictionary<Edge, int> midpoints,
            IBoundaryElementFactory2D factory
        )
        {
            var v1 = element.Vertices[0];
            var v2 = element.Vertices[1];
            int midpoint = midpoints[new Edge(v1, v2).Sorted()];

            self.AddBoundary(factory, [v1, midpoint], element.BoundaryIndex);
            self.AddBoundary(factory, [midpoint, v2], element.BoundaryIndex);
        }
    }
}
