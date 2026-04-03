using System.Diagnostics;
using System.Globalization;
using Model.Core.CoordinateSystem;
using Model.Model.Elements;
using Telma;
using Telma.Extensions;

namespace Model.Model.Mesh;


public static class MeshInput
{
    extension(TextReader self)
    {
        public Mesh2D ReadMesh2D(
            ICoordinateTransform<Vector2D, Vector2D> coordinateSystem,
            IFiniteElementFactory2D finiteElementFactory,
            IBoundaryElementFactory2D boundaryElementFactory
        ) => self.ReadMesh(coordinateSystem, finiteElementFactory, boundaryElementFactory);

        public Mesh3D ReadMesh3D(
            ICoordinateTransform<Vector3D, Vector3D> coordinateSystem,
            IFiniteElementFactory3D finiteElementFactory,
            IBoundaryElementFactory3D boundaryElementFactory
        ) => self.ReadMesh(coordinateSystem, finiteElementFactory, boundaryElementFactory);

        public Mesh<TSpace, TBoundary> ReadMesh<TSpace, TBoundary>(
            ICoordinateTransform<TSpace, TSpace> coordinateSystem,
            IFiniteElementFactory<TSpace> finiteElementFactory,
            IBoundaryElementFactory<TSpace, TBoundary> boundaryElementFactory
        )
            where TSpace : IVectorBase<TSpace>
            where TBoundary : IVectorBase<TBoundary>
        {
            var mesh = new Mesh<TSpace, TBoundary>(coordinateSystem);
            int currentLine = 0; // Used for detailed exceptions messages

            self.ReadVertices(mesh, ref currentLine);
            self.ReadElements(mesh, finiteElementFactory, ref currentLine);
            self.ReadBoundaryElements(mesh, boundaryElementFactory, ref currentLine);

            return mesh;
        }

        private void ReadVertices<TSpace, TBoundary>(Mesh<TSpace, TBoundary> mesh, ref int currentLine)
            where TSpace : IVectorBase<TSpace>
            where TBoundary : IVectorBase<TBoundary>
        {
            ++currentLine;
            if (!int.TryParse(self.ReadLine(), out int vertexCount))
                throw new FormatException("Expected vertex count at line 1");
            for (int i = 1; i <= vertexCount; ++i)
            {
                ++currentLine;
                string str = self.ReadLine() ??
                    throw new FormatException($"Expected vertex at line {currentLine}");
                double[] vertices = ParseDoubles(str, i + 1);
                if (vertices.Length != TSpace.Dimensions)
                    throw new FormatException($"Invalid coordinates count {vertices.Length} at line {currentLine}, expected {TSpace.Dimensions}");
                mesh.AddVertex(TSpace.FromSpan(vertices));
            }
        }

        private void ReadElements<TSpace, TBoundary>(
            Mesh<TSpace, TBoundary> mesh,
            IFiniteElementFactory<TSpace> finiteElementFactory,
            ref int currentLine
        )
            where TSpace : IVectorBase<TSpace>
            where TBoundary : IVectorBase<TBoundary>
        {
            ++currentLine;
            if (!int.TryParse(self.ReadLine(), out int elementCount))
                throw new FormatException($"Expected element count at line {currentLine}");
            for (int i = 1; i <= elementCount; ++i)
            {
                ++currentLine;
                string str = self.ReadLine() ??
                    throw new FormatException($"Expected element at line {currentLine}");

                int[] buf = ParseIntegers(str, currentLine);

                try
                {
                    mesh.AddElement(finiteElementFactory, vertices: buf[..^1], materialIndex: buf[^1]);
                }
                catch (ArgumentException ex)
                {
                    throw new FormatException($"Exception at line {currentLine}: {ex.Message}", ex);
                }
            }
        }

        private void ReadBoundaryElements<TSpace, TBoundary>(
            Mesh<TSpace, TBoundary> mesh,
            IBoundaryElementFactory<TSpace, TBoundary> boundaryElementFactory,
            ref int currentLine
        )
            where TSpace : IVectorBase<TSpace>
            where TBoundary : IVectorBase<TBoundary>
        {
            ++currentLine;
            if (!int.TryParse(self.ReadLine(), out int boundaryElementCount))
                throw new FormatException($"Expected boundary element count at line {currentLine}");
            for (int i = 1; i <= boundaryElementCount; ++i)
            {
                ++currentLine;
                string str = self.ReadLine() ??
                    throw new FormatException($"Expected boundary element at line {currentLine}");
                int[] buf = ParseIntegers(str, currentLine);

                try
                {
                    mesh.AddBoundary(boundaryElementFactory, vertices: buf[..^1], boundaryIndex: buf[^1]);
                }
                catch (ArgumentException ex)
                {
                    throw new FormatException($"Exception at line {currentLine}: {ex.Message}", ex);
                }
            }
        }
    }

    private static int[] ParseIntegers(string line, int lineNumber)
    {
        try
        {
            return [.. line.Split(' ', StringSplitOptions.RemoveEmptyEntries).Select(s => int.Parse(s, CultureInfo.InvariantCulture))];
        }
        catch (FormatException ex)
        {
            throw new FormatException($"Failed to parse int[] at line {lineNumber}", ex);
        }
    }

    private static double[] ParseDoubles(string line, int lineNumber)
    {
        try
        {
            return [.. line.Split(' ', StringSplitOptions.RemoveEmptyEntries).Select(s => double.Parse(s, CultureInfo.InvariantCulture))];
        }
        catch (FormatException ex)
        {
            throw new FormatException($"Failed to parse double[] at line {lineNumber}", ex);
        }
    }
}
