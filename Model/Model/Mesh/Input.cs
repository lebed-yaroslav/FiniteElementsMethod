using Model.Core.CoordinateSystem;
using Model.Model.Elements;
using Telma;

namespace Model.Model.Mesh;


public static class MeshInput
{
    extension(TextReader self)
    {
        public Mesh2D ReadMesh(
            ICoordinateTransform<Vector2D, Vector2D> coordinateSystem,
            IFiniteElementFactory<Vector2D> finiteElementFactory,
            IBoundaryElementFactory<Vector2D, Vector1D> boundaryElementFactory
        )
        {
            var mesh = new Mesh2D(coordinateSystem);

            // 1. Read vertices
            if (!int.TryParse(self.ReadLine(), out int n))
                throw new Exception("Некорректные входные данные (первая строка должна содержать кол-во узлов (n))!");
            string? str;
            for (int i = 0; i < n; ++i)
            {
                str = self.ReadLine() ?? throw new Exception("Некоректное кол-во узлов!");
                var vertex = (double[])str.Split(' ', StringSplitOptions.RemoveEmptyEntries).Select(double.Parse);
                if (vertex.Length != 2) throw new Exception("Введена не двумерная сетка!");
                mesh.AddVertex(new(vertex[0], vertex[1]));
            }

            // 2. Read elements
            if (!int.TryParse(self.ReadLine(), out n))
                throw new Exception("Некорректные входные данные (строка n+1 должна содержать кол-во элементов)!");
            for (int i = 0; i < n; ++i)
            {
                str = self.ReadLine() ?? throw new Exception("Неправильная структура входных данных!");
                int[] buf = [.. str.Split(' ', StringSplitOptions.RemoveEmptyEntries).Select(int.Parse)];
                mesh.AddElement(finiteElementFactory, vertices: buf[..^1], materialIndex: buf[^1]);
            }

            // 3. Read boundary elements
            if (!int.TryParse(self.ReadLine(), out n))
                throw new Exception("Некорректные входные данные (первая строка должна содержать кол-во узлов (n))!");
            for (int i = 0; i < n; ++i)
            {
                str = self.ReadLine() ?? throw new Exception("Неправильная структура входных данных!");
                var buf = str.Split(' ', StringSplitOptions.RemoveEmptyEntries).Select(int.Parse).ToArray();
                mesh.AddBoundary(boundaryElementFactory, vertices: buf[0..2], boundaryIndex: buf[2]);
            }

            return mesh;
        }
    }
}
