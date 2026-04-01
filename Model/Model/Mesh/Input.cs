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

            // 1. Read vertices
            if (!int.TryParse(self.ReadLine(), out int n))
                throw new Exception("Некорректные входные данные (первая строка должна содержать кол-во узлов (n))!");
            string? str;
            for (int i = 0; i < n; ++i)
            {
                str = self.ReadLine() ?? throw new Exception("Некоректное кол-во узлов!");
                double[] vertex = [.. str.Split(' ', StringSplitOptions.RemoveEmptyEntries).Select(double.Parse)];
                if (vertex.Length != TSpace.Dimensions) throw new Exception($"Неверное количество вершин {vertex.Length}, ожидалось {TSpace.Dimensions}");
                mesh.AddVertex(TSpace.FromSpan(vertex));
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
                int[] buf = [.. str.Split(' ', StringSplitOptions.RemoveEmptyEntries).Select(int.Parse)];
                mesh.AddBoundary(boundaryElementFactory, vertices: buf[0..^1], boundaryIndex: buf[^1]);
            }

            return mesh;
        }
    }
}
