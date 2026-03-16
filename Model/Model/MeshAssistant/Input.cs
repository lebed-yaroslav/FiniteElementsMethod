using Telma;
using Model.Model.Elements;

namespace Model.Model.MeshAssistant;

public static class InputAll
{
    public static void Input<TFiniteElementFactory, TBoundaryElementFactory, TSpace>(this Mesh2D mesh, TextReader input)
        where TFiniteElementFactory : IFiniteElementFactory<Vector2D>, new()
        where TBoundaryElementFactory : IBoundaryElementFactory<Vector2D, Vector1D>, new()
    {
        //Ввод узлов.
        if (!int.TryParse(input.ReadLine(), out int n))
            throw new Exception("Некорректные входные данные (первая строка должна содержать кол-во узлов (n))!");
        string? str;
        for (int i = 0; i < n; ++i)
        {
            str = input.ReadLine() ?? throw new Exception("Некоректное кол-во узлов!");
            var vertex = (double[])str.Split(' ', StringSplitOptions.RemoveEmptyEntries).Select(double.Parse);
            if (vertex.Length != 2) throw new Exception("Введена не двумерная сетка!");
            mesh.AddVertex(new Vector2D(vertex[0], vertex[1]));
        }

        //Ввод элементов.
        if (!int.TryParse(input.ReadLine(), out n))
            throw new Exception("Некорректные входные данные (n+1 строка должна содержать кол-во элементов)!");
        for (int i = 0; i < n; ++i)
        {
            str = input.ReadLine() ?? throw new Exception("Неправильная структура входных данных!");
            var buf = (int[])str.Split(' ', StringSplitOptions.RemoveEmptyEntries).Select(int.Parse);
            mesh.AddElement(new TFiniteElementFactory(), buf[0..2], buf[2]);
        }

        //Ввод краевых условий. Возможно, сделано лишнее действие(предполагается, что элементы могут содержать разное кол-во вершин)
        if (!int.TryParse(input.ReadLine(), out n))
            throw new Exception("Некорректные входные данные (первая строка должна содержать кол-во узлов (n))!");
        for (int i = 0; i < n; ++i)
        {
            str = input.ReadLine() ?? throw new Exception("Неправильная структура входных данных!");
            var buf = str.Split(' ', StringSplitOptions.RemoveEmptyEntries).Select(int.Parse).ToArray();
            mesh.AddBoundary(new TBoundaryElementFactory(), buf[0..2], buf[2]);
        }

        input.Close();
    }
}
