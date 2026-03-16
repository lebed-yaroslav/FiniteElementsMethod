using System.Xml.Linq;
using Model.Model.Elements;
using Telma;

namespace Model.Model.MeshAssistant;

public static class CreatePortrait
{
    public static List<HashSet<int>> CreateAdjacencyListDirichlet(this Mesh2D mesh, int DimensionMatrix, int FreeDOFCount)
    {
        int n = DimensionMatrix;                       //Размерность матрицы.
        var adjacencyList = new List<HashSet<int>>(n); //Матрица смежности.
        for (int i = 0; i < n; i++)                    //Выделяется память для матрицы.
            adjacencyList.Add(new());

        foreach (var element in mesh.BoundaryElements)
        {
            if (element.BoundaryIndex != 1) continue;
            var dof = element.DOF.Dof;
            int elementDOFCount = dof.Length;
            for (int i = 0; i < elementDOFCount; i++)
            {
                int row = dof[i] - FreeDOFCount;
                if (row < 0 || row >= n) continue;  //Отбрасываем зафиксированные элементы       ?

                var rowList = adjacencyList[row];
                for (int j = 0; j < elementDOFCount; j++)
                {
                    int col = dof[j] - FreeDOFCount;
                    if (col < 0 || col >= n) continue;
                    if (row <= col) continue;       //Отбрасываем симметричные элементы для построения портрета по нижнему треугольнику
                    rowList.Add(col);
                }
            }
        }

        for (int i = 0; i < n; i++)
            adjacencyList[i].OrderBy(j => j);

        return adjacencyList;
    }

    public static List<HashSet<int>> CreateAdjacencyList(this Mesh2D mesh, int FreeDOFCount)
    {
        int n = FreeDOFCount;                     //Размерность матрицы.
        var adjacencyList = new List<HashSet<int>>(n); //Матрица смежности.
        for (int i = 0; i < n; i++)                    //Выделяется память для матрицы.
            adjacencyList.Add(new());

        foreach (var element in mesh.FiniteElements)
        {
            var dof = element.DOF.Dof;
            int elementDOFCount = dof.Length;
            for (int i = 0; i < elementDOFCount; i++)
            {
                int row = dof[i];

                if (row >= n) continue;  //Отбрасываем зафиксированные элементы       ?

                var rowList = adjacencyList[row];
                for (int j = 0; j < elementDOFCount; j++)
                {
                    int col = dof[j];
                    if (row <= col) continue;       //Отбрасываем симметричные элементы для построения портрета по нижнему треугольнику
                    rowList.Add(col);
                }
            }
        }

        foreach (var element in mesh.BoundaryElements)
        {
            var dof = element.DOF.Dof;
            int elementDOFCount = dof.Length;
            for (int i = 0; i < elementDOFCount; i++)
            {
                int row = dof[i];

                if (row >= n) continue;  //Отбрасываем зафиксированные элементы       ?

                var rowList = adjacencyList[row];
                for (int j = 0; j < elementDOFCount; j++)
                {
                    int col = dof[j];
                    if (row <= col) continue;       //Отбрасываем симметричные элементы для построения портрета по нижнему треугольнику
                    rowList.Add(col);
                }
            }
        }

        for (int i = 0; i < n; i++)
            adjacencyList[i].OrderBy(j => j);

        return adjacencyList;
    }

    /// <summary>
    /// Возвращает профиль матрицы в строчно-столбцовом формате
    /// </summary>
    /// <param name="adjacencyList">Список смежности</param>
    /// <returns></returns>
    public static (int[] ig, int[] jg) GenerateIGJG(List<HashSet<int>> adjacencyList)
    {
        int n = adjacencyList.Count;

        int sum = 0;
        int[] ig = new int[n + 1];
        for (int i = 0; i < n; i++) //Задаем массив IG.
        {
            ig[i] = sum;
            sum += adjacencyList[i].Count;
        }
        ig[n] = sum; //инициализируем последний элемент.

        int[] jg = new int[sum]; //Создаем список jg, чтобы можно было использовать метод AddRange.
        int addr = 0;
        for (int i = 0; i < n; i++)
        {
            //Сортируем матрицу смежности, чтобы можно было добавлять в конец массива JG строки этой матрицы.
            foreach (var k in adjacencyList[i].OrderBy(j => j))
                jg[addr++] = k; //Добавляем в конец JG отсортированные строки матрицы смежности.
        }
        return (ig, jg);
    }
}
