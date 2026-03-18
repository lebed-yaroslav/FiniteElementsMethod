using Model.Core.Matrix;
using Model.Model.Elements;

namespace Model.Model.Assembly;


public static class PortraitGenerator
{
    public static List<HashSet<int>> CreateAdjacencyListDirichlet(
        IEnumerable<IDofManager> dirichletElementsDof,
        int matrixSize,
        int freeDofCount
    )
    {
        int n = matrixSize;
        var adjacencyList = new List<HashSet<int>>(n);
        for (int i = 0; i < n; i++)
            adjacencyList.Add([]);

        foreach (var elementDof in dirichletElementsDof)
        {
            var dof = elementDof.Dof;
            int elementDOFCount = dof.Length;
            for (int i = 0; i < elementDOFCount; i++)
            {
                int row = dof[i] - freeDofCount;
                if (row < 0 || row >= n) continue; // Ignore non-dirichlet dofs

                var rowList = adjacencyList[row];
                for (int j = 0; j < elementDOFCount; j++)
                {
                    int col = dof[j] - freeDofCount;
                    if (col < 0 || col >= n) continue;
                    if (row <= col) continue; // Build portrait by lower triangle
                    rowList.Add(col);
                }
            }
        }
        return adjacencyList;
    }

    public static List<HashSet<int>> CreateAdjacencyList(
        IEnumerable<IDofManager> elementsDof,
        int freeDOFCount
    )
    {
        int n = freeDOFCount;
        var adjacencyList = new List<HashSet<int>>(n);
        for (int i = 0; i < n; i++)
            adjacencyList.Add([]);

        foreach (var dofManager in elementsDof)
        {
            var dof = dofManager.Dof;
            int elementDOFCount = dof.Length;
            for (int i = 0; i < elementDOFCount; i++)
            {
                int row = dof[i];
                if (row >= n) continue; // Ignore fixed elements

                var rowList = adjacencyList[row];
                for (int j = 0; j < elementDOFCount; j++)
                {
                    int col = dof[j];
                    if (row <= col) continue; // Build portrait by lower triangle
                    rowList.Add(col);
                }
            }
        }

        return adjacencyList;
    }


    public static CsrMatrix.Portrait GeneratePortrait(List<HashSet<int>> adjacencyList)
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
        return new CsrMatrix.Portrait(ig, jg);
    }
}
