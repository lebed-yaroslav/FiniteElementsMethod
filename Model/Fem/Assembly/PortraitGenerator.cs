using Model.Fem.Elements;

namespace Model.Fem.Assembly;


public static class PortraitGenerator
{
    /// <summary>
    /// Производит построение списка смежности степеней свободы, используемого
    /// для построения потрета глобальной матрицы (нижнего треугольника).
    /// </summary>
    /// <param name="elementsDof">Степени свободы всех элементов сетки</param>
    /// <param name="minDofIndex">Минимальный индекс степени свободы, включаемой в смежную структуру</param>
    /// <param name="maxDofIndex">Максимальный индекс степени свободы, включаемой в смежную структуру</param>
    public static List<HashSet<int>> CreateAdjacencyList(
        IEnumerable<IDofManager> elementsDof,
        int minDofIndex,
        int maxDofIndex
    )
    {
        int n = maxDofIndex - minDofIndex + 1;

        var adjacencyList = new List<HashSet<int>>(n);
        for (int i = 0; i < n; i++)
            adjacencyList.Add([]);

        foreach (var elementDof in elementsDof)
        {
            var dof = elementDof.Dof;
            foreach (var dofI in dof)
            {
                int row = dofI - minDofIndex;
                if (row < 0 || row >= n) continue;

                var rowAdjacent = adjacencyList[row];
                foreach (var dofJ in dof)
                {
                    int col = dofJ - minDofIndex;
                    if (col < 0 || col >= n) continue;
                    //if (row <= col) continue; // Build portrait by lower triangle
                    rowAdjacent.Add(col);
                }
            }
        }
        return adjacencyList;
    }
    public static List<HashSet<int>> CreateAdjacencyListForPardiso(
        IEnumerable<IDofManager> elementsDof,
        int minDofIndex,
        int maxDofIndex
    )
    {
        int n = maxDofIndex - minDofIndex + 1;

        var adjacencyList = new List<HashSet<int>>(n);
        for (int i = 0; i < n; i++)
            adjacencyList.Add([]);

        foreach (var elementDof in elementsDof)
        {
            var dof = elementDof.Dof;
            foreach (var dofI in dof)
            {
                int row = dofI - minDofIndex;
                if (row < 0 || row >= n) continue;

                var rowAdjacent = adjacencyList[row];
                foreach (var dofJ in dof)
                {
                    int col = dofJ - minDofIndex;
                    if (col < 0 || col >= n) continue;
                    //if (row <= col) continue; // Build portrait by lower triangle
                    rowAdjacent.Add(col);
                }
            }
        }
        return adjacencyList;
    }
}
