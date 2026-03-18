using Model.Model.Elements;

namespace Model.Model.Assembly;


public static partial class DofNumerator
{
    /// <summary>
    /// Производит перенумерацию степеней свободы таким образом,
    /// что все зафиксированные условиями Дирихле узлы оказываются в конце
    /// </summary>
    /// <param name="dirichletElementsDofs">Степени свободы граничных элементов, на которых заданы условия Дирихле</param>
    /// <param name="elementsDof">Степени свободы всех элементов сетки</param>
    /// <param name="dofCount">Общее количество степеней свободы</param>
    /// <returns>Количество незафиксированных узлов</returns>
    public static int RenumberDirichletDof(
        IEnumerable<IDofManager> dirichletElementsDofs,
        IEnumerable<IDofManager> elementsDof,
        int dofCount
    )
    {
        var dofToRenumber = new HashSet<int>();
        foreach (var elementDof in dirichletElementsDofs)
        {
            foreach (int dof in elementDof.Dof)
                dofToRenumber.Add(dof);
        }

        int start = 0;
        int finish = dofCount - 1;
        int[] renumber = new int[dofCount];

        for (int i = 0; i < dofCount; i++)
            if (dofToRenumber.Contains(i))
                renumber[i] = finish--;
            else
                renumber[i] = start++;

        foreach (var elementDof in elementsDof)
            elementDof.Renumber(i => renumber[i]);

        return start;
    }
}
