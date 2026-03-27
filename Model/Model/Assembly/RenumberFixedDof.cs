using Model.Model.Elements;
using Model.Model.Mesh;
using Telma.Extensions;

namespace Model.Model.Assembly;


public static partial class DofNumerator<TSpace, TBoundary>
    where TSpace : IVectorBase<TSpace>
    where TBoundary : IVectorBase<TBoundary>
{
    /// <summary>
    /// Производит перенумерацию степеней свободы таким образом,
    /// что все зафиксированные условиями Дирихле узлы оказываются в конце
    /// </summary>
    /// <param name="totalDofCount">Общее количество степеней свободы</param>
    /// <returns>Количество свободных узлов</returns>
    public static int RenumberFixedDof(
        IMeshWithBoundaries<TSpace, TBoundary> mesh,
        BoundaryCondition<TSpace>[] boundaryConditions,
        int totalDofCount
    ) => RenumberFixedDof(
        fixedElementsDof: mesh.BoundaryElements
            .Where(e => boundaryConditions[e.BoundaryIndex] is BoundaryCondition<TSpace>.Dirichlet)
            .Select(e => e.DOF),
        elementsDof: mesh.AllElementsDof,
        totalDofCount: totalDofCount
    );

    /// <summary>
    /// Производит перенумерацию степеней свободы таким образом,
    /// что все зафиксированные условиями Дирихле узлы оказываются в конце
    /// </summary>
    /// <param name="fixedElementsDof">Степени свободы граничных элементов, на которых заданы условия Дирихле</param>
    /// <param name="elementsDof">Степени свободы всех элементов сетки</param>
    /// <param name="totalDofCount">Общее количество степеней свободы</param>
    /// <returns>Количество свободных узлов</returns>
    public static int RenumberFixedDof(
        IEnumerable<IDofManager> fixedElementsDof,
        IEnumerable<IDofManager> elementsDof,
        int totalDofCount
    )
    {
        var fixedDof = new HashSet<int>();
        foreach (var elementDof in fixedElementsDof)
        {
            foreach (int dof in elementDof.Dof)
                fixedDof.Add(dof);
        }

        int start = 0;
        int finish = totalDofCount - 1;
        int[] numberMapping = new int[totalDofCount];

        for (int i = 0; i < totalDofCount; i++)
            if (fixedDof.Contains(i))
                numberMapping[i] = finish--;
            else
                numberMapping[i] = start++;

        foreach (var elementDof in elementsDof)
            elementDof.Renumber(i => numberMapping[i]);

        return start;
    }
}
