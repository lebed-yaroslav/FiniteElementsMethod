using Telma;
using Model.Model.Elements;

namespace Model.Model.MeshAssistant;

public static class Renumber
{
    public static int RenumberFirstCondition(this Mesh2D mesh, int DOFCount)
    {
        var DOFToRenumber = new HashSet<int>();
        foreach (var elem in mesh.BoundaryElements)
        {
            if (elem.BoundaryIndex == 1)
            {
                foreach (int dof in elem.DOF.Dof)
                    DOFToRenumber.Add(dof);
            }
        }

        int start = 0;
        int finish = DOFCount - 1;
        int[] renumber = new int[DOFCount];

        for (int i = 0; i < DOFCount; i++)
            if (DOFToRenumber.Contains(i))
                renumber[i] = finish--;
            else
                renumber[i] = start++;

        foreach (var elem in mesh.FiniteElements)
            elem.DOF.Renumber(i => renumber[i]);

        return start;
    }
}
