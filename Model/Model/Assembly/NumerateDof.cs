using Model.Model.Elements;
using Model.Model.Mesh;

namespace Model.Model.Assembly;

public static partial class DofNumerator
{
    /// <summary>
    /// Производит первичную нумерацию степеней свободы
    /// </summary>
    /// <returns>Общее количество степеней свободы</returns>
    public static int NumerateDof(Mesh2D mesh)
    {

        int dofCount = 0;

        NumerateVertexDof(mesh, ref dofCount);
        NumerateEdgeDof(mesh, ref dofCount);
        NumerateElementDof(mesh, ref dofCount);

        return dofCount;
    }

    private static void NumerateVertexDof(Mesh2D mesh, ref int dofCount)
    {
        // 1. Count dof per vertex
        int[] dofByVertex = new int[mesh.VerticesCount]; // Stores max dof count for given vertex
        foreach (var element in mesh.AllElements)
            foreach (var i in element.Geometry.Vertices)
                dofByVertex[i] = Math.Max(dofByVertex[i], element.DOF.NumberOfDofOnVertex);

        // 2. Set up initial dof number per vertex
        for (int i = 0; i < mesh.VerticesCount; ++i)
        {
            int prevDofCount = dofCount;
            dofCount += dofByVertex[i];
            dofByVertex[i] = prevDofCount;
        }

        // 3. Numerate vertex dof
        foreach (var element in mesh.AllElements)
            for (int i = 0; i < element.Geometry.VertexCount; i++)
            {
                int globalVertex = element.Geometry.Vertices[i];
                for (int j = 0; j < element.DOF.NumberOfDofOnVertex; ++j)
                    element.DOF.SetVertexDof(i, j, dofByVertex[globalVertex] + j);
            }
    }


    private static void NumerateEdgeDof(Mesh2D mesh, ref int dofCount)
    {
        int localEdgeIndex = 0;

        // 1. Count dof per edge
        Dictionary<Edge, int> dofCountByEdge = []; // Stores max dof count for given edge
        foreach (var element in mesh.AllElements)
            foreach (var iter in element.Geometry.Edges)
            {
                var edge = iter.Sorted();
                if (!dofCountByEdge.TryGetValue(edge, out var dofNum))
                    dofCountByEdge.Add(edge, element.DOF.NumberOfDofOnEdge);
                else
                    dofCountByEdge[edge] = Math.Max(dofNum, element.DOF.NumberOfDofOnEdge);
            }

        // 2. Numerate edge dof
        Dictionary<Edge, int> dofNumberByEdge = [];
        foreach (var element in mesh.AllElements)
        {
            localEdgeIndex = 0;
            foreach (var iter in element.Geometry.Edges)
            {
                var edge = iter.Sorted(out var isOrientationFlipped);
                if (!dofNumberByEdge.TryGetValue(edge, out var first))
                {
                    first = dofCount;
                    dofNumberByEdge.Add(edge, first);
                    dofCount += dofCountByEdge[edge];
                }
                for (int i = 0; i < element.DOF.NumberOfDofOnEdge; i++)
                    element.DOF.SetEdgeDof(localEdgeIndex++, isOrientationFlipped, i, first++);
            }
        }
    }


    private static void NumerateElementDof(Mesh2D mesh, ref int dofCount)
    {
        foreach (var element in mesh.AllElements)
            for (int i = 0; i < element.DOF.NumberOfDofOnElement; i++)
                element.DOF.SetElementDof(i, dofCount++);
    }
}
