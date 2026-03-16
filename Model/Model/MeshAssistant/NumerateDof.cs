using Model.Model.Elements;

namespace Model.Model.MeshAssistant;

public static class Numerate
{
    public static (int, int) NumerateDof(this Mesh2D mesh)
    {

        int buf = 0, first = 0, counter = 0;

        #region Нумерация вершинных узлов
        //Подсчет кол-ва необходимых узлов на вершине.
        //Хранит максимальное кол-во узлов на вершине из всех смежных к этой вершине элементов.
        int[] numberDOFByVertex = new int[mesh.VerticesCount];
        foreach (var element in mesh.FiniteElements)
            foreach (var i in element.Vertices)
                numberDOFByVertex[i] = Math.Max(numberDOFByVertex[i], element.DOF.NumberOfDofOnVertex);

        for (int i = 0, sum = 0; i < mesh.VerticesCount; ++i)
        {
            sum += numberDOFByVertex[i];
            numberDOFByVertex[i] = buf;
            buf = sum;
        }

        //Нумерация вершинных узлов
        foreach (var element in mesh.FiniteElements)
            for (int i = 0; i < element.Vertices.Length; i++)
            {
                int globalVertex = element.Vertices[i];
                for (int j = 0; j < element.DOF.NumberOfDofOnVertex; ++j)
                    element.DOF.SetVertexDof(i, j, numberDOFByVertex[globalVertex] + j);
            }

        counter = buf;
        #endregion

        #region Нумерация рёберных узлов
        int localEdgeIndex = 0;

        //Подсчет кол-ва необходимых узлов на ребре.
        Dictionary<Edge, int> NumberDOFByEdge = new(); //Хранит кол-во узлов на ребре. Берет максимальное из двух элементов для ребра.
        foreach (var element in mesh.FiniteElements)
            foreach (var iter in element.Edges)
            {
                var edge = iter;
                if (edge.I > edge.J) edge = new Edge(edge.J, edge.I);
                if (!NumberDOFByEdge.TryGetValue(edge, out var NumDOF))
                    NumberDOFByEdge.Add(edge, element.DOF.NumberOfDofOnEdge);
                else
                    NumberDOFByEdge[edge] = Math.Max(NumDOF, element.DOF.NumberOfDofOnEdge);
            }

        //Нумерация реберных узлов.
        Dictionary<Edge, int> NumberByEdge = new(); //Номера ребер.
        foreach (var element in mesh.FiniteElements)
        {
            localEdgeIndex = 0;
            foreach (var iter in element.Edges)
            {
                var edge = iter;
                if (edge.I > edge.J) edge = new Edge(edge.J, edge.I);
                if (!NumberByEdge.TryGetValue(edge, out first))
                {
                    first = counter;
                    NumberByEdge.Add(edge, first);
                    counter += NumberDOFByEdge[edge];
                }
                for (int i = 0; i < element.DOF.NumberOfDofOnEdge; i++)
                    element.DOF.SetEdgeDof(localEdgeIndex++, IsOrientetionEdgeFliped(element.Edges, edge), i, first++);
            }
        }

        #endregion

        #region Нумерация элементных узлов
        //Нумерация элементных узлов.
        foreach (var element in mesh.FiniteElements)
            for (int i = 0; i < element.DOF.NumberOfDofOnElement; i++)
                element.DOF.SetElementDof(i, counter++);
        #endregion

        return (counter, counter);
    }

    private static bool IsOrientetionEdgeFliped(IEnumerable<Edge> edges, Edge edge)
    {
        bool isOrientationFlipped = true;
        foreach (var element in edges)
            if (element.I == edge.I && element.J == edge.J) isOrientationFlipped = false;
        return isOrientationFlipped;
    }

}
