using System.Diagnostics;
using Model.Fem.Elements;
using Model.Fem.Mesh;
using Model.Fem.Problem;
using Telma.Extensions;

namespace Model.Fem.Assembly;


public sealed class DofManager
{
    public int TotalDofCount { get; private init; }
    public int FreeDofCount { get; private init; }
    public int FixedDofCount => TotalDofCount - FreeDofCount;

    public static DofManager NumerateDof<TSpace, TBoundary>(
        IMeshWithBoundaries<TSpace, TBoundary> mesh,
        BoundaryCondition<TSpace>[] boundaryConditions
    )
        where TSpace : IVectorBase<TSpace>
        where TBoundary : IVectorBase<TBoundary>
    {
        int totalDofCount = DofNumerator<TSpace, TBoundary>.PreNumerateDof(mesh);
        int freeDofCount = DofNumerator<TSpace, TBoundary>.RenumberFixedDof(mesh, boundaryConditions, totalDofCount);

        return new() { TotalDofCount = totalDofCount, FreeDofCount = freeDofCount };
    }
}

public static class DofManagerApi
{
    extension(DofManager self)
    {
        public List<HashSet<int>> CreateFixedAdjacencyList<TSpace, TBoundary>(IMeshWithBoundaries<TSpace, TBoundary> mesh)
            where TSpace : IVectorBase<TSpace>
            where TBoundary : IVectorBase<TBoundary>
        => PortraitGenerator.CreateAdjacencyList(mesh.AllElementsDof, self.FreeDofCount, self.TotalDofCount - 1);

        public List<HashSet<int>> CreateFreeAdjacencyList<TSpace, TBoundary>(IMeshWithBoundaries<TSpace, TBoundary> mesh)
            where TSpace : IVectorBase<TSpace>
            where TBoundary : IVectorBase<TBoundary>
        => PortraitGenerator.CreateAdjacencyList(mesh.AllElementsDof, 0, self.FreeDofCount - 1);

        public List<HashSet<int>> CreateFullAdjacencyList<TSpace, TBoundary>(IMeshWithBoundaries<TSpace, TBoundary> mesh)
            where TSpace : IVectorBase<TSpace>
            where TBoundary : IVectorBase<TBoundary>
        => PortraitGenerator.CreateAdjacencyList(mesh.AllElementsDof, 0, self.TotalDofCount - 1);

        public double[] CreateFreeVector() => new double[self.FreeDofCount];
        public double[] CreateFixedVector() => new double[self.FixedDofCount];
        public double[] CreateFullVector() => new double[self.TotalDofCount];

        public bool TryGlobalToFixed(int globalDof, out int fixedDof)
        {
            fixedDof = globalDof - self.FreeDofCount;
            return fixedDof >= 0;
        }

        public bool TryGlobalToFree(int globalDof, out int freeDof)
        {
            freeDof = globalDof;
            return 0 <= freeDof && freeDof < self.FreeDofCount;
        }

        /// <summary>
        /// Create index mapping for free dof system, to use in
        /// <see cref="Core.Matrix.IGlobalMatrix.AddLocalMatrix(Core.Matrix.LocalMatrix, ReadOnlySpan{int})"/>.
        /// Negative indices corresponds to fixed dof
        /// </summary>
        /// <param name="elementDof">Element which indices is mapped</param>
        public int[] CreateFreeLocalToGlobalIndexMapping(IElementDof elementDof)
        {
            var indices = new int[elementDof.Count];
            self.CreateFreeLocalToGlobalIndexMapping(elementDof, indices);
            return indices;
        }

        /// <summary>
        /// Create index mapping for free dof system, to use in
        /// <see cref="Core.Matrix.IGlobalMatrix.AddLocalMatrix(Core.Matrix.LocalMatrix, ReadOnlySpan{int})"/>.
        /// </summary>
        /// <param name="elementDof">Element which indices is mapped</param>
        /// <param name="outIndices">Output parameter</param>
        public void CreateFreeLocalToGlobalIndexMapping(IElementDof elementDof, Span<int> outIndices)
        {
            Debug.Assert(outIndices.Length == elementDof.Count);
            for (int i = 0; i < elementDof.Count; ++i)
            {
                var dof = elementDof.Dof[i];
                outIndices[i] = (dof < self.FreeDofCount) ? dof : -(dof + 1); // IGlobalMatrix.AddLocalMatrix ignores negative indices
            }
        }

        /// <summary>
        /// Recovers fixed dof index that mapped by <see cref="CreateFreeLocalToGlobalIndexMapping(IElementDof, Span{int})"/>
        /// </summary>
        public int MappedFreeToFixed(int dofIndex)
        {
            Debug.Assert(dofIndex < 0 && -dofIndex >= self.FreeDofCount);
            return -dofIndex - 1 - self.FreeDofCount;
        }

        /// <summary>
        /// Create index mapping for fixed dof system, to use in
        /// <see cref="Core.Matrix.IGlobalMatrix.AddLocalMatrix(Core.Matrix.LocalMatrix, ReadOnlySpan{int})"/>.
        /// </summary>
        /// <param name="elementDof">Element which indices is mapped</param>
        /// <param name="outIndices">Output parameter</param>
        public void CreateFixedLocalToGlobalIndexMapping(IElementDof elementDof, Span<int> outIndices)
        {
            Debug.Assert(outIndices.Length == elementDof.Count);
            for (int i = 0; i < outIndices.Length; ++i)
                outIndices[i] = elementDof.Dof[i] - self.FreeDofCount; // IGlobalMatrix.AddLocalMatrix ignores negative indices
        }

        /// <summary>
        /// Recovers fixed dof index that mapped by <see cref="CreateFixedLocalToGlobalIndexMapping(IElementDof, Span{int})"/>
        /// </summary>
        public int MappedFixedToFree(int dofIndex)
        {
            Debug.Assert(dofIndex < 0 && -dofIndex < self.FreeDofCount);
            return dofIndex + self.FreeDofCount;
        }

        public Span<double> AsFreeSpan(Span<double> solution)
        {
            Debug.Assert(solution.Length == self.TotalDofCount);
            return solution[..self.FreeDofCount];
        }

        public Span<double> AsFixedSpan(Span<double> solution)
        {
            Debug.Assert(solution.Length == self.TotalDofCount);
            return solution[self.FreeDofCount..];
        }
    }
}

public static partial class DofNumerator<TSpace, TBoundary>
    where TSpace : IVectorBase<TSpace>
    where TBoundary : IVectorBase<TBoundary>
{
    /// <summary>
    /// Производит первичную нумерацию степеней свободы
    /// </summary>
    /// <returns>Общее количество степеней свободы</returns>
    public static int PreNumerateDof(IMeshWithBoundaries<TSpace, TBoundary> mesh)
    {
        int dofCount = 0;

        NumerateVertexDof(mesh, ref dofCount);
        NumerateEdgeDof(mesh, ref dofCount);
        NumerateElementDof(mesh, ref dofCount);

        return dofCount;
    }

    private static void NumerateVertexDof(IMeshWithBoundaries<TSpace, TBoundary> mesh, ref int dofCount)
    {
        // 1. Count dof per vertex
        int[] dofByVertex = new int[mesh.VertexCount]; // Stores max dof count for given vertex
        foreach (var element in mesh.AllElements)
            foreach (var i in element.Geometry.Vertices)
                dofByVertex[i] = Math.Max(dofByVertex[i], element.DOF.NumberOfDofOnVertex);

        // 2. Set up initial dof number per vertex
        for (int i = 0; i < mesh.VertexCount; ++i)
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


    private static void NumerateEdgeDof(IMeshWithBoundaries<TSpace, TBoundary> mesh, ref int dofCount)
    {
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
            int localEdgeIndex = 0;
            foreach (var edge in element.Geometry.Edges)
            {
                var sortedEdge = edge.Sorted(out var isOrientationFlipped);
                if (!dofNumberByEdge.TryGetValue(sortedEdge, out var dofIndex))
                {
                    dofIndex = dofCount;
                    dofNumberByEdge.Add(sortedEdge, dofIndex);
                    dofCount += dofCountByEdge[sortedEdge];
                }
                for (int i = 0; i < element.DOF.NumberOfDofOnEdge; i++)
                    element.DOF.SetEdgeDof(localEdgeIndex, isOrientationFlipped, i, dofIndex++);
                localEdgeIndex++;
            }
        }
    }

    private static void NumerateElementDof(IMeshWithBoundaries<TSpace, TBoundary> mesh, ref int dofCount)
    {
        foreach (var element in mesh.AllElements)
            for (int i = 0; i < element.DOF.NumberOfDofOnElement; i++)
                element.DOF.SetElementDof(i, dofCount++);
    }

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
        fixedElementsDof: mesh.FixedElementsBy(boundaryConditions).Select(e => e.DOF),
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
        IEnumerable<IElementDof> fixedElementsDof,
        IEnumerable<IElementDof> elementsDof,
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
