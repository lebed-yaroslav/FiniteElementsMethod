using System.Diagnostics;
using Model.Core.CoordinateSystem;
using Model.Core.Matrix;
using Model.Core.Solver;
using Model.Fem.Elements;
using Model.Fem.Integrator;
using Model.Fem.Mesh;
using Model.Fem.Problem;
using Telma.Extensions;

namespace Model.Fem.Assembly;


/// <summary>
/// Assembles the global finite element system: A·u = f, with support for Dirichlet, Neumann, and Robin boundary conditions.
/// Handles the separation between free and fixed degrees of freedom, where fixed DOFs are eliminated from the main system
/// and their contributions are moved to the right-hand side.
/// </summary>
/// <typeparam name="TSpace">Domain function space type</typeparam>
/// <typeparam name="TBoundary">Boundary function space type</typeparam>
/// <typeparam name="TOps">Matrix operations type for the function space</typeparam>
/// <remarks>
/// The assembler builds the global system:
/// <code>
/// [A_ff  A_fd] [q_f] = [q_f]
/// [A_df  A_dd] [q_d]   [q_d]
/// </code>
/// where subscript 'f' denotes free DOFs and 'd' denotes fixed (Dirichlet) DOFs.
/// The fixed DOFs are solved separately, and their contributions are added to the free system:
/// <code>
/// A_ff·u_f = f_f - A_fd·u_d
/// </code>
/// </remarks>
public sealed record Assembler<TSpace, TBoundary, TOps>(
    IMeshWithBoundaries<TSpace, TBoundary> Mesh,
    DofManager DofManager,
    IMatrixFactory MatrixFactory,
    IIntegrator<TSpace, TBoundary, TOps> Integrator
)
    where TSpace : IVectorBase<TSpace>
    where TBoundary : IVectorBase<TBoundary>
    where TOps : IMatrixOperations<TSpace, TSpace, TOps>
{
    private readonly List<HashSet<int>> _adjacencyList = PortraitGenerator.CreateAdjacencyList(
        Mesh.AllElementsDof, minDofIndex: 0, maxDofIndex: DofManager.FreeDofCount - 1
    );

    public IGlobalMatrix CreateGlobalMatrix() => MatrixFactory.Create(_adjacencyList);
    public double[] CreateRhsVector() => new double[DofManager.FreeDofCount];
    public double[] CreateFixedSolutionVector() => new double[DofManager.FixedDofCount];

    /// <summary>
    /// Solves for the fixed (Dirichlet) degrees of freedom by assembling and solving a separate system.
    /// </summary>
    /// <param name="boundaryConditions">
    /// Array of boundary conditions indexed by <see cref="IBoundaryElement.BoundaryIndex"/>.
    /// Only <see cref="BoundaryCondition{TSpace}.Dirichlet"/> conditions are processed.
    /// </param>
    /// <param name="time">Current simulation time, passed to Dirichlet condition evaluation.</param>
    /// <param name="solver">Solver instance used to solve the fixed DOF system.</param>
    /// <param name="paramz">Optional solver parameters.</param>
    /// <remarks>
    /// The fixed DOF system solves:
    /// <code>
    /// A_dd·u_d = f_d
    /// </code>
    /// where A_dd is assembled from Dirichlet boundary elements and f_d comes from the Dirichlet values.
    /// After solving, u_d is stored in <paramref name="outFixedSolution"/>.
    /// </remarks>
    public void CalculateFixedElements(
        BoundaryCondition<TSpace>[] boundaryConditions, 
        double time, ISolver solver, 
        Span<double> outFixedSolution,
        ISolver.Params paramz = default
    )
    {
        Debug.Assert(outFixedSolution.Length == DofManager.FixedDofCount);
        outFixedSolution.Fill(0);

        var adjacencyList = PortraitGenerator.CreateAdjacencyList(
            Mesh.AllElementsDof,
            minDofIndex: DofManager.FreeDofCount,
            maxDofIndex: DofManager.TotalDofCount - 1
        );

        var matrix = MatrixFactory.Create(adjacencyList);
        var rhsVector = new double[DofManager.FixedDofCount];

        foreach (var element in Mesh.BoundaryElements)
        {
            if (boundaryConditions[element.BoundaryIndex] is not BoundaryCondition<TSpace>.Dirichlet bc)
                continue;

            CalculateFixedElementMass(element, matrix);
            CalculateFixedElementLoad(element, boundaryValue: p => bc.Value(p, time), rhsVector);
        }

        solver.Matrix = matrix;
        solver.Solve(rhsVector, outFixedSolution, paramz);
    }

    public void CalculateStiffness(
        Func<int, Func<TSpace, double>> lambdaByMaterialIndex,
        Action<LocalMatrix, ReadOnlySpan<int>> callback
    )
    {
        foreach (var element in Mesh.FiniteElements)
        {
            var lambda = lambdaByMaterialIndex(element.MaterialIndex);
            var local = Integrator.CalculateLocalStiffness(element, lambda);
            var indices = DofManager.CreateFreeLocalToGlobalIndexMapping(element.DOF);
            callback(local, indices);
        }
    }

    public void CalculateMass(
        Func<int, Func<TSpace, double>> gammaByMaterialIndex,
        Action<LocalMatrix, ReadOnlySpan<int>> callback
    )
    {
        foreach (var element in Mesh.FiniteElements)
        {
            var gamma = gammaByMaterialIndex(element.MaterialIndex);
            var local = Integrator.CalculateLocalMass(element, gamma);
            var indices = DofManager.CreateFreeLocalToGlobalIndexMapping(element.DOF);
            callback(local, indices);
        }
    }

    public void CalculateLoad(
        Func<int, Func<TSpace, double>> sourceByMaterialIndex,
        Span<double> outLoad,
        double scale = 1.0
    )
    {
        foreach (var element in Mesh.FiniteElements)
        {
            var source = sourceByMaterialIndex(element.MaterialIndex);
            var load = new double[element.DOF.Count];
            Integrator.CalculateLocalLoad(element, source, load);
            AddLocalLoad(load, element.DOF, outLoad, scale);
        }
    }

    public void CalculateBoundaryLoadContribution(
        BoundaryCondition<TSpace>[] boundaryConditions,
        double time,
        Span<double> outLoad
    )
    {
        foreach (var element in Mesh.BoundaryElements)
        {
            switch (boundaryConditions[element.BoundaryIndex])
            {
                case BoundaryCondition<TSpace>.Neumann(var flux):
                    var load2 = new double[element.DOF.Count];
                    Integrator.CalculateLocalLoad(element, p => flux(p, time), load2);
                    AddLocalLoad(load2, element.DOF, outLoad);
                    break;
                case BoundaryCondition<TSpace>.Robin(var beta, var uBeta):
                    var load3 = new double[element.DOF.Count];
                    Integrator.CalculateLocalLoad(element, p => beta(p, time) * uBeta(p, time), load3);
                    AddLocalLoad(load3, element.DOF, outLoad);
                    break;
            }
        }
    }

    public void CalculateRobinMassContribution(
        BoundaryCondition<TSpace>[] boundaryConditions,
        double time,
        Action<LocalMatrix, ReadOnlySpan<int>> callback
    )
    {
        foreach (var element in Mesh.BoundaryElements)
        {
            if (boundaryConditions[element.BoundaryIndex] is not BoundaryCondition<TSpace>.Robin(var beta, var _))
                continue;
            var local = Integrator.CalculateLocalMass(element, p => beta(p, time));
            var indices = DofManager.CreateFreeLocalToGlobalIndexMapping(element.DOF);
            callback(local, indices);
        }
    }

    private void CalculateFixedElementMass(IBoundaryElement<TSpace, TBoundary> element, IGlobalMatrix matrix)
    {
        var mass = Integrator.CalculateLocalMass(element, _ => 1.0);
        var indices = new int[element.DOF.Count];
        DofManager.CreateFixedLocalToGlobalIndexMapping(element.DOF, indices);
        matrix.AddLocalMatrix(mass, indices);
    }

    private void CalculateFixedElementLoad(
        IBoundaryElement<TSpace, TBoundary> element,
        Func<TSpace, double> boundaryValue,
        Span<double> rhsVector
    )
    {
        int n = element.DOF.Count;
        var load = new double[n];
        Integrator.CalculateLocalLoad(element, boundaryValue, load);

        for (int i = 0; i < n; ++i)
        {
            int gi = element.DOF.Dof[i] - DofManager.FreeDofCount;
            if (gi >= 0) rhsVector[gi] += load[i]; // Ignore free dof
        }
    }

    public void AddFixedLoadContribution(
        LocalMatrix local,
        ReadOnlySpan<int> indices,
        ReadOnlySpan<double> fixedSolution,
        Span<double> outLoad,
        double scale = 1.0
    )
    {
        Debug.Assert(outLoad.Length == DofManager.FreeDofCount);
        for (int i = 0; i < local.Size; ++i)
        {
            var globalDofI = indices[i]; // Negative indices correpsonds to fixed dof
            if (globalDofI < 0) continue; // Skip fixed dof row
            for (int j = 0; j < local.Size; ++j)
            {
                if (indices[j] >= 0) continue; // Skip free dof column
                var fixedDofJ = DofManager.MappedFreeToFixed(indices[j]);
                outLoad[globalDofI] -= scale * local[i, j] * fixedSolution[fixedDofJ];
            }
        }
    }

    private void AddLocalLoad(
        ReadOnlySpan<double> local,
        IDofManager elementDof,
        Span<double> outLoad,
        double scale = 1.0
    )
    {
        Debug.Assert(outLoad.Length == DofManager.FreeDofCount);

        for (int i = 0; i < elementDof.Count; ++i)
        {
            int dof = elementDof.Dof[i];
            if (dof < DofManager.FreeDofCount) // Ignore fixed dof
                outLoad[dof] += scale * local[i];
        }
    }
}
