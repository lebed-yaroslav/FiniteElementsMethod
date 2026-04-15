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
    private readonly double[] _fixedSolution = new double[DofManager.FixedDofCount];

    private readonly List<HashSet<int>> _adjacencyList = PortraitGenerator.CreateAdjacencyList(
        Mesh.AllElementsDof, minDofIndex: 0, maxDofIndex: DofManager.FreeDofCount - 1
    );

    public ReadOnlySpan<double> FixedSolution => _fixedSolution;

    public IGlobalMatrix CreateGlobalMatrix() => MatrixFactory.Create(_adjacencyList);
    public double[] CreateRhsVector() => new double[DofManager.FreeDofCount];


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
    /// After solving, u_d is stored in <see cref="FixedSolution"/>.
    /// </remarks>
    public void CalculateFixedElements(
        BoundaryCondition<TSpace>[] boundaryConditions, 
        double time, ISolver solver, 
        ISolver.Params paramz = default)
    {
        Array.Fill(_fixedSolution, 0);

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
        solver.Solve(rhsVector, _fixedSolution, paramz);
    }

    /// <summary>
    /// Assembles the stiffness matrix contribution: G_ij = ∫_Ω λ(∇φ_i)·(∇φ_j) dΩ.
    /// </summary>
    /// <param name="lambdaByMaterialIndex">
    /// Function returning the material property λ(x) for a given material index.
    /// Property is defined in mesh-space coordinates.
    /// </param>
    /// <remarks>
    /// The local element stiffness matrices are:
    /// <list type="bullet">
    /// <item>Added to the global matrix <see cref="Matrix"/> (A_ff part)</item>
    /// <item>Used to compute -A_fd·u_d contributions to <see cref="RhsVector"/> from fixed DOFs</item>
    /// </list>
    /// </remarks>
    public void CalculateStiffness(
        Func<int, Func<TSpace, double>> lambdaByMaterialIndex,
        IGlobalMatrix outStiffness,
        Span<double> outLoad = default
    )
    {
        foreach (var element in Mesh.FiniteElements)
        {
            var lambda = lambdaByMaterialIndex(element.MaterialIndex);
            var local = Integrator.CalculateLocalStiffness(element, lambda);
            AddLocalMatrixWithFixedLoadContribution(local, element.DOF, outStiffness, outLoad);
        }
    }

    /// <summary>
    /// Assembles the mass matrix contribution: M_ij = ∫_Ω γ φ_i φ_j dΩ.
    /// </summary>
    /// <param name="gammaByMaterialIndex">
    /// Function returning the material property γ(x) for a given material index.
    /// Property is defined in mesh-space coordinates.
    /// </param>
    /// <remarks>
    /// The local element mass matrices are:
    /// <list type="bullet">
    /// <item>Added to the global matrix <see cref="Matrix"/> (A_ff part)</item>
    /// <item>Used to compute -A_fd·u_d contributions to <see cref="RhsVector"/> from fixed DOFs</item>
    /// </list>
    /// </remarks>
    public void CalculateMass(
        Func<int, Func<TSpace, double>> gammaByMaterialIndex,
        IGlobalMatrix outMass,
        Span<double> outLoad = default
    )
    {
        foreach (var element in Mesh.FiniteElements)
        {
            var gamma = gammaByMaterialIndex(element.MaterialIndex);
            var local = Integrator.CalculateLocalMass(element, gamma);
            AddLocalMatrixWithFixedLoadContribution(local, element.DOF, outMass, outLoad);
        }
    }

    /// <summary>
    /// Assembles the domain load vector: f_i = ∫_Ω s φ_i dΩ.
    /// </summary>
    /// <param name="sourceByMaterialIndex">
    /// Function returning the source term s(x) for a given material index.
    /// Function is defined in mesh-space coordinates.
    /// </param>
    /// <remarks>Local load vectors adds up to <see cref="RhsVector"/></remarks>
    public void CalculateLoad(
        Func<int, Func<TSpace, double>> sourceByMaterialIndex,
        Span<double> outLoad
    )
    {
        foreach (var element in Mesh.FiniteElements)
        {
            var source = sourceByMaterialIndex(element.MaterialIndex);
            var load = new double[element.DOF.Count];
            Integrator.CalculateLocalLoad(element, source, load);
            AddLocalLoad(load, element.DOF, outLoad);
        }
    }

    /// <summary>
    /// Assembles boundary load contributions from Neumann and Robin conditions.
    /// </summary>
    /// <param name="boundaryConditions">
    /// Array of boundary conditions indexed by <see cref="IBoundaryElement.BoundaryIndex"/>.
    /// Processes <see cref="BoundaryCondition{TSpace}.Neumann"/> and <see cref="BoundaryCondition{TSpace}.Robin"/>.
    /// </param>
    /// <param name="time">Current simulation time, passed to boundary condition evaluation.</param>
    /// <remarks>
    /// Adds contributions do <see cref="RhsVector"/>:
    /// <list type="bullet">
    /// <item>Neumann: f_i = ∫_S2 θ φ_i dS</item>
    /// <item>Robin: f_i = ∫_S3 β·u_β φ_i dS</item>
    /// </list>
    /// </remarks>
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

    /// <summary>
    /// Assembles the Robin boundary mass matrix: MS3_ij = ∫_S3 β φ_i φ_j dS.
    /// </summary>
    /// <param name="boundaryConditions">
    /// Array of boundary conditions indexed by <see cref="IBoundaryElement.BoundaryIndex"/>.
    /// Only <see cref="BoundaryCondition{TSpace}.Robin"/> conditions are processed.
    /// </param>
    /// <param name="time">Current simulation time, passed to β evaluation.</param>
    /// <remarks>
    /// The local element mass matrices are:
    /// <list type="bullet">
    /// <item>Added to the global matrix <see cref="Matrix"/> (A_ff part)</item>
    /// <item>Used to compute -A_fd·u_d contributions to <see cref="RhsVector"/> from fixed DOFs</item>
    /// </list>
    /// </remarks>
    public void CalculateRobinMassContribution(
        BoundaryCondition<TSpace>[] boundaryConditions,
        double time,
        IGlobalMatrix outMass3,
        Span<double> outLoad = default
    )
    {
        foreach (var element in Mesh.BoundaryElements)
        {
            if (boundaryConditions[element.BoundaryIndex] is not BoundaryCondition<TSpace>.Robin(var beta, var _))
                continue;
            var local = Integrator.CalculateLocalMass(element, p => beta(p, time));
            AddLocalMatrixWithFixedLoadContribution(local, element.DOF, outMass3, outLoad);
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

    private void AddLocalMatrixWithFixedLoadContribution(
        LocalMatrix local,
        IDofManager elementDof,
        IGlobalMatrix outMatrix,
        Span<double> outLoad = default
    )
    {
        var indices = DofManager.CreateFreeLocalToGlobalIndexMapping(elementDof);
        outMatrix.AddLocalMatrix(local, indices);

        if (!outLoad.IsEmpty)
            AddFixedLoadContribution(local, elementDof, indices, FixedSolution, outLoad); ;
    }

    private void AddFixedLoadContribution(
        LocalMatrix local,
        IDofManager elementDof,
        ReadOnlySpan<int> indices,
        ReadOnlySpan<double> fixedSolution,
        Span<double> outLoad
    )
    {
        Debug.Assert(outLoad.Length == DofManager.FreeDofCount);
        for (int i = 0; i < elementDof.Count; ++i)
        {
            var globalDofI = elementDof.Dof[i];
            if (globalDofI >= DofManager.FreeDofCount) continue; // Skip fixed dof row
            for (int j = 0; j < elementDof.Count; ++j)
            {
                var fixedDofJ = elementDof.Dof[j] - DofManager.FreeDofCount;
                if (fixedDofJ < 0) continue; // Skip free dof column
                outLoad[globalDofI] -= local[i, j] * _fixedSolution[fixedDofJ];
            }
        }
    }

    private void AddLocalLoad(
        ReadOnlySpan<double> local,
        IDofManager elementDof,
        Span<double> outLoad
    )
    {
        Debug.Assert(outLoad.Length == DofManager.FreeDofCount);

        for (int i = 0; i < elementDof.Count; ++i)
        {
            int dof = elementDof.Dof[i];
            if (dof < DofManager.FreeDofCount) // Ignore fixed dof
                outLoad[dof] += local[i];
        }
    }
}
