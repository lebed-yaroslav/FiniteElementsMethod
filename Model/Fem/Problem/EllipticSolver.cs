using Model.Core.CoordinateSystem;
using Model.Core.Matrix;
using Model.Core.Solver;
using Model.Fem.Assembly;
using Model.Fem.Integrator;
using Model.Fem.Mesh;
using Telma.Extensions;

namespace Model.Fem.Problem;


/// <summary>
/// Solves problem: -∇⋅(λ∇u) + γu = f
/// </summary>
/// <typeparam name="TSpace">Domain function space type</typeparam>
/// <typeparam name="TBoundary">Boundary function space type</typeparam>
/// <typeparam name="TOps">Matrix operations type for the function space</typeparam>
public sealed class EllipticSolver<TSpace, TBoundary, TOps>(
    IMatrixFactory matrixFactory,
    IIntegrator<TSpace, TBoundary, TOps> integrator,
    ISolver algebraicSolver
)
    where TSpace : IVectorBase<TSpace>
    where TBoundary : IVectorBase<TBoundary>
    where TOps : IMatrixOperations<TSpace, TSpace, TOps>
{
    private readonly IMatrixFactory _matrixFactory = matrixFactory;
    private readonly IIntegrator<TSpace, TBoundary, TOps> _integrator = integrator;
    private readonly ISolver _algebraicSolver = algebraicSolver;

    public StationarySolution<TSpace, TBoundary> Solve(
        EllipticProblem<TSpace> problem,
        ISolver.Params solverParams = new()
    )
    {
        const double time = 0.0; // As task is stationary time is zero
        var mesh = (IMeshWithBoundaries<TSpace, TBoundary>)problem.Mesh;
        var dofManager = DofManager.NumerateDof(mesh, problem.BoundaryConditions);
        var assembler = new Assembler<TSpace, TBoundary, TOps>(mesh, dofManager, _matrixFactory, _integrator);

        var globalMatrix = assembler.CreateGlobalMatrix();
        var rhsVector = assembler.CreateRhsVector();

        // 1. Assemble global matrix A = (M_ff + G_ff + MS3_ff) and add
        //    dirichlet contribution b = -(M_fd + G_fd + MS3_fd) * u_d
        assembler.CalculateFixedElements(problem.BoundaryConditions, time, _algebraicSolver, solverParams);
        assembler.CalculateStiffness(id => problem.Materials[id].Lambda, globalMatrix, rhsVector);
        assembler.CalculateMass(id => problem.Materials[id].Gamma, globalMatrix, rhsVector);
        assembler.CalculateRobinMassContribution(problem.BoundaryConditions, time, globalMatrix, rhsVector);

        // 2. Calculate rhs vector: b += f_ff + f_ff(Neumann)
        assembler.CalculateLoad(id => problem.Materials[id].Source, rhsVector);
        assembler.CalculateBoundaryLoadContribution(problem.BoundaryConditions, time, rhsVector);

        // 3. Solve SLAE
        var solution = new double[dofManager.TotalDofCount];
        _algebraicSolver.Matrix = globalMatrix;
        _algebraicSolver.Solve(
            rhsVector,
            solution.AsSpan()[..dofManager.FreeDofCount],
            solverParams
        );
        assembler.FixedSolution.CopyTo(solution.AsSpan()[dofManager.FreeDofCount..]);

        return new(
            mesh: mesh,
            coefficients: solution
        );
    }
}
