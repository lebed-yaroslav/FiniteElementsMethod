using System.Diagnostics;
using Model.Core.CoordinateSystem;
using Model.Core.Matrix;
using Model.Core.Solver;
using Model.Core.Util;
using Model.Fem.Assembly;
using Model.Fem.Integrator;
using Model.Fem.Mesh;
using Telma.Extensions;

namespace Model.Fem.Problem;

public readonly record struct TimeStepSolveInfo(
    double Time,
    double Residual,
    int Iterations,
    double StepSeconds
);


public sealed class ParabolicSolver<TSpace, TBoundary, TOps>(
    ITimeScheme[] timeSchemes,
    IMatrixFactory matrixFactory,
    IIntegrator<TSpace, TBoundary, TOps> integrator,
    ISolver algebraicSolver
)
    where TSpace : IVectorBase<TSpace>
    where TBoundary : IVectorBase<TBoundary>
    where TOps : IMatrixOperations<TSpace, TSpace, TOps>
{
    private readonly ITimeScheme[] _timeSchemes = timeSchemes;
    private readonly IMatrixFactory _matrixFactory = matrixFactory;
    private readonly IIntegrator<TSpace, TBoundary, TOps> _integrator = integrator;
    private readonly ISolver _algebraicSolver = algebraicSolver;

    public IEnumerable<StationarySolution<TSpace, TBoundary>> Solve(
        HyperbolicProblem<TSpace> problem,
        double[] timePoints,
        ISolver.Params solverParams = new(),
        ICollection<TimeStepSolveInfo>? outSolveStats = null
    )
    {
        DebugAssertSchemesAreCorrectTypes(_timeSchemes);

        var mesh = (IMeshWithBoundaries<TSpace, TBoundary>)problem.Mesh;
        var dofManager = DofManager.NumerateDof(mesh, problem.BoundaryConditions);
        var assembler = new Assembler<TSpace, TBoundary, TOps>(mesh, dofManager, _matrixFactory, _integrator);

        var globalMatrix = assembler.CreateGlobalMatrix(); // A_ff
        var rhsVector = dofManager.CreateFreeVector(); // b_ff

        var solutions = new SlidingWindow<double[]>(_timeSchemes[^1].SolutionLayerCount); // u
        solutions.Push(dofManager.CreateFullVector());
        assembler.CalculateInitialCondition(problem.InitialCondition, _algebraicSolver, solutions.Last, solverParams);

        var alpha = 0.0;
        var beta = 0.0;
        var gamma = new double[_timeSchemes[^1].SolutionLayerCount - 1];
        var delta = new double[_timeSchemes[^1].SolutionLayerCount - 1];

        void ApplyLocalStiffness(LocalMatrix local, ReadOnlySpan<int> indices, double dt)
        {
            // 1. History contributions: b += Sum(i, (γi)G  u_{n-i})
            var k = solutions.Count - 2;
            for (int i = 0; i <= k; ++i)
            {
                var oldSolution = solutions[k - i];
                local.AddMatVec(dofManager.AsFreeSpan(oldSolution), indices, rhsVector, scale: gamma[i]);
                assembler.AddFixedLoadContribution(local, indices, dofManager.AsFixedSpan(oldSolution), rhsVector, scale: -gamma[i]);
            }

            // 2. Global matrix: A += αG_ff, b -= αG_fd * u_d
            local *= alpha;
            globalMatrix.AddLocalMatrix(local, indices);
            assembler.AddFixedLoadContribution(local, indices, dofManager.AsFixedSpan(solutions.Last), rhsVector);
        }

        void ApplyLocalMass(LocalMatrix local, ReadOnlySpan<int> indices, double dt)
        {
            // 1. History contributions: b += Sum(i, (δi)M * u_{n-i})
            var k = solutions.Count - 2;
            for (int i = 0; i <= k; ++i)
            {
                var oldSolution = solutions[k - i];
                local.AddMatVec(dofManager.AsFreeSpan(oldSolution), indices, rhsVector, scale: delta[i]);
                assembler.AddFixedLoadContribution(local, indices, dofManager.AsFixedSpan(oldSolution), rhsVector, scale: -delta[i]);
            }

            // 2. Global matrix: A += βM_ff, b += -βM_fd * u_d
            local *= beta;
            globalMatrix.AddLocalMatrix(local, indices);
            assembler.AddFixedLoadContribution(local, indices, dofManager.AsFixedSpan(solutions.Last), rhsVector);
        }

        // globalMatrix += MS3_ff, And rhsVector += -MS3_fd * u_d
        void ApplyLocalMatrix(LocalMatrix local, ReadOnlySpan<int> indices)
        {
            globalMatrix.AddLocalMatrix(local, indices);
            assembler.AddFixedLoadContribution(local, indices, dofManager.AsFixedSpan(solutions.Last), rhsVector);
        }

        for (int i = 1; i < timePoints.Length; ++i)
        {

            // 0. Cleanup for next step
            solutions.CycleOrPushNew(dofManager.CreateFullVector);
            // Array.Fill(solutions.Last, 0) - Reuse old solution as initial guess
            Array.Fill(rhsVector, 0);
            globalMatrix.Fill(0);

            var time = timePoints[i];

            var currentScheme = _timeSchemes[int.Min(i, _timeSchemes.Length) - 1];

            // 1. Calculate time scheme coefficients
            var dt = time - timePoints[i - 1];
            alpha = currentScheme.GetStiffnessScale(dt);
            beta = currentScheme.GetMassScale(dt);
            currentScheme.GetHistoryStiffnessCoefficients(dt, gamma);
            currentScheme.GetHistoryMassCoefficients(dt, delta);
            var zeta = currentScheme.GetSourceScale(dt);

            // 2. Calculate fixed solution at current time (u_d)
            assembler.CalculateFixedElements(
                problem.BoundaryConditions, time,
                _algebraicSolver,
                dofManager.AsFixedSpan(solutions.Last),
                solverParams
            );

            // 3. Assemble main part: (αG + βM)u_n = ζf + Sum(i, [(γi)G +  (δi)M] * u_{n-i})
            assembler.CalculateStiffness(
                id => problem.Lambda(id, time),
                (local, indices) => ApplyLocalStiffness(local, indices, dt)
            );
            assembler.CalculateMass(
                id => problem.Sigma(id, time),
                (local, indices) => ApplyLocalMass(local, indices, dt)
            );
            assembler.CalculateLoad(id => problem.Source(id, time), rhsVector, scale: zeta);

            // 4. Calculate boundary condition contribution
            assembler.CalculateRobinMassContribution(
                problem.BoundaryConditions, time,
                ApplyLocalMatrix
            );
            assembler.CalculateBoundaryLoadContribution(problem.BoundaryConditions, time, rhsVector);

            // 5. Solve system
            _algebraicSolver.Matrix = globalMatrix;
            var sw = Stopwatch.StartNew();
            var (residual, iterations) = _algebraicSolver.Solve(
                rhsVector,
                dofManager.AsFreeSpan(solutions.Last),
                solverParams
            );
            sw.Stop();
            outSolveStats?.Add(new TimeStepSolveInfo(time, residual, iterations, sw.Elapsed.TotalSeconds));

            yield return new(mesh, [.. solutions.Last]);
        }

        yield break;
    }

    [Conditional("DEBUG")]
    private static void DebugAssertSchemesAreCorrectTypes(ITimeScheme[] schemes)
    {
        for (int i = 0; i < schemes.Length; ++i)
            Debug.Assert(schemes[i].SolutionLayerCount == i + 2);
    }
}
