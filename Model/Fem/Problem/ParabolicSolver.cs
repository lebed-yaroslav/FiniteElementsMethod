using Model.Core.CoordinateSystem;
using Model.Core.Matrix;
using Model.Core.Solver;
using Model.Core.Util;
using Model.Fem.Assembly;
using Model.Fem.Integrator;
using Model.Fem.Mesh;
using Telma.Extensions;

namespace Model.Fem.Problem;


public sealed class ParabolicSolver<TSpace, TBoundary, TOps>(
    ITimeScheme timeScheme,
    IMatrixFactory matrixFactory,
    IIntegrator<TSpace, TBoundary, TOps> integrator,
    ISolver algebraicSolver
)
    where TSpace : IVectorBase<TSpace>
    where TBoundary : IVectorBase<TBoundary>
    where TOps : IMatrixOperations<TSpace, TSpace, TOps>
{
    private readonly ITimeScheme _timeScheme = timeScheme;
    private readonly IMatrixFactory _matrixFactory = matrixFactory;
    private readonly IIntegrator<TSpace, TBoundary, TOps> _integrator = integrator;
    private readonly ISolver _algebraicSolver = algebraicSolver;

    public IEnumerable<StationarySolution<TSpace, TBoundary>> Solve(
        HyperbolicProblem<TSpace> problem,
        double[] timePoints,
        ISolver.Params solverParams = new()
    )
    {
        var mesh = (IMeshWithBoundaries<TSpace, TBoundary>)problem.Mesh;
        var dofManager = DofManager.NumerateDof(mesh, problem.BoundaryConditions);
        var assembler = new Assembler<TSpace, TBoundary, TOps>(mesh, dofManager, _matrixFactory, _integrator);

        int dofCount = dofManager.TotalDofCount;
        int freeDofCount = dofManager.FreeDofCount;
        int fixedDofCount = dofManager.FixedDofCount;

        var solutions = new SlidingWindow<double[]>(_timeScheme.SolutionLayerCount);
        solutions.Push(new double[dofCount]);

        var globalMatrix = assembler.CreateGlobalMatrix(); // A_ff
        var rhsVector = new double[freeDofCount]; // b_ff

        var alpha = 0.0;
        var beta = 0.0;
        var gamma = new double[_timeScheme.SolutionLayerCount - 1];
        var delta = new double[_timeScheme.SolutionLayerCount - 1];

        void ApplyLocalStiffness(LocalMatrix local, ReadOnlySpan<int> indices, double dt)
        {
            // 1. History contributions: b += Sum(i, (γi)G * u_{n-i})
            var k = solutions.Count - 2;
            for (int i = 0; i <= k; ++i)
            {
                local.AddMatVec(dofManager.AsFreeSpan(solutions[k - i]), indices, rhsVector, scale: gamma[i]);
                assembler.AddFixedLoadContribution(local, indices, dofManager.AsFixedSpan(solutions[k - i]), rhsVector, scale: -gamma[i]);
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
                local.AddMatVec(dofManager.AsFreeSpan(solutions[k - i]), indices, rhsVector, scale: delta[i]);
                assembler.AddFixedLoadContribution(local, indices, dofManager.AsFixedSpan(solutions[k - i]), rhsVector, scale: -delta[i]);
            }

            // 2. Global matrix: A += βM_ff, b += -βM_fd * u_d
            local *= beta;
            globalMatrix.AddLocalMatrix(local, indices);
            assembler.AddFixedLoadContribution(local, indices, dofManager.AsFixedSpan(solutions.Last), rhsVector, scale: beta);
        }

        // globalMatrix += MS3_ff, And rhsVector += -MS3_fd * u_d
        void ApplyLocalMatrix(LocalMatrix local, ReadOnlySpan<int> indices)
        {
            globalMatrix.AddLocalMatrix(local, indices);
            assembler.AddFixedLoadContribution(local, indices, dofManager.AsFixedSpan(solutions.Last), rhsVector);
        }

        for (int i = 1; i < timePoints.Length; ++i)
        {
            var time = timePoints[i];

            // 0. Calculate time scheme coefficients
            var dt = (i > 0) ? time - timePoints[i - 1] : 1;
            alpha = _timeScheme.GetStiffnessScale(dt);
            beta = _timeScheme.GetMassScale(dt);
            _timeScheme.GetHistoryStiffnessCoefficients(dt, gamma);
            _timeScheme.GetHistoryMassCoefficients(dt, delta);
            var zeta = _timeScheme.GetSourceScale(dt);

            // 1. Calculate fixed solution at current time (u_d)
            assembler.CalculateFixedElements(
                problem.BoundaryConditions, time,
                _algebraicSolver,
                dofManager.AsFixedSpan(solutions.Last),
                solverParams
            );

            // 2. Assemble main part: (αG + βM)u_n = ζf + Sum(i, [(γi)G +  (δi)M] * u_{n-i})
            assembler.CalculateStiffness(
                id => problem.Lambda(id, time),
                (local, indices) => ApplyLocalStiffness(local, indices, dt)
            );
            assembler.CalculateMass(
                id => problem.Sigma(id, time),
                (local, indices) => ApplyLocalMass(local, indices, dt)
            );
            assembler.CalculateLoad(id => problem.Source(id, time), rhsVector, scale: zeta);

            // 3. Calculate boundary condition contribution
            assembler.CalculateRobinMassContribution(
                problem.BoundaryConditions, time,
                ApplyLocalMatrix
            );
            assembler.CalculateBoundaryLoadContribution(problem.BoundaryConditions, time, rhsVector);

            // 4. Solve system
            _algebraicSolver.Matrix = globalMatrix;
            _algebraicSolver.Solve(
                rhsVector,
                dofManager.AsFreeSpan(solutions.Last),
                solverParams
            );

            yield return new(mesh, [.. solutions.Last]);

            // 5. Cleanup for next step
            solutions.CycleOrPushNew(() => new double[dofCount]);
            // Array.Fill(solutions.Last, 0) - Reuse old solution as initial guess
            Array.Fill(rhsVector, 0);
            globalMatrix.Fill(0);
        }

        yield break;
    }
}
