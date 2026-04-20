using System.Diagnostics;
using Model.Core.CoordinateSystem;
using Model.Core.Matrix;
using Model.Core.Solver;
using Model.Core.Util;
using Model.Core.Vector;
using Model.Fem.Assembly;
using Model.Fem.Integrator;
using Model.Fem.Mesh;
using Telma.Extensions;

namespace Model.Fem.Problem;


public sealed class ParabolicSolver_<TSpace, TBoundary, TOps>(
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

    public IEnumerable<StationarySolution<TSpace, TBoundary>> Solve(
        HyperbolicProblem_<TSpace> problem,
        ITimeScheme[] timeSchemes,
        ISolver.Params solverParams = new()
    )
    {
        DebugAssertSchemesCanBeUsed(problem, timeSchemes);

        var mesh = (IMesh<TSpace, TBoundary>)problem.Mesh;
        var dofManager = DofManager.NumerateDof(mesh, problem.BoundaryConditions);
        var assembler = new Assembler<TSpace, TBoundary, TOps>(mesh, dofManager, _matrixFactory, _integrator);

        int initialConditions = problem.InitialConditions.Length;
        int timeLayers = timeSchemes.Max(s => s.Layers);
        var t = problem.TimePoints;

        var solution = new StationarySolution<TSpace, TBoundary>(mesh, []);
        solution.BuildSearchTree();

        var solutions = new SlidingWindow<double[]>(timeLayers); // u_{n-i}
        var loadVectors = new SlidingWindow<double[]>(timeLayers); // b_ff{n-i}

        for (int i = 0; i < initialConditions; ++i)
        {
            solutions.Push(dofManager.CreateFullVector());
            assembler.ProjectToSolutionSpace(problem.InitialConditions[i], _algebraicSolver, solutions.Last, solverParams);

            loadVectors.Push(dofManager.CreateFreeVector());
            assembler.CalculateLoad(id => problem.Source(id, t[i]), loadVectors.Last);
            assembler.CalculateBoundaryLoadContribution(problem.BoundaryConditions, t[i], loadVectors.Last);

            yield return solution.WithCoefficients([.. solutions.Last]);
        }

        var globalMatrix = assembler.CreateGlobalMatrix(); // A_ff
        var rhsVector = dofManager.CreateFreeVector();

        void ApplyLocalMatrix(LocalMatrix local, ReadOnlySpan<int> indices, ReadOnlySpan<double> coefficients)
        {
            // 1. History contributions: b -= ∑ [i=1,k | (ci)A⋅u_{n-i}]
            for (int i = 1; i < coefficients.Length; ++i)
            {
                if (coefficients[i] == 0) continue;
                var oldSolution = solutions[^(i + 1)];
                local.AddMatVec(dofManager.AsFreeSpan(oldSolution), indices, rhsVector, scale: -coefficients[i]);
                assembler.AddFixedLoadContribution(local, indices, dofManager.AsFixedSpan(oldSolution), rhsVector, scale: coefficients[i]);
            }

            // 2. Global matrix: A += (c0)A_ff, b -= (c0)A_fd⋅u_d
            if (coefficients[0] != 0)
            {
                local *= coefficients[0];
                globalMatrix.AddLocalMatrix(local, indices);
                assembler.AddFixedLoadContribution(local, indices, dofManager.AsFixedSpan(solutions.Last), rhsVector);
            }
        }

        var alpha = new double[timeLayers];
        var beta = new double[timeLayers];
        var gamma = new double[timeLayers];

        for (int i = initialConditions; i < t.Length; ++i)
        {
            // 0. Cleanup for next step
            solutions.CycleOrPushNew(dofManager.CreateFullVector);
            // Array.Fill(solutions.Last, 0) - Reuse old solution as initial guess
            loadVectors.CycleOrPushNew(dofManager.CreateFreeVector);
            Array.Fill(loadVectors.Last, 0);
            Array.Fill(rhsVector, 0);
            globalMatrix.Fill(0);

            // 1. Calculate time scheme coefficients
            var currentScheme = timeSchemes[int.Min(i, timeSchemes.Length) - 1];
            var timeSpan = t.AsSpan(i - currentScheme.Layers + 1, currentScheme.Layers);
            var time = t[i];

            currentScheme.GetStiffnessScale(timeSpan, alpha);
            currentScheme.GetMassScale(timeSpan, beta);
            currentScheme.GetSourceScale(timeSpan, gamma);

            // 2. Calculate fixed solution at current time (u_d)
            assembler.CalculateFixedElements(
                problem.BoundaryConditions, time,
                _algebraicSolver,
                dofManager.AsFixedSpan(solutions.Last),
                solverParams
            );

            // 3. Assemble matrices part: ∑ [i=0,k | (αi)G + (βi)M + (ζi)MS3]⋅u_{n-i}
            assembler.CalculateStiffness(
                id => problem.Lambda(id, time),
                (local, indices) => ApplyLocalMatrix(local, indices, alpha)
            );
            assembler.CalculateMass(
                id => problem.Sigma(id, time),
                (local, indices) => ApplyLocalMatrix(local, indices, beta)
            );

            // 4. Assemble boundary conditions:
            assembler.CalculateRobinMassContribution(
                problem.BoundaryConditions, time,
                (local, indices) => ApplyLocalMatrix(local, indices, alpha)
            );

            // 5. Assemble load part: ∑ [(γi)⋅f_{n-i}] = 0
            assembler.CalculateLoad(id => problem.Source(id, time), loadVectors.Last);
            assembler.CalculateBoundaryLoadContribution(problem.BoundaryConditions, time, loadVectors.Last);
            for (int j = 0; j < loadVectors.Count; ++j)
                if (gamma[j] != 0)
                    rhsVector.AddScaled(-gamma[j], loadVectors[^(j + 1)]);

            // 6. Solve system
            _algebraicSolver.Matrix = globalMatrix;
            _algebraicSolver.Solve(
                rhsVector,
                dofManager.AsFreeSpan(solutions.Last),
                solverParams
            );

            yield return solution.WithCoefficients([.. solutions.Last]);
        }

        yield break;
    }

    [Conditional("DEBUG")]
    private static void DebugAssertSchemesCanBeUsed(HyperbolicProblem_<TSpace> problem, ITimeScheme[] schemes)
    {
        var conditions = problem.InitialConditions.Length;
        foreach (var scheme in schemes)
            Debug.Assert(scheme.Layers >= conditions + 1);
    }
}
