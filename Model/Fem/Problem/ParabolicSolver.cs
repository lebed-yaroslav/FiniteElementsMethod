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


[Obsolete($"Will be replaced with {nameof(ParabolicSolver_<,,>)}")]
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
        ISolver.Params solverParams = new()
    )
    {
        DebugAssertSchemesAreCorrectTypes(_timeSchemes);
        var timeLayers = _timeSchemes[^1].Layers;

        var mesh = (IMesh<TSpace, TBoundary>)problem.Mesh;
        var dofManager = DofManager.NumerateDof(mesh, problem.BoundaryConditions);
        var assembler = new Assembler<TSpace, TBoundary, TOps>(mesh, dofManager, _matrixFactory, _integrator);

        var globalMatrix = assembler.CreateGlobalMatrix(); // A_ff
        var rhsVector = dofManager.CreateFreeVector();

        var solutions = new SlidingWindow<double[]>(timeLayers); // u_{n-i}
        solutions.Push(dofManager.CreateFullVector());

        assembler.ProjectToSolutionSpace(problem.InitialCondition, _algebraicSolver, solutions.Last, solverParams);

        var loadVectors = new SlidingWindow<double[]>(timeLayers); // b_ff{n-i}
        loadVectors.Push(dofManager.CreateFreeVector());
        assembler.CalculateLoad(id => problem.Source(id, timePoints[0]), loadVectors.Last);
        assembler.CalculateBoundaryLoadContribution(problem.BoundaryConditions, timePoints[0], loadVectors.Last);

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

        var solution = new StationarySolution<TSpace, TBoundary>(mesh, []);
        solution.BuildSearchTree();

        for (int i = 1; i < timePoints.Length; ++i)
        {

            // 0. Cleanup for next step
            solutions.CycleOrPushNew(dofManager.CreateFullVector);
            // Array.Fill(solutions.Last, 0) - Reuse old solution as initial guess
            loadVectors.CycleOrPushNew(dofManager.CreateFreeVector);
            Array.Fill(loadVectors.Last, 0);
            Array.Fill(rhsVector, 0);
            globalMatrix.Fill(0);

            // 1. Calculate time scheme coefficients
            var currentScheme = _timeSchemes[int.Min(i, _timeSchemes.Length) - 1];
            var timeSpan = timePoints.AsSpan(i - currentScheme.Layers + 1, currentScheme.Layers);
            var time = timePoints[i];

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
    private static void DebugAssertSchemesAreCorrectTypes(ITimeScheme[] schemes)
    {
        for (int i = 0; i < schemes.Length; ++i)
            Debug.Assert(schemes[i].Layers == i + 2);
    }
}
