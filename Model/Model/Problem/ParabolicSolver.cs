using Model.Core.CoordinateSystem;
using Model.Core.Matrix;
using Model.Core.Solver;
using Model.Core.Vector;
using Model.Model.Assembly;
using Model.Model.Integrator;
using Model.Model.Mesh;
using Telma.Extensions;

namespace Model.Model.Problem;

[Obsolete("This feature is under development")]
public class ParabolicSolver<TSpace, TBoundary, TOps>(
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

    /// <summary>
    /// Решение параболической задачи.
    /// Sigma - коэффициент при du/dt (матрица масс M_sigma)
    /// Lambda - коэффициент диффузии (матрица жесткости G)
    /// Xi - коэффициент при u (матрица масс M_gamma)
    /// Source - правая часть (f)
    /// </summary>
    public double[] Solve(
        HyperbolicProblem<TSpace> problem,
        double tStart,
        double tEnd,
        double dt,
        bool isImplicit,
        ISolver.Params solverParams = new())
    {
        var mesh = (IMeshWithBoundaries<TSpace, TBoundary>)problem.Mesh;
        var dofManager = DofManager.NumerateDof(mesh, problem.BoundaryConditions);
        var assembler = new Assembler<TSpace, TBoundary, TOps>(mesh, dofManager, _matrixFactory, _integrator);

        int dofCount = dofManager.FreeDofCount;

        if (IsStationaryOverStep(mesh, dofManager, problem, tStart, tEnd, dt, solverParams))
            return SolveStationary(mesh, dofManager, problem, tEnd, solverParams);

        // Свободные узлы на трех слоях: j-2, j-1, j
        var q_prev2 = new double[dofCount];
        var q_prev1 = new double[dofCount];
        var q_curr = new double[dofCount];
        var tempVec = new double[dofCount];

        // Старт двухслойной схемы: считаем первый временной слой t0 + dt.
        bool hasAtLeastOneStep = (tStart + dt) <= (tEnd + 1e-9);
        if (hasAtLeastOneStep)
        {
            double firstTime = tStart + dt;

            assembler.ResetSystemMatrix();
            assembler.ResetLoadVector();
            assembler.ResetFixedElements();
            assembler.CalculateFixedElements(problem.BoundaryConditions, firstTime, _algebraicSolver, solverParams);

            if (isImplicit)
            {
                var M = GetPureMassMatrix(mesh, dofManager, problem, firstTime);

                assembler.CalculateMass(id => p => (1.0 / dt) * problem.Materials[id].Sigma(p, firstTime));
                assembler.CalculateStiffness(id => p => problem.Materials[id].Lambda(p, firstTime));
                assembler.CalculateMass(id => p => problem.Materials[id].Xi(p, firstTime));
                assembler.CalculateRobinMassContribution(problem.BoundaryConditions, firstTime);

                assembler.CalculateLoad(id => p => problem.Materials[id].Source(p, firstTime));
                assembler.CalculateBoundaryLoadContribution(problem.BoundaryConditions, firstTime);

                var bEff = assembler.RhsVector.ToArray();

                M.MulVec(q_prev2, tempVec);
                bEff.AsSpan().AddScaled(1.0 / dt, tempVec, bEff);
                AddInPlace(
                    bEff,
                    GetMassFixedHistoryContribution(
                        mesh,
                        dofManager,
                        problem,
                        fixedTime: tStart,
                        matrixTime: firstTime,
                        scale: 1.0 / dt,
                        solverParams
                    )
                );

                _algebraicSolver.Matrix = assembler.Matrix;
                _algebraicSolver.Solve(bEff, q_prev1, solverParams);
            }
            else
            {
                var M = GetPureMassMatrix(mesh, dofManager, problem, tStart);
                var GM = GetPureStiffnessMatrix(mesh, dofManager, problem, tStart);

                assembler.CalculateMass(id => p => (1.0 / dt) * problem.Materials[id].Sigma(p, tStart));
                assembler.CalculateLoad(id => p => problem.Materials[id].Source(p, tStart));
                assembler.CalculateBoundaryLoadContribution(problem.BoundaryConditions, tStart);

                var bEff = assembler.RhsVector.ToArray();

                GM.MulVec(q_prev2, tempVec);
                bEff.AsSpan().Sub(tempVec, bEff);
                AddInPlace(
                    bEff,
                    GetStiffnessFixedHistoryContribution(
                        mesh,
                        dofManager,
                        problem,
                        fixedTime: tStart,
                        matrixTime: tStart,
                        solverParams
                    )
                );

                M.MulVec(q_prev2, tempVec);
                bEff.AsSpan().AddScaled(1.0 / dt, tempVec, bEff);
                AddInPlace(
                    bEff,
                    GetMassFixedHistoryContribution(
                        mesh,
                        dofManager,
                        problem,
                        fixedTime: tStart,
                        matrixTime: tStart,
                        scale: 1.0 / dt,
                        solverParams
                    )
                );

                _algebraicSolver.Matrix = assembler.Matrix;
                _algebraicSolver.Solve(bEff, q_prev1, solverParams);
            }

        }
        else
        {
            // Если шага нет, фиксированные DOF должны соответствовать tEnd.
            assembler.ResetFixedElements();
            assembler.CalculateFixedElements(problem.BoundaryConditions, tEnd, _algebraicSolver, solverParams);
        }

        for (double time = tStart + 2 * dt; time <= tEnd + 1e-9; time += dt)
        {
            assembler.ResetSystemMatrix();
            assembler.ResetLoadVector();
            assembler.ResetFixedElements();

            // Граничные условия Дирихле
            assembler.CalculateFixedElements(problem.BoundaryConditions, time, _algebraicSolver, solverParams);

            // Чистые матрицы без побочных эффектов на основной RHS.
            var M = GetPureMassMatrix(mesh, dofManager, problem, time);
            var GM = (!isImplicit) ? GetPureStiffnessMatrix(mesh, dofManager, problem, time - dt) : null;

            double[] bEff;
            if (isImplicit)
            {
                // BDF2: (3/2dt) M q_j + K q_j = f_j + (2/dt) M q_{j-1} - (1/2dt) M q_{j-2}
                assembler.CalculateMass(id => p => (1.5 / dt) * problem.Materials[id].Sigma(p, time));
                assembler.CalculateStiffness(id => p => problem.Materials[id].Lambda(p, time));
                assembler.CalculateMass(id => p => problem.Materials[id].Xi(p, time));
                assembler.CalculateRobinMassContribution(problem.BoundaryConditions, time);

                assembler.CalculateLoad(id => p => problem.Materials[id].Source(p, time));
                assembler.CalculateBoundaryLoadContribution(problem.BoundaryConditions, time);

                bEff = assembler.RhsVector.ToArray();

                M.MulVec(q_prev1, tempVec);
                bEff.AsSpan().AddScaled(2.0 / dt, tempVec, bEff);
                AddInPlace(
                    bEff,
                    GetMassFixedHistoryContribution(
                        mesh,
                        dofManager,
                        problem,
                        fixedTime: time - dt,
                        matrixTime: time,
                        scale: 2.0 / dt,
                        solverParams
                    )
                );

                M.MulVec(q_prev2, tempVec);
                bEff.AsSpan().AddScaled(-0.5 / dt, tempVec, bEff);
                AddInPlace(
                    bEff,
                    GetMassFixedHistoryContribution(
                        mesh,
                        dofManager,
                        problem,
                        fixedTime: time - 2 * dt,
                        matrixTime: time,
                        scale: -0.5 / dt,
                        solverParams
                    )
                );
            }
            else
            {
                // Явная трехслойная формула.
                double prevTime = time - dt;

                assembler.CalculateMass(id => p => (0.5 / dt) * problem.Materials[id].Sigma(p, time));

                assembler.CalculateLoad(id => p => problem.Materials[id].Source(p, prevTime));
                assembler.CalculateBoundaryLoadContribution(problem.BoundaryConditions, prevTime);

                bEff = assembler.RhsVector.ToArray();

                GM?.MulVec(q_prev1, tempVec);
                bEff.AsSpan().Sub(tempVec, bEff);
                AddInPlace(
                    bEff,
                    GetStiffnessFixedHistoryContribution(
                        mesh,
                        dofManager,
                        problem,
                        fixedTime: prevTime,
                        matrixTime: prevTime,
                        solverParams
                    )
                );

                M.MulVec(q_prev2, tempVec);
                bEff.AsSpan().AddScaled(0.5 / dt, tempVec, bEff);
                AddInPlace(
                    bEff,
                    GetMassFixedHistoryContribution(
                        mesh,
                        dofManager,
                        problem,
                        fixedTime: time - 2 * dt,
                        matrixTime: time,
                        scale: 0.5 / dt,
                        solverParams
                    )
                );
            }

            _algebraicSolver.Matrix = assembler.Matrix;
            _algebraicSolver.Solve(bEff, q_curr, solverParams);

            Array.Copy(q_prev1, q_prev2, dofCount);
            Array.Copy(q_curr, q_prev1, dofCount);
        }

        var finalFree = hasAtLeastOneStep ? q_prev1 : q_prev2;
        return GetFullSolution(finalFree, assembler.FixedSolution, assembler.DofManager);
    }

    private bool IsStationaryOverStep(
        IMeshWithBoundaries<TSpace, TBoundary> mesh,
        DofManager dofManager,
        HyperbolicProblem<TSpace> problem,
        double tStart,
        double tEnd,
        double dt,
        ISolver.Params solverParams
    )
    {
        double tNext = tStart + dt;
        if (tNext > tEnd + 1e-9)
            return false;

        var snapshot0 = BuildStationarySnapshot(mesh, dofManager, problem, tStart, solverParams);
        var snapshot1 = BuildStationarySnapshot(mesh, dofManager, problem, tNext, solverParams);

        if (!AreClose(snapshot0.fixedValues, snapshot1.fixedValues))
            return false;
        if (!AreClose(snapshot0.rhs, snapshot1.rhs))
            return false;
        if (!AreClose(snapshot0.matrixOnes, snapshot1.matrixOnes))
            return false;

        return true;
    }

    private (double[] fixedValues, double[] rhs, double[] matrixOnes) BuildStationarySnapshot(
        IMeshWithBoundaries<TSpace, TBoundary> mesh,
        DofManager dofManager,
        HyperbolicProblem<TSpace> problem,
        double time,
        ISolver.Params solverParams
    )
    {
        var assembler = new Assembler<TSpace, TBoundary, TOps>(mesh, dofManager, _matrixFactory, _integrator);
        assembler.ResetSystemMatrix();
        assembler.ResetLoadVector();
        assembler.ResetFixedElements();

        assembler.CalculateFixedElements(problem.BoundaryConditions, time, _algebraicSolver, solverParams);
        assembler.CalculateStiffness(id => p => problem.Materials[id].Lambda(p, time));
        assembler.CalculateMass(id => p => problem.Materials[id].Xi(p, time));
        assembler.CalculateRobinMassContribution(problem.BoundaryConditions, time);
        assembler.CalculateLoad(id => p => problem.Materials[id].Source(p, time));
        assembler.CalculateBoundaryLoadContribution(problem.BoundaryConditions, time);

        var ones = new double[dofManager.FreeDofCount];
        Array.Fill(ones, 1.0);
        var matrixOnes = new double[dofManager.FreeDofCount];
        assembler.Matrix.MulVec(ones, matrixOnes);

        return (assembler.FixedSolution.ToArray(), assembler.RhsVector.ToArray(), matrixOnes);
    }

    private double[] SolveStationary(
        IMeshWithBoundaries<TSpace, TBoundary> mesh,
        DofManager dofManager,
        HyperbolicProblem<TSpace> problem,
        double time,
        ISolver.Params solverParams
    )
    {
        var assembler = new Assembler<TSpace, TBoundary, TOps>(mesh, dofManager, _matrixFactory, _integrator);
        assembler.ResetSystemMatrix();
        assembler.ResetLoadVector();
        assembler.ResetFixedElements();

        assembler.CalculateFixedElements(problem.BoundaryConditions, time, _algebraicSolver, solverParams);
        assembler.CalculateStiffness(id => p => problem.Materials[id].Lambda(p, time));
        assembler.CalculateMass(id => p => problem.Materials[id].Xi(p, time));
        assembler.CalculateRobinMassContribution(problem.BoundaryConditions, time);
        assembler.CalculateLoad(id => p => problem.Materials[id].Source(p, time));
        assembler.CalculateBoundaryLoadContribution(problem.BoundaryConditions, time);

        var free = new double[dofManager.FreeDofCount];
        _algebraicSolver.Matrix = assembler.Matrix;
        _algebraicSolver.Solve(assembler.RhsVector, free, solverParams);

        return GetFullSolution(free, assembler.FixedSolution, dofManager);
    }

    private IGlobalMatrix GetPureMassMatrix(
        IMeshWithBoundaries<TSpace, TBoundary> mesh,
        DofManager dofManager,
        HyperbolicProblem<TSpace> problem,
        double time
    )
    {
        var pureAssembler = new Assembler<TSpace, TBoundary, TOps>(mesh, dofManager, _matrixFactory, _integrator);
        pureAssembler.ResetSystemMatrix();
        pureAssembler.CalculateMass(id => p => problem.Materials[id].Sigma(p, time));
        return (IGlobalMatrix)pureAssembler.Matrix.Clone();
    }

    private IGlobalMatrix GetPureStiffnessMatrix(
        IMeshWithBoundaries<TSpace, TBoundary> mesh,
        DofManager dofManager,
        HyperbolicProblem<TSpace> problem,
        double time
    )
    {
        var pureAssembler = new Assembler<TSpace, TBoundary, TOps>(mesh, dofManager, _matrixFactory, _integrator);
        pureAssembler.ResetSystemMatrix();
        pureAssembler.CalculateStiffness(id => p => problem.Materials[id].Lambda(p, time));
        pureAssembler.CalculateMass(id => p => problem.Materials[id].Xi(p, time));
        pureAssembler.CalculateRobinMassContribution(problem.BoundaryConditions, time);
        return (IGlobalMatrix)pureAssembler.Matrix.Clone();
    }

    private double[] GetMassFixedHistoryContribution(
        IMeshWithBoundaries<TSpace, TBoundary> mesh,
        DofManager dofManager,
        HyperbolicProblem<TSpace> problem,
        double fixedTime,
        double matrixTime,
        double scale,
        ISolver.Params solverParams
    )
    {
        if (dofManager.FixedDofCount == 0 || Math.Abs(scale) < 1e-30)
            return new double[dofManager.FreeDofCount];

        var localAssembler = new Assembler<TSpace, TBoundary, TOps>(mesh, dofManager, _matrixFactory, _integrator);
        localAssembler.ResetSystemMatrix();
        localAssembler.ResetLoadVector();
        localAssembler.ResetFixedElements();

        localAssembler.CalculateFixedElements(problem.BoundaryConditions, fixedTime, _algebraicSolver, solverParams);
        // assembler добавляет -(coeff * M_fd * q_d) в RHS, поэтому берем -scale.
        localAssembler.CalculateMass(id => p => -scale * problem.Materials[id].Sigma(p, matrixTime));

        return localAssembler.RhsVector.ToArray();
    }

    private double[] GetStiffnessFixedHistoryContribution(
        IMeshWithBoundaries<TSpace, TBoundary> mesh,
        DofManager dofManager,
        HyperbolicProblem<TSpace> problem,
        double fixedTime,
        double matrixTime,
        ISolver.Params solverParams
    )
    {
        if (dofManager.FixedDofCount == 0)
            return new double[dofManager.FreeDofCount];

        var localAssembler = new Assembler<TSpace, TBoundary, TOps>(mesh, dofManager, _matrixFactory, _integrator);
        localAssembler.ResetSystemMatrix();
        localAssembler.ResetLoadVector();
        localAssembler.ResetFixedElements();

        localAssembler.CalculateFixedElements(problem.BoundaryConditions, fixedTime, _algebraicSolver, solverParams);
        localAssembler.CalculateStiffness(id => p => problem.Materials[id].Lambda(p, matrixTime));
        localAssembler.CalculateMass(id => p => problem.Materials[id].Xi(p, matrixTime));
        localAssembler.CalculateRobinMassContribution(problem.BoundaryConditions, matrixTime);

        return localAssembler.RhsVector.ToArray();
    }

    private static double[] GetFullSolution(ReadOnlySpan<double> freeSolution, ReadOnlySpan<double> fixedSolution, DofManager dofManager)
    {
        var result = new double[dofManager.TotalDofCount];
        freeSolution.CopyTo(result);
        fixedSolution.CopyTo(result.AsSpan(start: dofManager.FreeDofCount));
        return result;
    }

    private static void AddInPlace(Span<double> target, ReadOnlySpan<double> addend)
    {
        for (int i = 0; i < target.Length; i++)
            target[i] += addend[i];
    }

    private static bool AreClose(ReadOnlySpan<double> a, ReadOnlySpan<double> b)
    {
        if (a.Length != b.Length)
            return false;
        for (int i = 0; i < a.Length; i++)
        {
            if (Math.Abs(a[i] - b[i]) > 1e-12)
                return false;
        }
        return true;
    }
}
