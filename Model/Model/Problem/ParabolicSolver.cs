using Model.Core.CoordinateSystem;
using Model.Core.Matrix;
using Model.Core.Solver;
using Model.Core.Vector;
using Model.Model.Assembly;
using Model.Model.Integrator;
using Model.Model.Mesh;
using Telma.Extensions;

namespace Model.Model.Problem;

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
    public double[] Solve(HyperbolicProblem<TSpace> problem, double tStart, double tEnd, double dt, bool isImplicit)
    {
        var mesh = (IMeshWithBoundaries<TSpace, TBoundary>)problem.Mesh;
        var dofManager = DofManager.NumerateDof(mesh, problem.BoundaryConditions);
        var assembler = new Assembler<TSpace, TBoundary, TOps>(mesh, dofManager, _matrixFactory, _integrator);

        int dofCount = dofManager.FreeDofCount;
        var history = new List<double[]>();

        // вектрые для хранения свободных узлов на трех слоях (j-2, j-1 и j)
        var q_prev2 = new double[dofCount];
        var q_prev1 = new double[dofCount];
        var q_curr = new double[dofCount];

        var tempVec = new double[dofCount];

        for (double time = tStart + 2 * dt; time <= tEnd + 1e-9; time += dt)
        {
            assembler.ResetSystemMatrix();
            assembler.ResetLoadVector();
            assembler.ResetFixedElements();

            // Граничные условия Дирихле 
            assembler.CalculateFixedElements(problem.BoundaryConditions, time, _algebraicSolver);

            // Сборка глобальной матрицы
            // Добавляем матрицу жесткости G + M_sigma + M_gamma
            var M = GetPureMassMatrix(assembler, problem, time);
            var GM = (!isImplicit) ? GetPureStiffnessMatrix(assembler, problem, time - dt) : null;

            double[] b_eff;

            if (isImplicit) // это формула (7.24) из методички, для неявной схемы
            {
                // тут оставил вызовы, чтобы внесенные ранее вклады в матрицу от M_sigma не затерлись ( можно написать GetPureStiffnessMatrix более правильно, но пока так )
                assembler.CalculateMass(id => p => (0.5 / dt) * problem.Materials[id].Sigma(p, time)); // так как шаг dt фиксированный, то посчитал delta(t), delta(t0) и delta(t1) как констаныт  
                assembler.CalculateStiffness(id => p => problem.Materials[id].Lambda(p, time));
                assembler.CalculateMass(id => p => problem.Materials[id].Xi(p, time));
                assembler.CalculateRobinMassContribution(problem.BoundaryConditions, time);

                // Сборка правой части уравнения 
                // Добавляем источники (f)
                assembler.CalculateLoad(id => p => problem.Materials[id].Source(p, time));
                // Добавляем потоки (условия Неймана и Робина)
                assembler.CalculateBoundaryLoadContribution(problem.BoundaryConditions, time);

                b_eff = assembler.RhsVector.ToArray();

                M.MulVec(q_prev1, tempVec);
                b_eff.AsSpan().AddScaled(2.0 / dt, tempVec, b_eff);

                M.MulVec(q_prev2, tempVec);
                b_eff.AsSpan().AddScaled(-0.5 / dt, tempVec, b_eff);
            }
            else // это формула  для явной схемы(если я ее верно расписал) 
            {
                double prevTime = time - dt;

                // Левая часть
                assembler.CalculateMass(id => p => (0.5 / dt) * problem.Materials[id].Sigma(p, time));

                // Сборка правой части уравнения 
                assembler.CalculateLoad(id => p => problem.Materials[id].Source(p, prevTime));
                assembler.CalculateBoundaryLoadContribution(problem.BoundaryConditions, prevTime);

                b_eff = assembler.RhsVector.ToArray();

                GM?.MulVec(q_prev1, tempVec);
                b_eff.AsSpan().Sub(tempVec, b_eff);

                M.MulVec(q_prev2, tempVec);
                b_eff.AsSpan().AddScaled(0.5 / dt, tempVec, b_eff);
            }
            // Решение СЛАУ
            _algebraicSolver.Matrix = assembler.Matrix;
            _algebraicSolver.Solve(b_eff, q_curr);

            Array.Copy(q_prev1, q_prev2, dofCount);
            Array.Copy(q_curr, q_prev1, dofCount);
        }
        // Формирование итогового ответа 
        return GetFullSolution(q_curr, assembler.FixedSolution, assembler.DofManager);
    }


    private IGlobalMatrix GetPureMassMatrix(Assembler<TSpace, TBoundary, TOps> assembler, HyperbolicProblem<TSpace> problem, double time)
    {
        assembler.ResetSystemMatrix();
        assembler.CalculateMass(id => p => problem.Materials[id].Sigma(p, time));
        return (IGlobalMatrix)assembler.Matrix.Clone();
    }

    private IGlobalMatrix GetPureStiffnessMatrix(Assembler<TSpace, TBoundary, TOps> assembler, HyperbolicProblem<TSpace> problem, double time)
    {
        assembler.ResetSystemMatrix();
        assembler.CalculateStiffness(id => p => problem.Materials[id].Lambda(p, time));
        assembler.CalculateMass(id => p => problem.Materials[id].Xi(p, time));  // в Material нет гаммы, поэтому теперь это гамма
        assembler.CalculateRobinMassContribution(problem.BoundaryConditions, time);
        return (IGlobalMatrix)assembler.Matrix.Clone();
    }

    private static double[] GetFullSolution(ReadOnlySpan<double> freeSolution, ReadOnlySpan<double> fixedSolution, DofManager dofManager)
    {
        var result = new double[dofManager.TotalDofCount];
        freeSolution.CopyTo(result);
        fixedSolution.CopyTo(result.AsSpan(start: dofManager.FreeDofCount));
        return result;
    }
}


 
