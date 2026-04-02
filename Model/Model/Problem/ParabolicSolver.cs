using Model.Core.CoordinateSystem;
using Model.Core.Matrix;
using Model.Core.Solver;
using Model.Core.Vector;
using Model.Model.Assembly;

using Telma.Extensions;

namespace Model.Model.Problem;

public class ParabolicSolver<TSpace, TBoundary, TOps>
    where TSpace : IVectorBase<TSpace>
    where TBoundary : IVectorBase<TBoundary>
    where TOps : IMatrixOperations<TSpace, TSpace, TOps>
{
    private readonly Assembler<TSpace, TBoundary, TOps> _assembler;
    private readonly ISolver _algebraicSolver;

    public ParabolicSolver(Assembler<TSpace, TBoundary, TOps> assembler, ISolver algebraicSolver)
    {
        _assembler = assembler;
        _algebraicSolver = algebraicSolver;
    }
    /// <summary>
    /// Решение параболической задачи.
    /// Sigma - коэффициент при du/dt (матрица масс M_sigma)
    /// Lambda - коэффициент диффузии (матрица жесткости G)
    /// Xi - коэффициент при u (матрица масс M_gamma) 
    /// Source - правая часть (f)
    /// </summary>
    public double[] Solve(HyperbolicProblem<TSpace> problem, double tStart, double tEnd, double dt, bool isImplicit)
    {
        int dofCount = _assembler.DofManager.FreeDofCount;
        var history = new List<double[]>();

        // вектрые для хранения свободных узлов на трех слоях (j-2, j-1 и j)
        var q_prev2 = new double[dofCount];
        var q_prev1 = new double[dofCount];
        var q_curr = new double[dofCount];

        var tempVec = new double[dofCount];

        for (double time = tStart + 2 * dt; time <= tEnd + 1e-9; time += dt)
        {
            _assembler.ResetSystemMatrix();
            _assembler.ResetLoadVector();
            _assembler.ResetFixedElements();

            // Граничные условия Дирихле 
            _assembler.CalculateFixedElements(problem.BoundaryConditions, time, _algebraicSolver);

            // Сборка глобальной матрицы
            // Добавляем матрицу жесткости G + M_sigma + M_gamma
            var M = GetPureMassMatrix(problem, time);
            var GM = (!isImplicit) ? GetPureStiffnessMatrix(problem, time - dt) : null;

            double[] b_eff;

            if (isImplicit) // это формула (7.24) из методички, для неявной схемы
            {
                // тут оставил вызовы, чтобы внесенные ранее вклады в матрицу от M_sigma не затерлись ( можно написать GetPureStiffnessMatrix более правильно, но пока так )
                _assembler.CalculateMass(id => p => (0.5 / dt) * problem.Materials[id].Sigma(p, time)); // так как шаг dt фиксированный, то посчитал delta(t), delta(t0) и delta(t1) как констаныт  
                _assembler.CalculateStiffness(id => p => problem.Materials[id].Lambda(p, time));
                _assembler.CalculateMass(id => p => problem.Materials[id].Xi(p, time));
                _assembler.CalculateRobinMassContribution(problem.BoundaryConditions, time);

                // Сборка правой части уравнения 
                // Добавляем источники (f)
                _assembler.CalculateLoad(id => p => problem.Materials[id].Source(p, time));
                // Добавляем потоки (условия Неймана и Робина)
                _assembler.CalculateBoundaryLoadContribution(problem.BoundaryConditions, time);

                b_eff = _assembler.RhsVector.ToArray();

                M.MulVec(q_prev1, tempVec);
                b_eff.AsSpan().AddScaled(2.0 / dt, tempVec, b_eff);

                M.MulVec(q_prev2, tempVec);
                b_eff.AsSpan().AddScaled(-0.5 / dt, tempVec, b_eff);
            }
            else // это формула  для явной схемы(если я ее верно расписал) 
            {
                double prevTime = time - dt;

                // Левая часть
                _assembler.CalculateMass(id => p => (0.5 / dt) * problem.Materials[id].Sigma(p, time));

                // Сборка правой части уравнения 
                _assembler.CalculateLoad(id => p => problem.Materials[id].Source(p, prevTime));
                _assembler.CalculateBoundaryLoadContribution(problem.BoundaryConditions, prevTime);

                b_eff = _assembler.RhsVector.ToArray();

                GM?.MulVec(q_prev1, tempVec);
                b_eff.AsSpan().Sub(tempVec, b_eff);

                M.MulVec(q_prev2, tempVec);
                b_eff.AsSpan().AddScaled(0.5 / dt, tempVec, b_eff);
            }
            // Решение СЛАУ
            _algebraicSolver.Matrix = _assembler.Matrix;
            _algebraicSolver.Solve(b_eff, q_curr);

            Array.Copy(q_prev1, q_prev2, dofCount);
            Array.Copy(q_curr, q_prev1, dofCount);
        }
        // Формирование итогового ответа 
        return GetFullSolution(q_curr, _assembler.FixedSolution, _assembler.DofManager);
    }


    private IGlobalMatrix GetPureMassMatrix(HyperbolicProblem<TSpace> problem, double time)
    {
        _assembler.ResetSystemMatrix();
        _assembler.CalculateMass(id => p => problem.Materials[id].Sigma(p, time));
        return (IGlobalMatrix)_assembler.Matrix.Clone();
    }

    private IGlobalMatrix GetPureStiffnessMatrix(HyperbolicProblem<TSpace> problem, double time)
    {
        _assembler.ResetSystemMatrix();
        _assembler.CalculateStiffness(id => p => problem.Materials[id].Lambda(p, time));
        _assembler.CalculateMass(id => p => problem.Materials[id].Xi(p, time));  // в Material нет гаммы, поэтому теперь это гамма
        _assembler.CalculateRobinMassContribution(problem.BoundaryConditions, time);
        return (IGlobalMatrix)_assembler.Matrix.Clone();
    }

    private static double[] GetFullSolution(ReadOnlySpan<double> freeSolution, ReadOnlySpan<double> fixedSolution, DofManager dofManager)
    {
        var result = new double[dofManager.TotalDofCount];
        for (int i = 0; i < dofManager.TotalDofCount; i++)
        {
            if (i < dofManager.FreeDofCount)
                result[i] = freeSolution[i];
            else
                result[i] = fixedSolution[i - dofManager.FreeDofCount];
        }
        return result;
    }
}


 
