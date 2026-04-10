using Model.Core.CoordinateSystem;
using Model.Core.Matrix;
using Model.Core.Solver;
using Model.Core.Vector;
using Model.Model.Assembly;
using Model.Model.Integrator;
using Model.Model.Mesh;
using Telma.Extensions;

namespace Model.Model.Problem;
/// <summary>
/// Решение задачи с использованием двухслойных схем по времени.
/// Принимает массив timeLayers для поддержки неравномерных шагов по времени.
/// </summary>
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
    public List<StationarySolution<TSpace,TBoundary>> Solve2Layer(
        HyperbolicProblem<TSpace> problem,
        double[] timeLayers,
        bool isImplicit,
        ISolver.Params solverParams = new())
    {
        var mesh = (IMeshWithBoundaries<TSpace, TBoundary>)problem.Mesh;
        var dofManager = DofManager.NumerateDof(mesh, problem.BoundaryConditions);
        var assembler = new Assembler<TSpace, TBoundary, TOps>(mesh, dofManager, _matrixFactory, _integrator);

        int dofCount = dofManager.FreeDofCount;

        var q_prev = new double[dofCount];
        var q_curr = new double[dofCount];
        var tempVec = new double[dofCount];
        //var q_t = new double[timeLayers.Length][];
        //for (int i = 0; i < timeLayers.Length; i++)
        //{
        //    q_t[i] = new double[dofManager.TotalDofCount];
        //}
        var q_t = new List<StationarySolution<TSpace, TBoundary>>();
        double t0 = timeLayers[0];
        //Решение на первом временном слое 
        assembler.CalculateFixedElements(problem.BoundaryConditions, t0, _algebraicSolver, solverParams);
        assembler.CalculateStiffness(id => p => problem.Materials[id].Lambda(p, t0));
        assembler.CalculateMass(id => p => problem.Materials[id].Sigma(p, t0));
        assembler.CalculateRobinMassContribution(problem.BoundaryConditions, t0);

        assembler.CalculateLoad(id => p => problem.Materials[id].Source(p, t0));
        assembler.CalculateBoundaryLoadContribution(problem.BoundaryConditions, t0);

        _algebraicSolver.Matrix = assembler.Matrix;
        _algebraicSolver.Solve(assembler.RhsVector, q_prev, solverParams);

        double[] fullSolution = GetFullSolution(q_prev, assembler.FixedSolution, assembler.DofManager);
        q_t.Add(new StationarySolution<TSpace, TBoundary>(mesh:  assembler.Mesh,coefficients: fullSolution));

        for (int i = 1; i < timeLayers.Length; i++)
        {
            double time = timeLayers[i];
            double prevTime = timeLayers[i - 1];
            double dt = time - prevTime;

            // Собираем вспомогательные матрицы для умножения на q^{j-1}
            // Делаем это ДО сборки основной системы, чтобы не потерять ее
            IGlobalMatrix MassMatrix;
            IGlobalMatrix StiffnessMatrix = null;

            if (isImplicit)
            {
                MassMatrix = GetPureMassMatrix(assembler, problem, time);
            }
            else
            {
                MassMatrix = GetPureMassMatrix(assembler, problem, prevTime);
                StiffnessMatrix = GetPureStiffnessMatrix(assembler, problem, prevTime);
            }


            assembler.ResetSystemMatrix();
            assembler.ResetLoadVector();
            assembler.ResetFixedElements();

            assembler.CalculateFixedElements(problem.BoundaryConditions, time, _algebraicSolver, solverParams);

            double[] b = new double[dofCount];

            if (isImplicit)
            {
                //неявная двуслойная схема
                // Левая часть: (1/dt * M + G + M^S3)
                assembler.CalculateMass(id => p => (1.0 / dt) * problem.Materials[id].Sigma(p, time));
                assembler.CalculateStiffness(id => p => problem.Materials[id].Lambda(p, time));
                assembler.CalculateRobinMassContribution(problem.BoundaryConditions, time);

                // Правая часть: b^j
                assembler.CalculateLoad(id => p => problem.Materials[id].Source(p, time));
                assembler.CalculateBoundaryLoadContribution(problem.BoundaryConditions, time);
                b = assembler.RhsVector.ToArray();

                // Добавка в правую часть: + 1/dt * M * q^{j-1}
                MassMatrix.MulVec(q_prev, tempVec);
                b.AsSpan().AddScaled(1.0 / dt, tempVec, b);
            }
            else
            {
                // явная двуслойная схема
                // Левая часть: (1/dt * M + M^S3)
                assembler.CalculateMass(id => p => (1.0 / dt) * problem.Materials[id].Sigma(p, time));
                assembler.CalculateRobinMassContribution(problem.BoundaryConditions, time);

                // Правая часть базовая: b^{j-1} 
                assembler.CalculateLoad(id => p => problem.Materials[id].Source(p, prevTime));
                assembler.CalculateBoundaryLoadContribution(problem.BoundaryConditions, prevTime);
                b = assembler.RhsVector.ToArray();

                //  + 1/dt * M * q^{j-1}
                MassMatrix.MulVec(q_prev, tempVec);
                b.AsSpan().AddScaled(1.0 / dt, tempVec, b);

                //  - G * q^{j-1}
                StiffnessMatrix.MulVec(q_prev, tempVec);
                b.AsSpan().Sub(tempVec, b);
            }

            // Решение СЛАУ для текущего слоя
            _algebraicSolver.Matrix = assembler.Matrix;
            _algebraicSolver.Solve(b, q_curr, solverParams);

            Array.Copy(q_curr, q_prev, dofCount);
            fullSolution = GetFullSolution(q_prev, assembler.FixedSolution, assembler.DofManager);
            q_t.Add(new StationarySolution<TSpace, TBoundary>(mesh: assembler.Mesh, coefficients: fullSolution));
        }

        return q_t;
    }
    /// <summary>
    /// Решение параболической задачи.
    /// Sigma - коэффициент при du/dt (матрица масс M_sigma)
    /// Lambda - коэффициент диффузии (матрица жесткости G)
    /// Xi - коэффициент при u (матрица масс M_gamma)
    /// Source - правая часть (f)
    /// </summary> 
    //public double[][] Solve(
    //    HyperbolicProblem<TSpace> problem, double[] timeLayers
    //    bool isImplicit,
    //    ISolver.Params solverParams = new())
    //{
    //    var mesh = (IMeshWithBoundaries<TSpace, TBoundary>)problem.Mesh;
    //    var dofManager = DofManager.NumerateDof(mesh, problem.BoundaryConditions);
    //    var assembler = new Assembler<TSpace, TBoundary, TOps>(mesh, dofManager, _matrixFactory, _integrator);

    //    int dofCount = dofManager.FreeDofCount;
    //    var history = new List<double[]>();

    //    // вектрые для хранения свободных узлов на трех слоях (j-2, j-1 и j)
    //    var q_prev2 = new double[dofCount];
    //    var q_prev1 = new double[dofCount];
    //    var q_curr = new double[dofCount];

    //    var tempVec = new double[dofCount];
    //    var q_t = new double[timeLayers.Length][];
    //    for (int i = 0; i < timeLayers.Length; i++)
    //    {
    //        q_t[i] = new double[dofManager.TotalDofCount];
    //    }

    //    for (int i = 1; i < timeLayers.Length; i++)
    //    {
    //        double time = timeLayers[i];
    //        double prevTime = timeLayers[i - 1];
    //        double dt = time - prevTime;

    //        assembler.ResetSystemMatrix();
    //        assembler.ResetLoadVector();
    //        assembler.ResetFixedElements();

    //        // Граничные условия Дирихле 
    //        assembler.CalculateFixedElements(problem.BoundaryConditions, time, _algebraicSolver, solverParams);

    //        // Сборка глобальной матрицы
    //        // Добавляем матрицу жесткости G + M_sigma + M_gamma
    //        var M = GetPureMassMatrix(assembler, problem, time);
    //        var GM = (!isImplicit) ? GetGloblaMatrixForFreeDofs(assembler, problem, time - dt) : null;

    //        double[] b_eff;

    //        if (isImplicit) // это формула (7.24) из методички, для неявной схемы
    //        {
    //            // тут оставил вызовы, чтобы внесенные ранее вклады в матрицу от M_sigma не затерлись ( можно написать GetPureStiffnessMatrix более правильно, но пока так )
    //            assembler.CalculateMass(id => p => (0.5 / dt) * problem.Materials[id].Sigma(p, time)); // так как шаг dt фиксированный, то посчитал delta(t), delta(t0) и delta(t1) как констаныт  
    //            assembler.CalculateStiffness(id => p => problem.Materials[id].Lambda(p, time));
    //            assembler.CalculateMass(id => p => problem.Materials[id].Xi(p, time));
    //            assembler.CalculateRobinMassContribution(problem.BoundaryConditions, time);

    //            // Сборка правой части уравнения 
    //            // Добавляем источники (f)
    //            assembler.CalculateLoad(id => p => problem.Materials[id].Source(p, time));
    //            // Добавляем потоки (условия Неймана и Робина)
    //            assembler.CalculateBoundaryLoadContribution(problem.BoundaryConditions, time);

    //            b_eff = assembler.RhsVector.ToArray();

    //            M.MulVec(q_prev1, tempVec);
    //            b_eff.AsSpan().AddScaled(2.0 / dt, tempVec, b_eff);

    //            M.MulVec(q_prev2, tempVec);
    //            b_eff.AsSpan().AddScaled(-0.5 / dt, tempVec, b_eff);
    //        }
    //        else // это формула  для явной схемы(если я ее верно расписал) 
    //        {
    //            double prevTime = time - dt;

    //            // Левая часть
    //            assembler.CalculateMass(id => p => (0.5 / dt) * problem.Materials[id].Sigma(p, time));

    //            // Сборка правой части уравнения 
    //            assembler.CalculateLoad(id => p => problem.Materials[id].Source(p, prevTime));
    //            assembler.CalculateBoundaryLoadContribution(problem.BoundaryConditions, prevTime);

    //            b_eff = assembler.RhsVector.ToArray();

    //            GM?.MulVec(q_prev1, tempVec);
    //            b_eff.AsSpan().Sub(tempVec, b_eff);

    //            M.MulVec(q_prev2, tempVec);
    //            b_eff.AsSpan().AddScaled(0.5 / dt, tempVec, b_eff);
    //        }
    //        // Решение СЛАУ
    //        _algebraicSolver.Matrix = assembler.Matrix;
    //        _algebraicSolver.Solve(b_eff, q_curr, solverParams);

    //        Array.Copy(q_prev1, q_prev2, dofCount);
    //        Array.Copy(q_curr, q_prev1, dofCount);
    //    }
    //    // Формирование итогового ответа 
    //    return GetFullSolution(q_curr, assembler.FixedSolution, assembler.DofManager);
    //}
    


    /// <summary>
    /// Вспомогательный метод: собирает "чистую" матрицу жесткости G.
    /// Учитывает ТОЛЬКО Lambda (условия Робина не учитываются).
    /// </summary>
    private IGlobalMatrix GetPureStiffnessMatrix(Assembler<TSpace, TBoundary, TOps> assembler, HyperbolicProblem<TSpace> problem, double time)
    {
        assembler.ResetSystemMatrix();
        assembler.CalculateStiffness(id => p => problem.Materials[id].Lambda(p, time));
        return (IGlobalMatrix)assembler.Matrix.Clone();
    }
    
    private IGlobalMatrix GetPureMassMatrix(Assembler<TSpace, TBoundary, TOps> assembler, HyperbolicProblem<TSpace> problem, double time)
    {
        assembler.ResetSystemMatrix();
        assembler.CalculateMass(id => p => problem.Materials[id].Sigma(p, time));
        return (IGlobalMatrix)assembler.Matrix.Clone();
    }

    private static double[] GetFullSolution(ReadOnlySpan<double> freeSolution, ReadOnlySpan<double> fixedSolution, DofManager dofManager)
    {
        var result = new double[dofManager.TotalDofCount];
        freeSolution.CopyTo(result);
        fixedSolution.CopyTo(result.AsSpan(start: dofManager.FreeDofCount));
        return result;
    }
    private IGlobalMatrix GetGloblaMatrixForFreeDofs(Assembler<TSpace, TBoundary, TOps> assembler, HyperbolicProblem<TSpace> problem, double time)
    {
        assembler.ResetSystemMatrix();
        assembler.CalculateStiffness(id => p => problem.Materials[id].Lambda(p, time));
        assembler.CalculateMass(id => p => problem.Materials[id].Sigma(p, time));
        assembler.CalculateRobinMassContribution(problem.BoundaryConditions, time);
        return (IGlobalMatrix)assembler.Matrix.Clone();
    }
}


 
