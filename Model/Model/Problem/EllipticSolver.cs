using Model.Core.CoordinateSystem;
using Model.Core.Solver;
using Model.Model.Assembly;
using Model.Model.Mesh;
using Telma.Extensions;

namespace Model.Model.Problem;

public class EllipticSolver<TSpace, TBoundary, TOps>
    where TSpace : IVectorBase<TSpace>
    where TBoundary : IVectorBase<TBoundary>
    where TOps : IMatrixOperations<TSpace, TSpace, TOps>
{
    private readonly Assembler<TSpace, TBoundary, TOps> _assembler;
    private readonly ISolver _algebraicSolver;

    public EllipticSolver(Assembler<TSpace, TBoundary, TOps> assembler, ISolver algebraicSolver)
    {
        _assembler = assembler;
        _algebraicSolver = algebraicSolver;
    }

    public double[] Solve(HyperbolicProblem<TSpace> problem, ISolver.Params solverParams = default)
    {
        if (solverParams.MaxIterations == 0)
        {
            solverParams = new ISolver.Params();
        }
        const double time = 0.0; // Время равно 0, так как задача стационарная

        _assembler.ResetSystemMatrix();
        _assembler.ResetLoadVector();
        _assembler.ResetFixedElements();
        
        // Граничные условия Дирихле 
        _assembler.CalculateFixedElements(problem.BoundaryConditions, time, _algebraicSolver,solverParams);

        // Сборка глобальной матрицы
        // Добавляем матрицу жесткости G
        _assembler.CalculateStiffness(matId => p => problem.Materials[matId].Lambda(p,time));

        // Добавляем матрицу масс M 
        _assembler.CalculateMass(matId => p => problem.Materials[matId].Sigma(p,time)); //здесь сигма это гамма

        // Добавляем вклад от 3го краевого в матрицу
        _assembler.CalculateRobinMassContribution(problem.BoundaryConditions, time);

        // Сборка правой части уравнения 
        // Добавляем источники (f)
        _assembler.CalculateLoad(matId => p => problem.Materials[matId].Source(p,time));

        // Добавляем потоки (условия Неймана и Робина)
        _assembler.CalculateBoundaryLoadContribution(problem.BoundaryConditions, time);

        // Решение СЛАУ
        var freeSolution = new double[_assembler.DofManager.FreeDofCount];
        _algebraicSolver.Matrix = _assembler.Matrix;
        _algebraicSolver.Solve(_assembler.RhsVector, freeSolution,solverParams);

        // Формирование итогового ответа 
        return GetFullSolution(freeSolution, _assembler.FixedSolution, _assembler.DofManager);
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



