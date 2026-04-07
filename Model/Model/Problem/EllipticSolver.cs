using Model.Core.CoordinateSystem;
using Model.Core.Matrix;
using Model.Core.Solver;
using Model.Model.Assembly;
using Model.Model.Integrator;
using Model.Model.Mesh;
using Telma.Extensions;

namespace Model.Model.Problem;

/// <summary>
/// Solves problem: -∇⋅(λ∇u) + γu = f
/// </summary>
/// <typeparam name="TSpace">Domain function space type</typeparam>
/// <typeparam name="TBoundary">Boundary function space type</typeparam>
/// <typeparam name="TOps">Matrix operations type for the function space</typeparam>
public class EllipticSolver<TSpace, TBoundary, TOps>(
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

    public double[] Solve(
        EllipticProblem<TSpace> problem,
        ISolver.Params solverParams = new()
    )
    {
        const double time = 0.0; // Время равно 0, так как задача стационарная
        var mesh = (IMeshWithBoundaries<TSpace, TBoundary>)problem.Mesh;
        var dofManager = DofManager.NumerateDof(mesh, problem.BoundaryConditions);
        var assembler = new Assembler<TSpace, TBoundary, TOps>(mesh, dofManager, _matrixFactory, _integrator);

        // Учитываем условия Дирихле
        assembler.CalculateFixedElements(problem.BoundaryConditions, time, _algebraicSolver, solverParams);

        // Сборка глобальной матрицы
        // Добавляем матрицу жесткости G
        assembler.CalculateStiffness(matId => problem.Materials[matId].Lambda);

        // Добавляем матрицу масс M 
        assembler.CalculateMass(matId => problem.Materials[matId].Gamma);

        // Добавляем вклад от 3го краевого в матрицу
        assembler.CalculateRobinMassContribution(problem.BoundaryConditions, time);

        // Сборка правой части уравнения 
        // Добавляем источники (f)
        assembler.CalculateLoad(matId => problem.Materials[matId].Source);
        
        // Добавляем потоки (условия Неймана и Робина)
        assembler.CalculateBoundaryLoadContribution(problem.BoundaryConditions, time);

        // Решение СЛАУ
        var freeSolution = new double[assembler.DofManager.FreeDofCount];
        _algebraicSolver.Matrix = assembler.Matrix;
        _algebraicSolver.Solve(assembler.RhsVector, freeSolution, solverParams);

        // Формирование итогового ответа 
        return GetFullSolution(freeSolution, assembler.FixedSolution, assembler.DofManager);
    }

    private static double[] GetFullSolution(ReadOnlySpan<double> freeSolution, ReadOnlySpan<double> fixedSolution, DofManager dofManager)
    {
        var result = new double[dofManager.TotalDofCount];
        freeSolution.CopyTo(result);
        fixedSolution.CopyTo(result.AsSpan(start: dofManager.FreeDofCount));
        return result;
    }
}
