using Model.Core.Matrix;
using Model.Core.Solver;
using Model.Model.Elements;
using Model.Model.Integrator;
using Model.Model.Mesh;
using Telma.Extensions;

namespace Model.Model.Assembly;


public sealed record Assembler<TSpace, TBoundary>(
    IMeshWithBoundaries<TSpace, TBoundary> Mesh,
    DofManager DofManager,
    IMatrixFactory MatrixFactory,
    ISolver Solver,
    IIntegrator<TSpace, TBoundary> Integrator
)
    where TSpace : IVectorBase<TSpace>
    where TBoundary : IVectorBase<TBoundary>
{
    private readonly double[] _solution = new double[DofManager.TotalDofCount];

    public void CalculateFixedElements(BoundaryCondition<TSpace>[] boundaryConditions, double time)
    {
        var adjacencyList = PortraitGenerator.CreateAdjacencyList(
            Mesh.AllElementsDof,
            minDofIndex: DofManager.FreeDofCount,
            maxDofIndex: DofManager.TotalDofCount - 1
        );

        var matrix = MatrixFactory.Create(adjacencyList);
        var rhsVector = new double[DofManager.FixedDofCount];

        foreach (var element in Mesh.BoundaryElements)
        {
            if (boundaryConditions[element.BoundaryIndex] is not BoundaryCondition<TSpace>.Dirichlet bc)
                continue;

            CalculateFixedElementMass(element, matrix);
            CalculateFixedElementLoad(element, boundaryValue: p => bc.Value(p, time), rhsVector);
        }

        Solver.Matrix = matrix;
        Solver.Solve(rhsVector, _solution.AsSpan(start: DofManager.FreeDofCount));
    }

    private void CalculateFixedElementMass(IBoundaryElement<TSpace, TBoundary> element, IGlobalMatrix matrix)
    {
        var mass = Integrator.CalculateLocalMass(element, _ => 1.0);

        int n = element.DOF.Count;
        var indices = new int[n];
        for (int i = 0; i < n; i++)
            indices[i] = element.DOF.Dof[i] - DofManager.FreeDofCount;

        matrix.AddLocalMatrix(mass, indices);
    }

    private void CalculateFixedElementLoad(
        IBoundaryElement<TSpace, TBoundary> element,
        Func<TSpace, double> boundaryValue,
        Span<double> rhsVector
    )
    {
        int n = element.DOF.Count;
        var load = new double[n];
        Integrator.CalculateLocalLoad(element, boundaryValue, load);

        for (int i = 0; i < n; ++i)
        {
            int gi = element.DOF.Dof[i] - DofManager.FreeDofCount;
            if (gi >= 0) rhsVector[gi] = load[i];
        }
    }
}
