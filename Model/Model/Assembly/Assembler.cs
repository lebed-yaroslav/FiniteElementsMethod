using Model.Core.CoordinateSystem;
using Model.Core.Matrix;
using Model.Core.Solver;
using Model.Model.Elements;
using Model.Model.Integrator;
using Model.Model.Mesh;
using Telma.Extensions;

namespace Model.Model.Assembly;


public sealed record Assembler<TSpace, TBoundary, TOps>(
    IMeshWithBoundaries<TSpace, TBoundary> Mesh,
    DofManager DofManager,
    IMatrixFactory MatrixFactory,
    IIntegrator<TSpace, TBoundary, TOps> Integrator
)
    where TSpace : IVectorBase<TSpace>
    where TBoundary : IVectorBase<TBoundary>
    where TOps : IMatrixOperations<TSpace, TSpace, TOps>
{
    private readonly double[] _rhsVector = new double[DofManager.FreeDofCount];
    private readonly double[] _fixedSolution = new double[DofManager.FixedDofCount];

    public IGlobalMatrix Matrix { get; } = MatrixFactory.Create(PortraitGenerator.CreateAdjacencyList(
        Mesh.AllElementsDof, minDofIndex: 0, maxDofIndex: DofManager.FreeDofCount - 1
    ));
    public ReadOnlySpan<double> RhsVector => _rhsVector;
    public ReadOnlySpan<double> FixedSolution => _fixedSolution;

    public void ResetFixedElements() => Array.Fill(_fixedSolution, 0);
    public void ResetSystemMatrix() => Matrix.Fill(0);
    public void ResetLoadVector() => Array.Fill(_rhsVector, 0);

    /// <summary>
    /// 
    /// </summary>
    /// <param name="boundaryConditions"></param>
    /// <param name="time"></param>
    /// <param name="paramz"></param>
    public void CalculateFixedElements(BoundaryCondition<TSpace>[] boundaryConditions, double time, ISolver solver, ISolver.Params paramz = default)
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

        solver.Matrix = matrix;
        solver.Solve(rhsVector, _fixedSolution, paramz);
    }

    public void CalculateStiffness(Func<int, Func<TSpace, double>> lambdaByMaterialIndex)
    {
        foreach (var element in Mesh.FiniteElements)
        {
            var lambda = lambdaByMaterialIndex(element.MaterialIndex);
            var stiffness = Integrator.CalculateLocalStiffness(element, lambda);
            AddLocalMatrixWithFixedLoadContribution(stiffness, element.DOF);
        }
    }

    public void CalculateMass(Func<int, Func<TSpace, double>> gammaByMaterialIndex)
    {
        foreach (var element in Mesh.FiniteElements)
        {
            var gamma = gammaByMaterialIndex(element.MaterialIndex);
            var stiffness = Integrator.CalculateLocalMass((IFiniteElementBase<TSpace, TBoundary>)element, gamma);
            AddLocalMatrixWithFixedLoadContribution(stiffness, element.DOF);
        }
    }

    public void CalculateLoad(Func<int, Func<TSpace, double>> sourceByMaterialIndex)
    {
        foreach (var element in Mesh.FiniteElements)
        {
            var source = sourceByMaterialIndex(element.MaterialIndex);
            var load = new double[element.DOF.Count];
            Integrator.CalculateLocalLoad(element, source, load);
            AddLocalLoad(load, element.DOF);
        }
    }

    public void CalculateBoundaryLoadContribution(BoundaryCondition<TSpace>[] boundaryConditions, double time)
    {
        foreach (var element in Mesh.BoundaryElements)
        {
            switch(boundaryConditions[element.BoundaryIndex])
            {
                case BoundaryCondition<TSpace>.Neumann(var flux):
                    var load2 = new double[element.DOF.Count];
                    Integrator.CalculateLocalLoad(element, p => flux(p, time), load2);
                    AddLocalLoad(load2, element.DOF);
                    break;
                case BoundaryCondition<TSpace>.Robin(var beta, var uBeta):
                    var load3 = new double[element.DOF.Count];
                    Integrator.CalculateLocalLoad(element, p => beta(p, time) * uBeta(p, time), load3);
                    AddLocalLoad(load3, element.DOF);
                    break;
            }
        }
    }

    public void CalculateRobinMassContribution(BoundaryCondition<TSpace>[] boundaryConditions, double time)
    {
        foreach (var element in Mesh.BoundaryElements)
        {
            if (boundaryConditions[element.BoundaryIndex] is not BoundaryCondition<TSpace>.Robin(var beta, var _))
                continue;
            var mass = Integrator.CalculateLocalMass(element, p => beta(p, time));
            AddLocalMatrixWithFixedLoadContribution(mass, element.DOF);
        }
    }

    private void CalculateFixedElementMass(IBoundaryElement<TSpace, TBoundary> element, IGlobalMatrix matrix)
    {
        var mass = Integrator.CalculateLocalMass(element, _ => 1.0);
        var indices = new int[element.DOF.Count];
        DofManager.CreateFixedLocalToGlobalIndexMapping(element.DOF, indices);
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
            if (gi >= 0) rhsVector[gi] = load[i]; // Ignore free dof
        }
    }

    private void AddLocalMatrixWithFixedLoadContribution(LocalMatrix matrix, IDofManager elementDof)
    {
        var indices = new int[elementDof.Count];
        DofManager.CreateFreeLocalToGlobalIndexMapping(elementDof, indices);
        Matrix.AddLocalMatrix(matrix, indices);

        // Add contribution of fixed elements to load vector
        for (int i = 0; i < elementDof.Count; ++i)
        {
            var globalDofI = elementDof.Dof[i];
            if (globalDofI >= DofManager.FreeDofCount) continue; // Skip fixed dof row
            for (int j = 0; j < elementDof.Count; ++j)
            {
                var fixedDofJ = elementDof.Dof[j] - DofManager.FreeDofCount;
                if (fixedDofJ < 0) continue; // Skip free dof column
                _rhsVector[globalDofI] -= matrix[i, j] * _fixedSolution[fixedDofJ];
            }
        }
    }

    private void AddLocalLoad(ReadOnlySpan<double> load, IDofManager elementDof)
    {
        for (int i = 0; i < elementDof.Count; ++i)
        {
            int dof = elementDof.Dof[i];
            if (dof < DofManager.FreeDofCount) // Ignore fixed dof
                _rhsVector[dof] += load[i];
        }
    }
}
