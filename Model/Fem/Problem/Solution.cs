using Model.Fem.Elements;
using Model.Fem.Mesh;
using Telma.Extensions;

namespace Model.Fem.Problem;


public sealed class StationarySolution<TSpace, TBoundary>(
    IMeshWithBoundaries<TSpace, TBoundary> mesh,
    double[] coefficients
)
    where TSpace : IVectorBase<TSpace>
    where TBoundary : IVectorBase<TBoundary>
{
    public IMeshWithBoundaries<TSpace, TBoundary> Mesh { get; } = mesh;
    private readonly double[] _coefficients = coefficients;
    public ReadOnlySpan<double> Coefficients => _coefficients;

    private SearchTree<TSpace>? _searchTree = null;

    public StationarySolution<TSpace, TBoundary> WithCoefficients(double[] coefficients)
        => new(Mesh, coefficients) { _searchTree = _searchTree };

    public void BuildSearchTree() => _searchTree ??= SearchTree<TSpace>.BuildFor(Mesh);

    public double Evaluate(TSpace point, CoordinateSpace coordSpace = CoordinateSpace.Mesh)
    {
        point = coordSpace switch
        {
            CoordinateSpace.Physical => Mesh.CoordinateSystem.Transform(point),
            CoordinateSpace.Mesh => point,
            _ => throw new NotSupportedException()
        };
        _searchTree ??= SearchTree<TSpace>.BuildFor(Mesh);
        var element = _searchTree.FindElementAt(point) ??
            throw new ArgumentException("Failed to find element at the point", nameof(point));
        return element.Evaluate(_coefficients, point);
    }

    /// <summary>
    /// Numerically calculates L2 norm difference with <paramref name="analyticalSolution"/>
    /// </summary>
    /// <param name="analyticalSolution">Function defined in mesh-space coordinates</param>
    public double Difference(Func<TSpace, double> analyticalSolution)
    {
        double totalDifference = 0;
        foreach (var element in Mesh.FiniteElements)
        {
            var masterCs = element.MasterElementCoordinateSystem;

            double elementDifference = 0;
            foreach (var q in element.Quadratures)
            {
                var mp = masterCs.InverseTransform(q.Point); // mesh-space point
                var analyticalValue = analyticalSolution(mp);
                var femValue = element.EvaluateAtElementSpace(_coefficients, q.Point);
                var difference = analyticalValue - femValue;
                elementDifference += q.Weight * difference * difference * Math.Abs(masterCs.Jacobian(q.Point));
            }
            totalDifference += elementDifference;
        }
        return Math.Sqrt(totalDifference);
    }
}
