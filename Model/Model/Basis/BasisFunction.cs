using Telma;

namespace Model.Model.Basis;

public interface IBasisFunction
{
    double Value(Vector2D point);
    Vector2D Derivatives(Vector2D point);
}

public sealed class OrientedBasisFunction(IBasisFunction basis) : IBasisFunction
{
    private int _sign = 1;
    private readonly IBasisFunction _basis = basis;

    public bool IsOrientationFlipped { get => _sign == -1; set => _sign = value ? -1 : 1; }

    public double Value(Vector2D point) => _sign * _basis.Value(point);
    public Vector2D Derivatives(Vector2D point) => _sign * _basis.Derivatives(point);
}
