using Telma;

namespace Model.Model.Basis;

public interface IBasisFunction
{
    double Value(Vector2D point);
    Vector2D Derivatives(Vector2D point);
}

public struct OrientedBasisFunction(IBasisFunction basis) : IBasisFunction
{
    private int _sign = 1;
    private IBasisFunction _basis = basis;

    public bool IsOrientationFlipped
    {
        readonly get => _sign == -1;
        set => _sign = value ? -1 : 1;
    }

    public readonly double Value(Vector2D point) => _sign * _basis.Value(point);
    public readonly Vector2D Derivatives(Vector2D point) => _sign * _basis.Derivatives(point);
}
