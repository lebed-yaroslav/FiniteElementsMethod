using Telma;

namespace Model.Model.Basis;

public interface IBasisFunction<TVector> where TVector : struct
{
    double Value(TVector point);
    TVector Derivatives(TVector point);
}

public interface IBasisFunction2D : IBasisFunction<Vector2D>;

public sealed class OrientedBasisFunction<TVector>(IBasisFunction<TVector> basis) :
    IBasisFunction<TVector>
    where TVector : struct, IMultiplyOperators<TVector, double, TVector>
{
    private int _sign = 1;
    private readonly IBasisFunction<TVector> _basis = basis;

    public bool IsOrientationFlipped { get => _sign == -1; set => _sign = value ? -1 : 1; }

    public double Value(TVector point) => _basis.Value(point) * _sign;
    public TVector Derivatives(TVector point) => _basis.Derivatives(point) * _sign;
}
