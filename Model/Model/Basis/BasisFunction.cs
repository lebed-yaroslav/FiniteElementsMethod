global using IBasisFunction1D = Model.Model.Basis.IBasisFunction<Telma.Vector1D>;
global using IBasisFunction2D = Model.Model.Basis.IBasisFunction<Telma.Vector2D>;
global using IBasisFunction3D = Model.Model.Basis.IBasisFunction<Telma.Vector3D>;

global using OrientedBasisFunction1D = Model.Model.Basis.OrientedBasisFunction<Telma.Vector1D>;
global using OrientedBasisFunction2D = Model.Model.Basis.OrientedBasisFunction<Telma.Vector2D>;
global using OrientedBasisFunction3D = Model.Model.Basis.OrientedBasisFunction<Telma.Vector3D>;

using Telma;
using Telma.Extensions;


namespace Model.Model.Basis;


public interface IBasisFunction<TSpace> where TSpace : IVectorBase<TSpace>
{
    double Value(TSpace point);
    TSpace Derivatives(TSpace point);
}

public sealed class OrientedBasisFunction<TSpace>(IBasisFunction<TSpace> basis) :
    IBasisFunction<TSpace>
    where TSpace : IVectorBase<TSpace>
{
    private int _sign = 1;
    private readonly IBasisFunction<TSpace> _basis = basis;

    public bool IsOrientationFlipped { get => _sign == -1; set => _sign = value ? -1 : 1; }

    public double Value(TSpace point) => _basis.Value(point) * _sign;
    public TSpace Derivatives(TSpace point) => _basis.Derivatives(point) * _sign;
}


/// <summary>
/// Тензорное произведение двух одномерных базисов
/// </summary>
public readonly struct TensorBasis2D(IBasisFunction1D bX, IBasisFunction1D bY) : IBasisFunction2D
{
    public double Value(Vector2D p) => bX.Value(p.X) * bY.Value(p.Y);

    public Vector2D Derivatives(Vector2D p)
    {
        double x = bX.Value(p.X);
        double y = bY.Value(p.Y);
        double dx = bX.Derivatives(p.X);
        double dy = bY.Derivatives(p.Y);
        return new(dx * y, x * dy);
    }
}
