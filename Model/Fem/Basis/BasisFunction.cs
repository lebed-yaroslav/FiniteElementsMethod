global using IBasisFunction1D = Model.Fem.Basis.IBasisFunction<Telma.Vector1D>;
global using IBasisFunction2D = Model.Fem.Basis.IBasisFunction<Telma.Vector2D>;
global using IBasisFunction3D = Model.Fem.Basis.IBasisFunction<Telma.Vector3D>;

global using OrientedBasisFunction1D = Model.Fem.Basis.OrientedBasisFunction<Telma.Vector1D>;
global using OrientedBasisFunction2D = Model.Fem.Basis.OrientedBasisFunction<Telma.Vector2D>;
global using OrientedBasisFunction3D = Model.Fem.Basis.OrientedBasisFunction<Telma.Vector3D>;

global using OrientedPolynomialBasisFunction1D = Model.Fem.Basis.OrientedPolynomialBasisFunction<Telma.Vector1D>;
global using OrientedPolynomialBasisFunction2D = Model.Fem.Basis.OrientedPolynomialBasisFunction<Telma.Vector2D>;

using Telma;
using Telma.Extensions;
using Model.Fem.Integrator;

namespace Model.Fem.Basis;


public interface IBasisFunction<TSpace> where TSpace : IVectorBase<TSpace>
{
    double Value(TSpace point);
    TSpace Derivatives(TSpace point);
}

public class OrientedBasisFunction<TSpace>(IBasisFunction<TSpace> basis) :
    IBasisFunction<TSpace>
    where TSpace : IVectorBase<TSpace>
{
    private int _sign = 1;
    protected readonly IBasisFunction<TSpace> _basis = basis;

    public bool IsOrientationFlipped { get => _sign == -1; set => _sign = value ? -1 : 1; }

    public double Value(TSpace point) => _basis.Value(point) * _sign;
    public TSpace Derivatives(TSpace point) => _basis.Derivatives(point) * _sign;
}

public class OrientedPolynomialBasisFunction<TSpace>(IPolynomial<TSpace> basis) : 
    OrientedBasisFunction<TSpace>(basis), IPolynomial<TSpace>
    where TSpace : IVectorBase<TSpace>
{
    private readonly IPolynomial<TSpace> _basisPoly = basis;

    public IPolynomial<TSpace> Poly => IsOrientationFlipped ? _basisPoly.Scale(-1.0) : _basisPoly;

    public int Degree => _basisPoly.Degree;

    public void Delete_Nulls() => _basisPoly.Delete_Nulls();
    public IPolynomial<TSpace> Scale(double scal) => _basisPoly.Scale(scal);
}

/// <summary>
/// Тензорное произведение двух одномерных базисов
/// </summary>
public static class TensorBasis2D
{
    public static Polynomial2D CreatePoly(Polynomial1D bX, Polynomial1D bY)
    {
        return Polynomial2D.TensorMult(bX, bY);
    }
}
