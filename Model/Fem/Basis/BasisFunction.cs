global using IBasisFunction1D = Model.Fem.Basis.IBasisFunction<Telma.Vector1D>;
global using IBasisFunction2D = Model.Fem.Basis.IBasisFunction<Telma.Vector2D>;
global using IBasisFunction3D = Model.Fem.Basis.IBasisFunction<Telma.Vector3D>;

global using IAnalyticalBasisFunction2D = Model.Fem.Basis.IAnalyticalBasisFunction<Telma.Vector2D>;
global using IAnalyticalBasisFunction1D = Model.Fem.Basis.IAnalyticalBasisFunction<Telma.Vector1D>;

global using OrientedBasisFunction1D = Model.Fem.Basis.OrientedBasisFunction<Telma.Vector1D>;
global using OrientedBasisFunction2D = Model.Fem.Basis.OrientedBasisFunction<Telma.Vector2D>;
global using OrientedBasisFunction3D = Model.Fem.Basis.OrientedBasisFunction<Telma.Vector3D>;
using Model.Fem.Integrator;
using Telma;
using Telma.Extensions;

namespace Model.Fem.Basis;


public interface IBasisFunction<TSpace> where TSpace : IVectorBase<TSpace>
{
    double Value(TSpace point);
    TSpace Derivatives(TSpace point);
}
public interface IAnalyticalBasisFunction<TSpace> : IBasisFunction<TSpace> where TSpace : IVectorBase<TSpace>
{
    IPolynomial Polynomial {get;}
}

public sealed class OrientedBasisFunction<TSpace>(IBasisFunction<TSpace> basis) :
    IAnalyticalBasisFunction<TSpace>
    where TSpace : IVectorBase<TSpace>
{
    private int _sign = 1;
    private readonly IBasisFunction<TSpace> _basis = basis;
    private readonly IAnalyticalBasisFunction<TSpace>? _analyticalBasis = basis as IAnalyticalBasisFunction<TSpace>;
    public bool IsOrientationFlipped { get => _sign == -1; set => _sign = value ? -1 : 1; }

    public double Value(TSpace point) => _basis.Value(point) * _sign;
    public TSpace Derivatives(TSpace point) => _basis.Derivatives(point) * _sign;
    public IPolynomial Polynomial
    { 
        get
        {
            if (_analyticalBasis == null)
            {
                throw new InvalidOperationException($"Базисная функция {_basis.GetType().Name} не поддерживает полиномы.");
            }
            var basePolynomial = _analyticalBasis.Polynomial;
            if (!IsOrientationFlipped) return basePolynomial;
            var flippedPolynomial = new Polynomial();
            foreach (var summand in basePolynomial.Summands)
            {
                flippedPolynomial.Summands.Add((summand.p, summand.q, -summand.coeff));
            }
            return flippedPolynomial;
        }
    }
}


/// <summary>
/// Тензорное произведение двух одномерных базисов
/// </summary>
//public readonly struct TensorBasis2D(IBasisFunction1D bX, IBasisFunction1D bY) : IBasisFunction2D
//{
//    public double Value(Vector2D p) => bX.Value(p.X) * bY.Value(p.Y);

//    public Vector2D Derivatives(Vector2D p)
//    {
//        double x = bX.Value(p.X);
//        double y = bY.Value(p.Y);
//        double dx = bX.Derivatives(p.X);
//        double dy = bY.Derivatives(p.Y);
//        return new(dx * y, x * dy);
//    }
//}
public readonly struct TensorBasis2D(IBasisFunction1D bX, IBasisFunction1D bY) : IAnalyticalBasisFunction2D
{
    private readonly IBasisFunction1D _bX = bX;
    private readonly IBasisFunction1D _bY = bY;
    private readonly IAnalyticalBasisFunction1D? _analyticBX = bX as IAnalyticalBasisFunction1D;
    private readonly IAnalyticalBasisFunction1D? _analyticBY = bY as IAnalyticalBasisFunction1D;
    public double Value(Vector2D p) => _bX.Value(p.X) * _bY.Value(p.Y);

    public Vector2D Derivatives(Vector2D p)
    {
        double x = _bX.Value(p.X);
        double y = _bY.Value(p.Y);
        double dx = _bX.Derivatives(p.X);
        double dy = _bY.Derivatives(p.Y);
        return new(dx * y, x * dy);
    }
    public IPolynomial Polynomial
    {
        get
        {
            if (_analyticBX == null || _analyticBY == null)
            {
                string name = _analyticBX == null ? _bX.GetType().Name : _bY.GetType().Name;
                throw new InvalidOperationException($"Компонента тензорного базиса ({name}) не поддерживает полиномы.");
            }
            return ComputeTensorPolynomial(_analyticBX.Polynomial, _analyticBY.Polynomial);
        }
    }

    private static Polynomial ComputeTensorPolynomial(IPolynomial polynomialX, IPolynomial polynomialY)
    {
        var result = new Polynomial();

        // polynomialX хранит степени X в свойстве 'p'
        // polynomialY хранит степени Y в свойстве 'p' (т.к. это тоже 1D полином)
        foreach (var termX in polynomialX.Summands)
        {
            foreach (var termY in polynomialY.Summands)
            {
                   result.Summands.Add((termX.p, termY.p, termX.coeff * termY.coeff));
            }
        }

        return result;
    }
}
