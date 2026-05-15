global using IBasisFunction1D = Model.Fem.Basis.IBasisFunction<Telma.Vector1D>;
global using IBasisFunction2D = Model.Fem.Basis.IBasisFunction<Telma.Vector2D>;
global using IBasisFunction3D = Model.Fem.Basis.IBasisFunction<Telma.Vector3D>;

using Telma;
using Telma.Extensions;
using Model.Fem.Integrator;

namespace Model.Fem.Basis;


public interface IBasisFunction<TSpace> where TSpace : IVectorBase<TSpace>
{
    double Value(TSpace point);
    TSpace Derivatives(TSpace point);
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
