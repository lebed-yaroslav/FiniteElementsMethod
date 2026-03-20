global using Material1D = Model.Model.Material<Telma.Vector1D>;
global using Material2D = Model.Model.Material<Telma.Vector2D>;
global using Material3D = Model.Model.Material<Telma.Vector3D>;
using Telma.Extensions;

namespace Model.Model;


public sealed record Material<TSpace>(
    Func<TSpace, double, double> Lambda,
    Func<TSpace, double, double> Xi,
    Func<TSpace, double, double> Sigma,
    Func<TSpace, double, double> Source
) where TSpace : IVectorBase<TSpace>;
