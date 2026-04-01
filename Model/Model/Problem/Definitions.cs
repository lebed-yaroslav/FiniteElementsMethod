using Telma.Extensions;

namespace Model.Model.Problem;


public sealed record Material<TSpace>(
    Func<TSpace, double, double> Lambda,
    Func<TSpace, double, double> Xi,
    Func<TSpace, double, double> Sigma,
    Func<TSpace, double, double> Source
) where TSpace : IVectorBase<TSpace>;


public abstract record BoundaryCondition<TSpace>
    where TSpace : IVectorBase<TSpace>
{
    private BoundaryCondition() { }

    /// <summary>
    /// Краевые условия первого рода вида:
    /// u = g
    /// </summary>
    /// <param name="Value">Функция g</param>
    public sealed record Dirichlet(
        Func<TSpace, double, double> Value
    ) : BoundaryCondition<TSpace>();

    /// <summary>
    /// Краевые условия второго рода вида:
    /// λ(∂u/∂n) = θ
    /// </summary>
    /// <param name="Flux">Функция θ</param>
    public sealed record Neumann(
        Func<TSpace, double, double> Flux
    ) : BoundaryCondition<TSpace>();

    /// <summary>
    /// Краевые условия третьего рода вида:
    /// λ(∂u/∂n) + β * (u - uβ) = 0
    /// </summary>
    /// <param name="Beta">Коэффициент β</param>
    /// <param name="UBeta">Функция β</param>
    public sealed record Robin(
        Func<TSpace, double, double> Beta,
        Func<TSpace, double, double> UBeta
    ) : BoundaryCondition<TSpace>();
}
