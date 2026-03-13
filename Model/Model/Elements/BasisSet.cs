using Model.Model.Basis;
using Telma;
using Telma.Extensions;

namespace Model.Model.Elements;


public interface IBasisSet<TSpace> where TSpace : IVectorBase<TSpace>
{
    IEnumerable<Quadratures.Node<TSpace>> Quadratures { get; }
    ReadOnlySpan<IBasisFunction<TSpace>> Basis { get; }
}


public interface IBasisSet2D : IBasisSet<Vector2D>;


public readonly struct BasisSet<TSpace>(
    Func<IEnumerable<Quadratures.Node<TSpace>>> quadratures,
    params IBasisFunction<TSpace>[] basis
) : IBasisSet<TSpace> where TSpace : IVectorBase<TSpace>
{
    public IEnumerable<Quadratures.Node<TSpace>> Quadratures => quadratures();
    private readonly IBasisFunction<TSpace>[] _basis = basis;
    public ReadOnlySpan<IBasisFunction<TSpace>> Basis => _basis;
}


public sealed class MutableBasisSet<TSpace>(
    Func<IEnumerable<Quadratures.Node<TSpace>>> quadratures,
    params IBasisFunction<TSpace>[] basis
) : IBasisSet<TSpace>
    where TSpace : IVectorBase<TSpace>
{
    public IEnumerable<Quadratures.Node<TSpace>> Quadratures => quadratures();
    private IBasisFunction<TSpace>[] _basis = basis;
    public ReadOnlySpan<IBasisFunction<TSpace>> Basis => _basis;
    public Span<IBasisFunction<TSpace>> MutableBasis => _basis;
}
