global using IBasisSet1D = Model.Model.Elements.IBasisSet<Telma.Vector1D>;
global using IBasisSet2D = Model.Model.Elements.IBasisSet<Telma.Vector2D>;
global using IBasisSet3D = Model.Model.Elements.IBasisSet<Telma.Vector3D>;

global using BasisSet1D = Model.Model.Elements.BasisSet<Telma.Vector1D>;
global using BasisSet2D = Model.Model.Elements.BasisSet<Telma.Vector2D>;
global using BasisSet3D = Model.Model.Elements.BasisSet<Telma.Vector3D>;

using Model.Model.Basis;
using Telma;
using Telma.Extensions;

namespace Model.Model.Elements;


public interface IBasisSet<TSpace> where TSpace : IVectorBase<TSpace>
{
    IEnumerable<Quadratures.Node<TSpace>> Quadratures { get; }
    ReadOnlySpan<IBasisFunction<TSpace>> Basis { get; }
}


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
