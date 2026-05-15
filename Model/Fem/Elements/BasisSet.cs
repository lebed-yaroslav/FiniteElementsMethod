using Model.Fem.Basis;
using Model.Fem.Integrator;
using Telma;
using Telma.Extensions;

namespace Model.Fem.Elements;


public interface IBasisSet<TSpace> where TSpace : IVectorBase<TSpace>
{
    IEnumerable<Quadratures.Node<TSpace>> Quadratures { get; }
    ReadOnlySpan<IPolynomial<TSpace>> Basis { get; }
}


public readonly struct BasisSet<TSpace>(
    Func<IEnumerable<Quadratures.Node<TSpace>>> quadratures,
    params IPolynomial<TSpace>[] basis
) : IBasisSet<TSpace> where TSpace : IVectorBase<TSpace>
{
    public IEnumerable<Quadratures.Node<TSpace>> Quadratures => quadratures();
    private readonly IPolynomial<TSpace>[] _basis = basis;
    public ReadOnlySpan<IPolynomial<TSpace>> Basis => _basis;
}


public sealed class MutableBasisSet<TSpace>(
    Func<IEnumerable<Quadratures.Node<TSpace>>> quadratures,
    params IPolynomial<TSpace>[] basis
) : IBasisSet<TSpace>
    where TSpace : IVectorBase<TSpace>
{
    public IEnumerable<Quadratures.Node<TSpace>> Quadratures => quadratures();
    private IPolynomial<TSpace>[] _basis = basis;
    public ReadOnlySpan<IPolynomial<TSpace>> Basis => _basis;
    public Span<IPolynomial<TSpace>> MutableBasis => _basis;
}
