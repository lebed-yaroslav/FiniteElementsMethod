namespace Telma.Extensions;

public interface IVector<TDim> where TDim : IDimensions
{
    static int Dimensions => TDim.Count;
    static abstract IVector<TDim> Zero { get; }

    double Norm { get; }
    double NormSqr { get; }

    ReadOnlySpan<double> AsSpan();
}
