namespace Telma.Extensions;


public interface IVectorTraits<TSelf>
    where TSelf : IVectorTraits<TSelf>
{
    static abstract int Dimensions { get; }
    static abstract TSelf Zero { get; }
}
