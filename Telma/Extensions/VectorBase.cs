using System.Numerics;

namespace Telma.Extensions;

/// <summary>Defines common n-dimensional vector traits</summary>
/// <typeparam name="TSelf">
/// One of <see cref="Vector1D"/>, <see cref="Vector2D"/>, <see cref="Vector3D"/>
/// </typeparam>
public interface IVectorBase<TSelf> :
    IUnaryNegationOperators<TSelf, TSelf>,
    IAdditionOperators<TSelf, TSelf, TSelf>,
    ISubtractionOperators<TSelf, TSelf, TSelf>,
    IMultiplyOperators<TSelf, double, TSelf>,
    IDivisionOperators<TSelf, double, TSelf>
    where TSelf : IVectorBase<TSelf>
{
    static abstract int Dimensions { get; }
    static abstract TSelf Zero { get; }
}
