using Model.Core.CoordinateSystem;
using Telma;

namespace Model.Core.CoordinateSystems;

public sealed class PolarCoordinateSystem : ICoordinateSystem2D
{
    public static readonly PolarCoordinateSystem Instance = new();
    private PolarCoordinateSystem() { }

    public Vector2D ToLocal(Vector2D globalPoint)
        => new(globalPoint.Norm, Math.Atan2(globalPoint.Y, globalPoint.X));

    public Vector2D ToGlobal(Vector2D localPoint)
    {
        (var r, var theta) = localPoint;
        return new(r * Math.Cos(theta), r * Math.Sin(theta));
    }

    public bool IsJacobianConstant => false;
    public double Jacobian(Vector2D localPoint) => localPoint.X;

    public IJacobyMatrix InverseJacoby() => IdentityJacobyMatrix.Instance;
}


public sealed class InversePolarJacobyMatrix : IJacobyMatrix
{
    public static readonly InversePolarJacobyMatrix Instance = new();
    private InversePolarJacobyMatrix() { }

    public static bool IsConstant => false;

    public double Det(Vector2D sourcePoint)
        => 1 / sourcePoint.X; // 1 / r

    public double this[int i, int j] =>
        throw new NotSupportedException($"{nameof(InversePolarJacobyMatrix)} is not constant");

    public double this[int i, int j, Vector2D sourcePoint]
    {
        get
        {
            (var r, var theta) = sourcePoint;
            return (i, j) switch
            {
                (0, 0) => Math.Cos(theta),
                (0, 1) => Math.Sin(theta),
                (1, 0) => Math.Sin(theta) / r,
                (1, 1) => Math.Cos(theta) / r,
                _ => throw new IndexOutOfRangeException()
            };

        }
    }

}
