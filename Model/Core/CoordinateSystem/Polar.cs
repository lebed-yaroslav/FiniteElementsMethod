using Telma;

namespace Model.Core.CoordinateSystem;


public sealed class PolarCoordinateSystem : ICoordinateTransform<Vector2D, Vector2D>
{
    public static readonly PolarCoordinateSystem Instance = new();
    private PolarCoordinateSystem() { }

    public static bool IsLinear => false;

    public Vector2D Transform(Vector2D sourcePoint)
        => new(sourcePoint.Norm, Math.Atan2(sourcePoint.Y, sourcePoint.X));

    public Vector2D InverseTransform(Vector2D targetPoint)
    {
        (var r, var theta) = targetPoint;
        return new(r * Math.Cos(theta), r * Math.Sin(theta));
    }

    public double Jacobian(Vector2D targetPoint) => targetPoint.X; // r

    public IJacobyMatrix<Vector2D, Vector2D> InverseJacoby()
        => InversePolarJacobyMatrix.Instance;
}


public sealed class InversePolarJacobyMatrix : IJacobyMatrix<Vector2D, Vector2D>
{
    public static readonly InversePolarJacobyMatrix Instance = new();
    private InversePolarJacobyMatrix() { }

    public static bool IsConstant => false;

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
                (1, 0) => -Math.Sin(theta) / r,
                (1, 1) => Math.Cos(theta) / r,
                _ => throw new IndexOutOfRangeException()
            };

        }
    }

    public double Det(Vector2D sourcePoint)
        => 1 / sourcePoint.X; // 1 / r
}
