using Telma;

namespace Model.Core.CoordinateSystem;

public sealed record BarycentricCoordinateSystem : ICoordinateTransform<Vector2D, Vector2D>
{
    public Vector2D A { get; }
    public Vector2D B { get; }
    public Vector2D C { get; }

    public readonly ConstantJacobyMatrix2D J;

    private readonly Lazy<ConstantJacobyMatrix2D> _invJ;

    public static bool IsLinear => true;

    public BarycentricCoordinateSystem(Vector2D a, Vector2D b, Vector2D c)
    {
        A = a; B = b; C = c;
        J = new(new[,] {
            {B.X - A.X, C.X - A.X},
            {B.Y - A.Y, C.Y - A.Y}
        });
        _invJ = new(() => J.Inverse());
    }

    public Vector2D Transform(Vector2D sourcePoint)
    {
        var invJ = _invJ.Value;
        var delta = sourcePoint - A;
        return invJ * delta;
    }

    public Vector2D InverseTransform(Vector2D targetPoint)
    {
        (var xi, var eta) = targetPoint;
        return A * (1 - xi - eta) + B * xi + C * eta;
    }

    public double Jacobian(Vector2D targetPoint) => J.Det(targetPoint);
    public IJacobyMatrix<Vector2D, Vector2D> InverseJacoby() => _invJ.Value;
}
