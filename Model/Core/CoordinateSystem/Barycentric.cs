using Model.Core.CoordinateSystem;
using Telma;

namespace Model.Core.CoordinateSystems;

public sealed record BarycentricCoordinateSystem : ICoordinateSystem2D
{
    public Vector2D A { get; }
    public Vector2D B { get; }
    public Vector2D C { get; }

    public readonly ConstantJacobyMatrix J;

    private readonly Lazy<ConstantJacobyMatrix> _invJ;

    public BarycentricCoordinateSystem(Vector2D a, Vector2D b, Vector2D c)
    {
        A = a; B = b; C = c;
        J = new(new[,] {
            {B.X - A.X, C.X - A.X},
            {B.Y - A.Y, C.Y - A.Y}
        });
        _invJ = new(() => J.Inverse());
    }

    public Vector2D ToGlobal(Vector2D localPoint)
    {
        (var xi, var eta) = localPoint;
        return A * (1 - xi - eta) + B * xi + C * eta;
    }

    public Vector2D ToLocal(Vector2D globalPoint)
    {
        var invJ = _invJ.Value;
        var delta = globalPoint - A;
        return (invJ as IJacobyMatrix) * delta;
    }

    public bool IsJacobianConstant => true;
    public double Jacobian(Vector2D point) => J.Det(point);
    public IJacobyMatrix InverseJacoby() => _invJ.Value;
}
