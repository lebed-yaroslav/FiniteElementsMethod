using Model.Core.CoordinateSystem;
using Telma;

namespace Model.Core.CoordinateSystems;

public sealed record CartesianCoordinateSystem : ICoordinateSystem2D
{
    public Vector2D P00 { get; }
    public Vector2D P10 { get; }
    public Vector2D P11 { get; }
    public Vector2D P01 { get; }

    public readonly ConstantJacobyMatrix J;

    private readonly Lazy<ConstantJacobyMatrix> _invJ;
    public CartesianCoordinateSystem(Vector2D p00, Vector2D p10, Vector2D p11, Vector2D p01)
    {
        P00 = p00;
        P10 = p10;
        P11 = p11;
        P01 = p01;

        J = new(new[,] {
            {P10.X - P00.X, P01.X - P00.X},
            {P10.Y - P00.Y, P01.Y - P00.Y}
        });

        _invJ = new(() => J.Inverse());
    }

    private static (double N00, double N10, double N11, double N01) ShapeFunctions(double η, double ξ)
    {
        double N00 = (1 - η) * (1 - ξ);
        double N10 = η * (1 - ξ);
        double N11 = η * ξ;
        double N01 = (1 - η) * ξ;
        return (N00, N10, N11, N01);
    }
    public Vector2D ToLocal(Vector2D globalPoint)
    {
        var invJ = _invJ.Value;
        var delta = globalPoint - P00;
        return (invJ as IJacobyMatrix) * delta;

    }
    public Vector2D ToGlobal(Vector2D localPoint)
    {
        var (η, ξ) = localPoint;

        var (N00, N10, N11, N01) = ShapeFunctions(η, ξ);

        return N00 * P00 + N10 * P10 + N11 * P11 + N01 * P01; 
    }
    public bool IsJacobianConstant => true;
    public double Jacobian(Vector2D point) => J.Det(point);

    public IJacobyMatrix InverseJacoby() => _invJ.Value;
}

public sealed class IdentityJacobyMatrix : IJacobyMatrix
{
    public static readonly IdentityJacobyMatrix Instance = new();
    private IdentityJacobyMatrix() { }

    public static bool IsConstant => true;
    public double Det(Vector2D _) => 1.0;
    public double this[int i, int j] => (i == j) ? 1.0 : 0.0;
    public double this[int i, int j, Vector2D _] => this[i, j];
}
