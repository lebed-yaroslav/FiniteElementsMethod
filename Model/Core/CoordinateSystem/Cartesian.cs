using Model.Core.CoordinateSystem;
using Telma;

namespace Model.Core.CoordinateSystems;

public sealed class CartesianCoordinateSystem : ICoordinateSystem2D
{
    public static readonly CartesianCoordinateSystem Instance = new();
    private CartesianCoordinateSystem() { }

    public Vector2D ToLocal(Vector2D globalPoint) => globalPoint;
    public Vector2D ToGlobal(Vector2D localPoint) => localPoint;
    public bool IsJacobianConstant => true;
    public double Jacobian(Vector2D localPoint) => 1.0;

    public IJacobyMatrix InverseJacoby() => IdentityJacobyMatrix.Instance;
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
