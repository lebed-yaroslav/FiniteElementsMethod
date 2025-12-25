using Model.Core.Matrix;
using Telma;

namespace Model.Core.CoordinateSystem;

public interface ICoordinateSystem2D
{
    Vector2D ToLocal(Vector2D globalPoint);
    Vector2D ToGlobal(Vector2D localPoint);

    bool IsJacobianConstant { get; }
    double Jacobian(Vector2D localPoint);

    IJacobyMatrix InverseJacoby();
}

public interface IJacobyMatrix : IMatrix {
    int IMatrix.Size => 2; // FIXME: Temporary workaround

    static abstract bool IsConstant { get; }

    double Det(Vector2D sourcePoint);
    double this[int i, int j, Vector2D sourcePoint] { get; }
    double this[int i, int j] { get; }

    public static Vector2D operator *(IJacobyMatrix j, Vector2D rhs) =>
        new(j[0, 0] * rhs.X + j[0, 1] * rhs.Y, j[1, 0] * rhs.X + j[1, 1] * rhs.Y);

    public IJacobyMatrix MulAt(Vector2D lp, IJacobyMatrix rhs, Vector2D rp) => new ConstantJacobyMatrix(new[,] {
        {
            this[0, 0, lp] * rhs[0, 0, rp] + this[0, 1, lp] * rhs[1, 0, rp],
            this[0, 0, lp] * rhs[0, 1, rp] + this[0, 1, lp] * rhs[1, 1, rp]
        },
        {
            this[1, 0, lp] * rhs[0, 0, rp] + this[1, 1, lp] * rhs[1, 0, rp],
            this[1, 0, lp] * rhs[0, 1, rp] + this[1, 1, lp] * rhs[1, 1, rp]
        }
    });
}

public sealed class ConstantJacobyMatrix(double[,] j) : IJacobyMatrix
{
    public static bool IsConstant => true;

    private readonly double[,] _j = j;
    public double Det(Vector2D _) => _j[0, 0] * _j[1, 1] - _j[0, 1] * _j[1, 0];
    public double this[int i, int j, Vector2D _] => _j[i, j];
    public double this[int i, int j] => _j[i, j];

    public ConstantJacobyMatrix Inverse() {
        var detJ = Det(Vector2D.Zero);
        return new(new[,] {
            {_j[1, 1] / detJ, -_j[0, 1] / detJ},
            {-_j[1, 0] / detJ, _j[0, 0] / detJ}
        });
    }
}
