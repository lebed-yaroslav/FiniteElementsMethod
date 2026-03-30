using Telma;
using Telma.Extensions;

namespace Model.Core.CoordinateSystem;

public interface IMatrixOperations<TRows, TCols, TSelf>
    where TRows : IVectorBase<TRows>
    where TCols : IVectorBase<TCols>
    where TSelf : IMatrixOperations<TRows, TCols, TSelf>
{
    static abstract double Det(ConstantJacobyMatrix<TRows, TCols, TSelf> matrix);
    static abstract TRows Mul(ConstantJacobyMatrix<TRows, TCols, TSelf> lhs, TCols rhs);
    static abstract TCols Mul(TRows lhs, ConstantJacobyMatrix<TRows, TCols, TSelf> rhs);
    static abstract ConstantJacobyMatrix<TRows, TCols, TSelf> Inverse(ConstantJacobyMatrix<TRows, TCols, TSelf> matrix);
}

public static class MatrixOperations
{
    public static readonly IMatrixOperations<Vector1D, Vector1D, Ops1X1> O1X1 = new Ops1X1();
    public static readonly IMatrixOperations<Vector2D, Vector2D, Ops2X2> O2X2 = new Ops2X2();
    public static readonly IMatrixOperations<Vector3D, Vector3D, Ops3X3> O3X3 = new Ops3X3();

    public static readonly IMatrixOperations<Vector1D, Vector2D, Ops1X2> O1X2 = new Ops1X2();
    public static readonly IMatrixOperations<Vector2D, Vector3D, Ops2X3> O2X3 = new Ops2X3();

    public sealed class Ops1X1 : IMatrixOperations<Vector1D, Vector1D, Ops1X1>
    {
        public static double Det(ConstantJacobyMatrix1X1 matrix) => matrix[0, 0];
        public static Vector1D Mul(ConstantJacobyMatrix1X1 lhs, Vector1D rhs) => lhs[0, 0] * rhs.X;
        public static Vector1D Mul(Vector1D lhs, ConstantJacobyMatrix1X1 rhs) => lhs.X * rhs[0, 0];
        public static ConstantJacobyMatrix1X1 Inverse(ConstantJacobyMatrix1X1 matrix) => new(new[,] { { 1.0 / Det(matrix) } });
    }

    public sealed class Ops2X2 : IMatrixOperations<Vector2D, Vector2D, Ops2X2>
    {
        public static double Det(ConstantJacobyMatrix2X2 m) => m[0, 0] * m[1, 1] - m[0, 1] * m[1, 0];
        public static Vector2D Mul(ConstantJacobyMatrix2X2 lhs, Vector2D rhs)
            => new(
                lhs[0, 0] * rhs.X + lhs[0, 1] * rhs.Y,
                lhs[1, 0] * rhs.X + lhs[1, 1] * rhs.Y
            );

        public static Vector2D Mul(Vector2D lhs, ConstantJacobyMatrix2X2 rhs)
            => new(
                lhs.X * rhs[0, 0] + lhs.Y * rhs[1, 0],
                lhs.X * rhs[0, 1] + lhs.Y * rhs[1, 1]
            );

        public static ConstantJacobyMatrix2X2 Inverse(ConstantJacobyMatrix2X2 matrix)
        {
            var det = Det(matrix);
            return new(new[,] {
                {matrix[1, 1] / det, -matrix[0, 1] / det},
                {-matrix[1, 0] / det, matrix[0, 0] / det}
            });
        }
    }

    public sealed class Ops3X3 : IMatrixOperations<Vector3D, Vector3D, Ops3X3>
    {
        public static double Det(ConstantJacobyMatrix3X3 m) =>
            + m[0, 0] * (m[1, 1] * m[2, 2] - m[1, 2] * m[2, 1]) 
            - m[0, 1] * (m[1, 0] * m[2, 2] - m[1, 2] * m[2, 0]) 
            + m[0, 2] * (m[1, 0] * m[2, 1] - m[1, 1] * m[2, 0]);

        public static Vector3D Mul(ConstantJacobyMatrix3X3 lhs, Vector3D rhs)
            => new(
                lhs[0, 0] * rhs.X + lhs[0, 1] * rhs.Y + lhs[0, 2] * rhs.Z,
                lhs[1, 0] * rhs.X + lhs[1, 1] * rhs.Y + lhs[1, 2] * rhs.Z,
                lhs[2, 0] * rhs.X + lhs[2, 1] * rhs.Y + lhs[2, 2] * rhs.Z
            );

        public static Vector3D Mul(Vector3D lhs, ConstantJacobyMatrix3X3 rhs)
            => new(
                lhs.X * rhs[0, 0] + lhs.Y * rhs[1, 0] + lhs.Z * rhs[2, 0],
                lhs.X * rhs[0, 1] + lhs.Y * rhs[1, 1] + lhs.Z * rhs[2, 1],
                lhs.X * rhs[0, 2] + lhs.Y * rhs[1, 2] + lhs.Z * rhs[2, 2]
            );

        public static ConstantJacobyMatrix3X3 Inverse(ConstantJacobyMatrix3X3 matrix)
            => throw new NotImplementedException("TODO: Implement inverse for 3x3 matrix");
    }

    // Segment parametrization
    public sealed class Ops1X2 : IMatrixOperations<Vector1D, Vector2D, Ops1X2>
    {
        public static double Det(ConstantJacobyMatrix1X2 m) => new Vector2D(m[0, 0], m[0, 1]).Norm;

        public static Vector1D Mul(ConstantJacobyMatrix1X2 lhs, Vector2D rhs)
            => lhs[0, 0] * rhs.X + lhs[0, 1] * rhs.Y;

        public static Vector2D Mul(Vector1D lhs, ConstantJacobyMatrix1X2 rhs)
            => new(lhs.X * rhs[0, 0], lhs.X * rhs[0, 1]);

        public static ConstantJacobyMatrix1X2 Inverse(ConstantJacobyMatrix1X2 matrix)
            => throw new NotSupportedException("Inverse can be called only for square matrices");
    }

    // Surface parametrization
    public sealed class Ops2X3 : IMatrixOperations<Vector2D, Vector3D, Ops2X3>
    {
        public static double Det(ConstantJacobyMatrix2X3 m)
            => throw new NotImplementedException("Determinant for surface parametrization is not implemented yet");

        public static Vector2D Mul(ConstantJacobyMatrix2X3 lhs, Vector3D rhs)
            => new(
                 lhs[0, 0] * rhs.X + lhs[0, 1] * rhs.Y + lhs[0, 2] * rhs.Z,
                lhs[1, 0] * rhs.X + lhs[1, 1] * rhs.Y + lhs[1, 2] * rhs.Z
            );

        public static Vector3D Mul(Vector2D lhs, ConstantJacobyMatrix2X3 rhs)
            => new(
                lhs.X * rhs[0, 0] + lhs.Y * rhs[1, 0],
                lhs.X * rhs[0, 1] + lhs.Y * rhs[1, 1],
                lhs.X * rhs[0, 2] + lhs.Y * rhs[1, 2]
            );

        public static ConstantJacobyMatrix2X3 Inverse(ConstantJacobyMatrix2X3 matrix)
            => throw new NotSupportedException("Inverse can be called only for square matrices");
    }
}
