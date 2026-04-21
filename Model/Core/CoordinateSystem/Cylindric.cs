using Telma;

namespace Model.Core.CoordinateSystem
{
    public sealed class CylindricCoordinateSystem : ICoordinateTransform<Vector2D, Vector2D>
    {
        public static readonly CylindricCoordinateSystem Instance = new();
        private CylindricCoordinateSystem() { }

        public static bool IsLinear => false;

        public Vector2D Transform(Vector2D sourcePoint)
            => sourcePoint;

        public Vector2D InverseTransform(Vector2D targetPoint) => targetPoint;

        public double Jacobian(Vector2D targetPoint)
        {
            return targetPoint.X;
        }

        public IJacobyMatrix<Vector2D, Vector2D> InverseJacoby() => InverseCylindricJacobyMatrix.Instance;

    }

    public sealed class InverseCylindricJacobyMatrix : IJacobyMatrix<Vector2D, Vector2D>
    {
        public static readonly InverseCylindricJacobyMatrix Instance = new();
        private InverseCylindricJacobyMatrix() { }

        public static bool IsConstant => false;

        public double this[int i, int j] =>
            throw new NotSupportedException($"{nameof(InverseCylindricJacobyMatrix)} is not constant");

        public double this[int i, int j, Vector2D sourcePoint] => (i == j) ? 1.0 : 0.0;
        public double Det(Vector2D sourcePoint) => 1.0 / sourcePoint.X;
    }
}

