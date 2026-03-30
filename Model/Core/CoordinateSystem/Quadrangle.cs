using Model.Model.Basis;
using Telma;

using static Model.Model.Basis.QuadrangleBasis;

namespace Model.Core.CoordinateSystem;


/// <summary>
/// Отображение произвольного четырехугольника с вершинами:
/// <code>
/// P01 -- P11    3 --- 2
///  |      |     |     | - Локальная нумерация
/// P00 -- P10    0 --- 1
/// </code>
/// в квадрат [0; 1] x [0; 1]
/// </summary>
public sealed class QuadrangleCoordinateSystem(
    Vector2D p00,
    Vector2D p10,
    Vector2D p11,
    Vector2D p01
) : ICoordinateTransform<Vector2D, Vector2D>
{
    public static bool IsLinear => false;
    private readonly QuadrangleJacobyMatrix _j = new(p00, p10, p11, p01);

    public Vector2D Transform(Vector2D sourcePoint)
    {
        // Performs Newton method to solve non-linear system
        var targetPoint = new Vector2D(0.5, 0.5); // initial guess
        const int maxIterations = 100;
        const double eps = 1e-13;

        for (int iter = 0; iter < maxIterations; iter++)
        {
            var residual = InverseTransform(targetPoint) - sourcePoint; // r = F(η, ξ) - P

            if (residual.Norm < eps) return targetPoint;

            var invJ = _j.At(targetPoint).Inverse();
            var delta = invJ * residual;  // Δ = J^(-1) * r
            targetPoint -= delta; // (η, ξ) = (η, ξ) - Δ
        }

        return targetPoint;
    }

    public Vector2D InverseTransform(Vector2D targetPoint)
    {
        var n00 = N00.Value(targetPoint);
        var n10 = N10.Value(targetPoint);
        var n11 = N11.Value(targetPoint);
        var n01 = N01.Value(targetPoint);

        return n00 * _j.P00 + n10 * _j.P10 + n11 * _j.P11 + n01 * _j.P01;
    }

    public double Jacobian(Vector2D targetPoint) => _j.Det(targetPoint);
    public IJacobyMatrix<Vector2D, Vector2D> InverseJacoby() => new QuadrangleInverseJacobyMatrix(_j);
}


public sealed record QuadrangleJacobyMatrix(
    Vector2D P00,
    Vector2D P10,
    Vector2D P01,
    Vector2D P11
) : IJacobyMatrix<Vector2D, Vector2D>
{
    public static bool IsConstant => false;

    public double this[int i, int j] =>
        throw new NotSupportedException($"{nameof(QuadrangleInverseJacobyMatrix)} is not constant.");

    public double this[int i, int j, Vector2D targetPoint] => At(targetPoint)[i, j];

    public double Det(Vector2D targetPoint) => At(targetPoint).Det(Vector2D.Zero);

    public ConstantJacobyMatrix2X2 At(Vector2D sourcePoint)
    {
        var dN00 = N00.Derivatives(sourcePoint); // [dξ, dη]
        var dN10 = N10.Derivatives(sourcePoint); // [dξ, dη]
        var dN11 = N11.Derivatives(sourcePoint); // [dξ, dη]
        var dN01 = N01.Derivatives(sourcePoint); // [dξ, dη]

        var dx = dN00 * P00.X + dN10 * P10.X + dN11 * P11.X + dN01 * P01.X; // [dξ, dη]
        var dy = dN00 * P00.Y + dN10 * P10.Y + dN11 * P11.Y + dN01 * P01.Y; // [dξ, dη]

        return new(new[,]
        {
            {dx.X,  dx.Y},
            {dy.X,  dy.Y}
        });
    }
}


public sealed class QuadrangleInverseJacobyMatrix(
    QuadrangleJacobyMatrix j
) : IJacobyMatrix<Vector2D, Vector2D>
{
    private readonly QuadrangleJacobyMatrix _j = j;
    public static bool IsConstant => false;

    public double Det(Vector2D sourcePoint) => 1.0 / j.Det(sourcePoint);
    public double this[int i, int j] => throw new NotSupportedException($"{nameof(QuadrangleInverseJacobyMatrix)} is not constant.");
    public double this[int i, int j, Vector2D sourcePoint]
        => _j.At(sourcePoint).Inverse()[i, j];
}
