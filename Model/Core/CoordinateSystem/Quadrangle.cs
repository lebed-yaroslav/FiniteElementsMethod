using Model.Core.CoordinateSystem;
using Telma;

namespace Model.Core.CoordinateSystems;

/// <summary>
/// Отображение в квадрат [0,1]х[0,1]
/// произвольного четырехугольника с вершинами:
/// P00 -- P10
///  |      |
/// P01 -- P11
/// в локальной нумерации: 
/// 2 --- 3
/// |     |
/// 0 --- 1
/// </summary>
public sealed class QuadrangleCoordinateSystem(
    Vector2D p00,
    Vector2D p10,
    Vector2D p01,
    Vector2D p11
) : ICoordinateTransform<Vector2D, Vector2D>
{
    public static bool IsLinear => false;
    private readonly QuadrangleJacobyMatrix _j = new(p00, p10, p01, p11);

    public Vector2D Transform(Vector2D sourcePoint)
    {
        var local = new Vector2D(0.5, 0.5);
        const int maxIterations = 100;
        const double eps = 1e-13;

        for (int iter = 0; iter < maxIterations; iter++)
        {
            var residual = InverseTransform(local) - sourcePoint; // r = F(η, ξ) - P, residual - вектор невязки

            if (residual.Norm < eps) return local;

            var invJ = _j.At(local).Inverse();
            var delta = invJ * residual;  // Δ = J^(-1) * r, delta - вектор коррекции
            local -= delta; // (η, ξ) = (η, ξ) - Δ, обновляем локальные координаты
        }
        return local;
    }

    public Vector2D InverseTransform(Vector2D targetPoint)
    {
        (var ξ, var η) = targetPoint;

        var n00 = (1.0 - ξ) * (1.0 - η);
        var n10 = ξ * (1.0 - η);
        var n11 = ξ * η;
        var n01 = (1.0 - ξ) * η;

        return n00 * _j.P00 + n10 * _j.P10 + n11 * _j.P11 + n01 * _j.P01;
    }

    public double Jacobian(Vector2D targetPoint) => _j.Det(targetPoint);
    public IJacobyMatrix<Vector2D, Vector2D> InverseJacoby() => new QuadrangleInverseJacobyMatrix(_j);
}


// FIXME: This matrix used target point instead of source point
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

    public ConstantJacobyMatrix2D At(Vector2D targetPoint)
    {
        (var ξ, var η) = targetPoint;

        // N00 = (1-ξ)(1-η)
        // N10 = ξ(1-η)
        // N11 = ξ η
        // N01 = (1-ξ)η

        var dN00_dξ = -(1.0 - η);
        var dN10_dξ = +(1.0 - η);
        var dN11_dξ = +η;
        var dN01_dξ = -η;

        var dN00_dη = -(1.0 - ξ);
        var dN10_dη = -ξ;
        var dN11_dη = +ξ;
        var dN01_dη = +(1.0 - ξ);

        var dx_dξ = dN00_dξ * P00.X + dN10_dξ * P10.X + dN11_dξ * P11.X + dN01_dξ * P01.X;
        var dx_dη = dN00_dη * P00.X + dN10_dη * P10.X + dN11_dη * P11.X + dN01_dη * P01.X;
        var dy_dξ = dN00_dξ * P00.Y + dN10_dξ * P10.Y + dN11_dξ * P11.Y + dN01_dξ * P01.Y;
        var dy_dη = dN00_dη * P00.Y + dN10_dη * P10.Y + dN11_dη * P11.Y + dN01_dη * P01.Y;

        return new(new[,]
        {
            {dx_dξ,  dx_dη},
            {dy_dξ,  dy_dη }
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
