using Telma;

namespace Model.Model;

/// <summary>
/// Задача вида div(lambda * grad(u)) + gamma * u = f
/// </summary>
public sealed class EllipticProblem
{
    public required Func<Vector2D, double> Lambda { get; init; }
    public required Func<Vector2D, double> Gamma { get; init; }
    public required Func<Vector2D, double> Source { get; init; }
}

