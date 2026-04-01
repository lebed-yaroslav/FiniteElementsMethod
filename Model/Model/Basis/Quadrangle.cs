using Telma;
using Hermite1D = Model.Model.Basis.SegmentBasis.Hermite1D;
using Lagrange1D = Model.Model.Basis.SegmentBasis.Lagrange1D;

namespace Model.Model.Basis;

public static class QuadrangleBasis
{
    /// <summary>N00(ξ, η) = (1 - ξ) * (1 - η)</summary>
    public static readonly IBasisFunction2D N00 = new TensorBasis2D(SegmentBasis.N0, SegmentBasis.N0);
    /// <summary>N10(ξ, η) = ξ * (1 - η)</summary>
    public static readonly IBasisFunction2D N10 = new TensorBasis2D(SegmentBasis.N1, SegmentBasis.N0);
    /// <summary>N11(ξ, η) = ξ * η</summary>
    public static readonly IBasisFunction2D N11 = new TensorBasis2D(SegmentBasis.N1, SegmentBasis.N1);
    /// <summary>N01(ξ, η) = (1 - ξ) * η</summary>
    public static readonly IBasisFunction2D N01 = new TensorBasis2D(SegmentBasis.N0, SegmentBasis.N1);

    /// <summary>
    /// Билинейная нода для всех 4 вершин одинакова, так как элемент прямоугольный и узлы расположены в одних и тех же местах по xi и eta.
    /// Поэтому можно использовать один массив для всех 4 вершин.
    /// </summary>
    public static readonly IBasisFunction2D[] Q1 = [N00, N10, N11, N01];
    /// <summary>
    /// Биквадратная базисные функции для квадрата, построенные как тензорное произведение одномерных лагранжевых базисов.
    /// </summary>
    public static readonly IBasisFunction2D[] Q2_Lagrange = CreateTensorLagrange(2);
    /// <summary>
    /// Бикубическая базисные функции для квадрата, построенные как тензорное произведение одномерных лагранжевых базисов.
    /// </summary>
    public static readonly IBasisFunction2D[] Q3_Lagrange = CreateTensorLagrange(3);
    /// <summary>
    /// бикубическая базисные функции для квадрата, построенные как тензорное произведение одномерных эрмитовых базисов.
    /// </summary>
    public static readonly IBasisFunction2D[] Q3_Hermite = CreateTensorHermite();

    /// <summary>
    /// Тензорное произведение одномерных лагранжевых базисов для квадрата.
    /// Для degree=2 узлы расположены в {0, 0.5, 1},
    /// для degree=3 узлы расположены в {0, 1/3, 2/3, 1}.
    /// </summary>
    /// <param name="degree">Степень полинома в каждом направлении.</param>
    /// <returns>Возвращает массив базисных функций для квадрата, построенных как тензорное произведение одномерных лагранжевых базисов.</returns>
    /// <exception cref="ArgumentOutOfRangeException">Выброс исключения, если степень не равна 2 или 3.</exception>
    private static IBasisFunction2D[] CreateTensorLagrange(int degree)
    {
        // degree=2 -> nodes {0, 1/2, 1}
        // degree=3 -> nodes {0, 1/3, 2/3, 1}
        double[] nodes = degree switch
        {
            2 => [0, 0.5, 1],
            3 => [0, 1.0 / 3, 2.0 / 3, 1],
            _ => throw new ArgumentOutOfRangeException(nameof(degree), "Only degree 2 or 3 supported")
        };

        int n = nodes.Length;
        var basisFunctions = new IBasisFunction2D[n * n];
        int index = 0;

        for (int j = 0; j < n; j++)
            for (int i = 0; i < n; i++)
            {
                basisFunctions[index++] = new TensorBasis2D(
                    new Lagrange1D(nodes, i),
                    new Lagrange1D(nodes, j)
                );
            }

        return basisFunctions;
    }

    /// <summary>
    /// Эрмитовы базисные функции для квадрата, построенные как тензорное произведение одномерных эрмитовых базисов.
    /// </summary>
    /// <returns>Возвращает массив базисных функций для квадрата, построенных как тензорное произведение одномерных эрмитовых базисов.
    /// </returns>
    private static IBasisFunction2D[] CreateTensorHermite()
    {
        var basisFunctions = new IBasisFunction2D[16];
        int index = 0;

        for (int j = 0; j < 4; j++)
        {
            for (int i = 0; i < 4; i++)
            {
                basisFunctions[index++] = new TensorBasis2D(
                    new Hermite1D(i),
                    new Hermite1D(j)
                );
            }
        }
        return basisFunctions;
    }
}
