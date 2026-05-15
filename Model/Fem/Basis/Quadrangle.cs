using Model.Fem.Integrator;
using Hermite1D = Model.Fem.Basis.SegmentBasis.Hermite1D;
using Lagrange1D = Model.Fem.Basis.SegmentBasis.Lagrange1D;

namespace Model.Fem.Basis;


public static class QuadrangleBasis
{
    /// <summary>N00(ξ, η) = (1 - ξ) * (1 - η)</summary>
    public static readonly Polynomial2D N00 = TensorBasis2D.CreatePoly(SegmentBasis.N0, SegmentBasis.N0);
    /// <summary>N10(ξ, η) = ξ * (1 - η)</summary>
    public static readonly Polynomial2D N10 = TensorBasis2D.CreatePoly(SegmentBasis.N1, SegmentBasis.N0);
    /// <summary>N11(ξ, η) = ξ * η</summary>
    public static readonly Polynomial2D N11 = TensorBasis2D.CreatePoly(SegmentBasis.N1, SegmentBasis.N1);
    /// <summary>N01(ξ, η) = (1 - ξ) * η</summary>
    public static readonly Polynomial2D N01 = TensorBasis2D.CreatePoly(SegmentBasis.N0, SegmentBasis.N1);

    /// <summary>
    /// Билинейная нода для всех 4 вершин одинакова, так как элемент прямоугольный и узлы расположены в одних и тех же местах по xi и eta.
    /// Поэтому можно использовать один массив для всех 4 вершин.
    /// </summary>
    public static readonly Polynomial2D[] Q1 = [N00, N10, N11, N01];
    /// <summary>
    /// Биквадратная базисные функции для квадрата, построенные как тензорное произведение одномерных лагранжевых базисов.
    /// </summary>
    public static readonly Polynomial2D[] Q2_Lagrange = CreateTensorLagrange(2);
    /// <summary>
    /// Бикубическая базисные функции для квадрата, построенные как тензорное произведение одномерных лагранжевых базисов.
    /// </summary>
    public static readonly Polynomial2D[] Q3_Lagrange = CreateTensorLagrange(3);
    /// <summary>
    /// бикубическая базисные функции для квадрата, построенные как тензорное произведение одномерных эрмитовых базисов.
    /// </summary>
    public static readonly Polynomial2D[] Q3_Hermite = CreateTensorHermite();

    /// <summary>
    /// Тензорное произведение одномерных лагранжевых базисов для квадрата.
    /// Для degree=2 узлы расположены в {0, 0.5, 1},
    /// для degree=3 узлы расположены в {0, 1/3, 2/3, 1}.
    /// </summary>
    /// <param name="degree">Степень полинома в каждом направлении.</param>
    /// <returns>Возвращает массив базисных функций для квадрата, построенных как тензорное произведение одномерных лагранжевых базисов.</returns>
    /// <exception cref="ArgumentOutOfRangeException">Выброс исключения, если степень не равна 2 или 3.</exception>
    private static Polynomial2D[] CreateTensorLagrange(int degree)
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
        var basisFunctions = new Polynomial2D[n * n];
        int index = 0;

        for (int j = 0; j < n; j++)
            for (int i = 0; i < n; i++)
            {
                basisFunctions[index++] = TensorBasis2D.CreatePoly(
                    Lagrange1D.CreatePoly(nodes, i),
                    Lagrange1D.CreatePoly(nodes, j)
                );
            }

        return basisFunctions;
    }

    /// <summary>
    /// Эрмитовы базисные функции для квадрата, построенные как тензорное произведение одномерных эрмитовых базисов.
    /// </summary>
    /// <returns>Возвращает массив базисных функций для квадрата, построенных как тензорное произведение одномерных эрмитовых базисов.
    /// </returns>
    private static Polynomial2D[] CreateTensorHermite()
    {
        var basis = new Polynomial2D[16];
        int k = 0;
        Polynomial1D[] H =
        [
            Hermite1D.CreatePoly(0), Hermite1D.CreatePoly(1), Hermite1D.CreatePoly(2), Hermite1D.CreatePoly(3)
        ];
        // узел (0,0)
        basis[k++] = TensorBasis2D.CreatePoly(H[0], H[0]); // u
        basis[k++] = TensorBasis2D.CreatePoly(H[1], H[0]); // uξ
        basis[k++] = TensorBasis2D.CreatePoly(H[0], H[1]); // uη
        basis[k++] = TensorBasis2D.CreatePoly(H[1], H[1]); // uξη

        // узел (1,0)
        basis[k++] = TensorBasis2D.CreatePoly(H[2], H[0]);
        basis[k++] = TensorBasis2D.CreatePoly(H[3], H[0]);
        basis[k++] = TensorBasis2D.CreatePoly(H[2], H[1]);
        basis[k++] = TensorBasis2D.CreatePoly(H[3], H[1]);

        // узел (1,1)
        basis[k++] = TensorBasis2D.CreatePoly(H[2], H[2]);
        basis[k++] = TensorBasis2D.CreatePoly(H[3], H[2]);
        basis[k++] = TensorBasis2D.CreatePoly(H[2], H[3]);
        basis[k++] = TensorBasis2D.CreatePoly(H[3], H[3]);

        // узел (0,1)
        basis[k++] = TensorBasis2D.CreatePoly(H[0], H[2]);
        basis[k++] = TensorBasis2D.CreatePoly(H[1], H[2]);
        basis[k++] = TensorBasis2D.CreatePoly(H[0], H[3]);
        basis[k++] = TensorBasis2D.CreatePoly(H[1], H[3]);

        return basis;
    }
}
