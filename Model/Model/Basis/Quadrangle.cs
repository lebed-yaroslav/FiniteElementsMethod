using Telma;
using Lagrange1D = Model.Model.Basis.SegmentBasis.Lagrange1D;
using Hermite1D = Model.Model.Basis.SegmentBasis.Hermite1D;

namespace Model.Model.Basis;

public static class QuadrangleBasis
{
    /// <summary>N00(xi, eta) = (1 - xi) * (1 - eta)</summary>
    public static readonly IBasisFunction2D N00 = new QuadrangleN00();
    /// <summary>N10(xi, eta) = xi * (1 - eta)</summary>
    public static readonly IBasisFunction2D N10 = new QuadrangleN10();
    /// <summary>N11(xi, eta) = xi * eta</summary>
    public static readonly IBasisFunction2D N11 = new QuadrangleN11();
    /// <summary>N11(xi, eta) = (1 - xi) * eta</summary>
    public static readonly IBasisFunction2D N01 = new QuadrangleN01();

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
    /// Базисная функция для линейного квадрата, определенная по (1 - xi, 1 - eta).
    /// </summary>
    private readonly struct QuadrangleN00 : IBasisFunction2D
    {
        public double Value(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return (1 - xi) * (1 - eta);
        }

        public Vector2D Derivatives(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return new Vector2D(-(1 - eta), -(1 - xi)); // [d/dxi, d/deta]
        }
    }

    /// <summary>
    /// Базисная функция для линейного квадрата, определенная по (xi, 1 - eta).
    /// </summary>
    private readonly struct QuadrangleN10 : IBasisFunction2D
    {
        public double Value(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return (xi) * (1 - eta);
        }

        public Vector2D Derivatives(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return new Vector2D((1 - eta), -xi); // [d/dxi, d/deta]
        }
    }

    /// <summary>
    /// Базисная функция для линейного квадрата, определенная по ( xi, eta ).
    /// </summary>
    private readonly struct QuadrangleN11 : IBasisFunction2D
    {
        public double Value(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return (xi) * (eta);
        }

        public Vector2D Derivatives(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return new Vector2D((eta), xi); // [d/dxi, d/deta]
        }
    }

    /// <summary>
    /// Базисная функция для линейного квадрата, определенная по (1 - xi, eta ).
    /// </summary>
    private readonly struct QuadrangleN01 : IBasisFunction2D
    {
        public double Value(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return (1 - xi) * eta;
        }

        public Vector2D Derivatives(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return new Vector2D(-eta, 1 - xi); // [d/dxi, d/deta]
        }
    }

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

<<<<<<< HEAD
=======

>>>>>>> Исправления: одномерный лагранжевые и эрмитовые базисы вынесены в SegmentBasis
        for (int j = 0; j < n; j++)
            for (int i = 0; i < n; i++)
            {
                basisFunctions[index++] = new TensorBasis2D(
                    new Lagrange1D(nodes, i),
                    new Lagrange1D(nodes, j)
                );
            }
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

    /// <summary>
    /// Универсальное тензорное произведение двух одномерных базисов для построения 2D базиса.
    /// </summary>
    private readonly struct TensorBasis2D(IBasisFunction<Vector1D> bX, IBasisFunction<Vector1D> bY) : IBasisFunction2D
    {
        public double Value(Vector2D p) => bX.Value(p.X) * bY.Value(p.Y);

        public Vector2D Derivatives(Vector2D p)
        {
            double valX = bX.Value(p.X);
            double valY = bY.Value(p.Y);

            double derX = bX.Derivatives(p.X);
            double derY = bY.Derivatives(p.Y);

            return new Vector2D(derX * valY, valX * derY);
        }
    }
}
