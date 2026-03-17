using Telma;

namespace Model.Model.Basis;

public static class QuadrangleBasis
{
    /// <summary>N00(xi, eta) = (1 - xi) * (1 - eta)</summary>
    public static readonly IBasisFunction N00 = new QuadrangleN00();
    /// <summary>N10(xi, eta) = xi * (1 - eta)</summary>
    public static readonly IBasisFunction N10 = new QuadrangleN10();
    /// <summary>N11(xi, eta) = xi * eta</summary>
    public static readonly IBasisFunction N11 = new QuadrangleN11();
    /// <summary>N11(xi, eta) = (1 - xi) * eta</summary>
    public static readonly IBasisFunction N01 = new QuadrangleN01();

    /// <summary>
    /// Билинейная нода для всех 4 вершин одинакова, так как элемент прямоугольный и узлы расположены в одних и тех же местах по xi и eta.
    /// Поэтому можно использовать один массив для всех 4 вершин.
    /// </summary>
    public static readonly IBasisFunction[] Q1 = { N00, N10, N11, N01 };
    /// <summary>
    /// Биквадратная базисные функции для квадрата, построенные как тензорное произведение одномерных лагранжевых базисов.
    /// </summary>
    public static readonly IBasisFunction[] Q2_Lagrange = CreateTensorLagrange(2);
    /// <summary>
    /// Бикубическая базисные функции для квадрата, построенные как тензорное произведение одномерных лагранжевых базисов.
    /// </summary>
    public static readonly IBasisFunction[] Q3_Lagrange = CreateTensorLagrange(3);
    /// <summary>
    /// бикубическая базисные функции для квадрата, построенные как тензорное произведение одномерных эрмитовых базисов.
    /// </summary>
    public static readonly IBasisFunction[] Q3_Hermite = CreateTensorHermite();

    /// <summary>
    /// Базисная функция для линейного квадрата, определенная по (1 - xi, 1 - eta).
    /// </summary>
    private readonly struct QuadrangleN00 : IBasisFunction
    {
        public double Value(Vector2D localCoords)
        {
            var xi = localCoords.X;
            var eta = localCoords.Y;
            return (1 - xi) * (1 - eta);
        }
        public Vector2D Derivatives(Vector2D localCoords)
        {
            var xi = localCoords.X;
            var eta = localCoords.Y;
            return new Vector2D(-(1 - eta), -(1 - xi)); // [d/dxi, d/deta]
        }
    }
    /// <summary>
    /// Базисная функция для линейного квадрата, определенная по (xi, 1 - eta).
    /// </summary>
    private readonly struct QuadrangleN10 : IBasisFunction
    {
        public double Value(Vector2D localCoords)
        {
            var xi = localCoords.X;
            var eta = localCoords.Y;
            return (xi) * (1 - eta);
        }
        public Vector2D Derivatives(Vector2D localCoords)
        {
            var xi = localCoords.X;
            var eta = localCoords.Y;
            return new Vector2D((1 - eta), -xi); // [d/dxi, d/deta]
        }
    }
    /// <summary>
    /// Базисная функция для линейного квадрата, определенная по ( xi, eta ).
    /// </summary>
    private readonly struct QuadrangleN11 : IBasisFunction
    {
        public double Value(Vector2D localCoords)
        {
            var xi = localCoords.X;
            var eta = localCoords.Y;
            return (xi) * (eta);
        }
        public Vector2D Derivatives(Vector2D localCoords)
        {
            var xi = localCoords.X;
            var eta = localCoords.Y;
            return new Vector2D((eta), xi); // [d/dxi, d/deta]
        }
    }
    /// <summary>
    /// Базисная функция для линейного квадрата, определенная по (1 - xi, eta ).
    /// </summary>
    private readonly struct QuadrangleN01 : IBasisFunction
    {
        public double Value(Vector2D localCoords)
        {
            var xi = localCoords.X;
            var eta = localCoords.Y;
            return (1 - xi) * eta;
        }
        public Vector2D Derivatives(Vector2D localCoords)
        {
            var xi = localCoords.X;
            var eta = localCoords.Y;
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
    private static IBasisFunction[] CreateTensorLagrange(int degree)
    {
        // degree=2 -> nodes {0, 1/2, 1}
        // degree=3 -> nodes {0, 1/3, 2/3, 1}
        var nodes = degree switch
        {
            2 => new double[] { 0, 0.5, 1 },
            3 => new double[] { 0, 1.0 / 3, 2.0 / 3, 1 },
            _ => throw new ArgumentOutOfRangeException(nameof(degree), "Only degree 2 or 3 supported")
        };

        var oneD = new Lagrange1D(nodes);

        int n = nodes.Length;

        var basisFunctions = new IBasisFunction[n * n];

        int index = 0;



        for (int j = 0; j < n; j++)
        {
            for (int i = 0; i < n; i++)
            {
                basisFunctions[index++] = new TensorLagrange2D(oneD, i, j);
            }
        }
        return basisFunctions;
    }
    /// <summary>
    /// Лагранжевы базисные функции для одномерного интервала, определенные по заданным узлам.
    /// </summary>
    private readonly struct Lagrange1D
    {
        private readonly double[] _nodes;
        public Lagrange1D(double[] nodes)
        {
            _nodes = nodes;
        }
        public double Value(int i, double x)
        {
            double xi = _nodes[i];
            double res = 1.0;

            for (int j = 0; j < _nodes.Length; j++)
            {
                if (j == i) continue;
                double xj = _nodes[j];
                res *= (x - xj) / (xi - xj);
            }

            return res;
        }
        public double Derivative(int i, double x)
        {
            double xi = _nodes[i];
            double sum = 0.0;

            for (int j = 0; j < _nodes.Length; j++)
            {
                if (j == i) continue;

                double xj = _nodes[j];
                double product = 1.0 / (xi - xj);

                for (int k = 0; k < _nodes.Length; k++)
                {
                    if (k == i || k == j) continue;
                    double xk = _nodes[k];
                    product *= (x - xk) / (xi - xk);
                }

                sum += product;
            }
            return sum;
        }
    }
    /// <summary>
    /// Тензорное произведение одномерных лагранжевых базисов для квадрата.
    /// </summary>
    private readonly struct TensorLagrange2D : IBasisFunction
    {
        private readonly Lagrange1D _oneD;
        private readonly int _iXi;
        private readonly int _iEta;

        public TensorLagrange2D(Lagrange1D oneD, int iXi, int iEta)
        {
            _oneD = oneD;
            _iXi = iXi;
            _iEta = iEta;
        }
        public double Value(Vector2D localCoords)
        {
            return _oneD.Value(_iXi, localCoords.X) * _oneD.Value(_iEta, localCoords.Y);
        }

        public Vector2D Derivatives(Vector2D localCoords)
        {
            var (xi, eta) = localCoords;

            double Lx = _oneD.Value(_iXi, xi);
            double Ly = _oneD.Value(_iEta, eta);

            double dLx = _oneD.Derivative(_iXi, xi);
            double dLy = _oneD.Derivative(_iEta, eta);

            return new Vector2D(dLx * Ly, Lx * dLy);
        }
    }
    /// <summary>
    /// Эрмитовы базисные функции для квадрата, построенные как тензорное произведение одномерных эрмитовых базисов.
    /// </summary>
    /// <returns>Возвращает массив базисных функций для квадрата, построенных как тензорное произведение одномерных эрмитовых базисов.
    /// </returns>
    private static IBasisFunction[] CreateTensorHermite()
    {
        var hx = new Hermite1D();
        var hy = new Hermite1D();

        var basisFunctions = new IBasisFunction[16];
        int index = 0;

        for (int j = 0; j < 4; j++)
        {
            for (int i = 0; i < 4; i++)
            {
                basisFunctions[index++] = new TensorHermite2D(hx, hy, i, j);
            }
        }
        return basisFunctions;
    }
    /// <summary>
    /// Эрмитовы базисные функции для одномерного интервала, определенные по 4 функциям: H00, H10, H01, H11.
    /// </summary>
    private readonly struct Hermite1D
    {
        public double Value(int i, double x)
        {
            return i switch
            {
                0 => 1 - 3 * x * x + 2 * x * x * x, // H00
                1 => x - 2 * x * x + x * x * x,     // H10
                2 => 3 * x * x - 2 * x * x * x,     // H01
                3 => -x * x + x * x * x,            // H11
                _ => throw new ArgumentOutOfRangeException(nameof(i), "Only indices 0 to 3 supported")
            };
        }
        public double Derivative(int i, double x)
        {
            return i switch
            {
                0 => -6 * x + 6 * x * x,
                1 => 1 - 4 * x + 3 * x * x,
                2 => 6 * x - 6 * x * x,
                3 => -2 * x + 3 * x * x,
                _ => throw new ArgumentOutOfRangeException(nameof(i), "Only indices 0 to 3 supported")
            };
        }
    }
    /// <summary>
    /// Тензорное произведение одномерных эрмитовых базисов для квадрата.
    /// </summary>
    private readonly struct TensorHermite2D : IBasisFunction
    {
        private readonly Hermite1D _hx;
        private readonly Hermite1D _hy;
        private readonly int _iXi;
        private readonly int _iEta;
        public TensorHermite2D(Hermite1D hx, Hermite1D hy, int iXi, int iEta)
        {
            _hx = hx;
            _hy = hy;
            _iXi = iXi;
            _iEta = iEta;
        }
        public double Value(Vector2D localCoords)
        {
            return _hx.Value(_iXi, localCoords.X) * _hy.Value(_iEta, localCoords.Y);
        }
        public Vector2D Derivatives(Vector2D localCoords)
        {
            var (xi, eta) = localCoords;
            double Hx = _hx.Value(_iXi, xi);
            double Hy = _hy.Value(_iEta, eta);
            double dHx = _hx.Derivative(_iXi, xi);
            double dHy = _hy.Derivative(_iEta, eta);
            return new Vector2D(dHx * Hy, Hx * dHy);
        }
    }
}
