using Telma;

namespace Model.Model.Basis;


public static class SegmentBasis
{
    /// <summary>N0(ξ) = 1 - ξ</summary>
    public static readonly IBasisFunction1D N0 = new SegmentN0();
    /// <summary>N1(ξ) = ξ</summary>
    public static readonly IBasisFunction1D N1 = new SegmentN1();
    /// <summary>N0N1(ξ) = ξ (1 - ξ)</summary>
    public static readonly IBasisFunction1D N0N1 = new SegmentN0N1();

    private readonly struct SegmentN0 : IBasisFunction1D
    {
        public double Value(Vector1D point) => 1 - point;
        public Vector1D Derivatives(Vector1D _) => -1;
    }

    private readonly struct SegmentN1 : IBasisFunction1D
    {
        public double Value(Vector1D point) => point;
        public Vector1D Derivatives(Vector1D _) => 1;
    }

    private readonly struct SegmentN0N1 : IBasisFunction1D
    {
        public double Value(Vector1D point) => point * (1 - point);
        public Vector1D Derivatives(Vector1D point) => 1 - 2 * point;
    }

    public readonly struct Lagrange1D(double[] nodes, int index) : IBasisFunction<Vector1D>
    {
        public double Value(Vector1D point)
        {
            double x = point;
            double xi = nodes[index];
            double res = 1.0;

            for (int j = 0; j < nodes.Length; j++)
            {
                if (j == index) continue;
                double xj = nodes[j];
                res *= (x - xj) / (xi - xj);
            }

            return res;
        }

        public Vector1D Derivatives(Vector1D point)
        {
            double x = point;
            double xi = nodes[index];
            double sum = 0.0;

            for (int j = 0; j < nodes.Length; j++)
            {
                if (j == index) continue;

                double xj = nodes[j];
                double product = 1.0 / (xi - xj);

                for (int k = 0; k < nodes.Length; k++)
                {
                    if (k == index || k == j) continue;
                    double xk = nodes[k];
                    product *= (x - xk) / (xi - xk);
                }

                sum += product;
            }
            return sum;
        }
    }

    /// <summary>
    /// Эрмитовы базисные функции для одномерного интервала, определенные по 4 функциям: H00, H10, H01, H11.
    /// </summary>
    public readonly struct Hermite1D(int index) : IBasisFunction<Vector1D>
    {
        public double Value(Vector1D point)
        {
            double x = point;
            return index switch
            {
                0 => 1 - 3 * x * x + 2 * x * x * x, // H00
                1 => x - 2 * x * x + x * x * x,     // H10
                2 => 3 * x * x - 2 * x * x * x,     // H01
                3 => -x * x + x * x * x,            // H11
                _ => throw new ArgumentOutOfRangeException(nameof(index), "Only indices 0 to 3 supported")
            };
        }

        public Vector1D Derivatives(Vector1D point)
        {
            double x = point;
            return index switch
            {
                0 => -6 * x + 6 * x * x,
                1 => 1 - 4 * x + 3 * x * x,
                2 => 6 * x - 6 * x * x,
                3 => -2 * x + 3 * x * x,
                _ => throw new ArgumentOutOfRangeException(nameof(index), "Only indices 0 to 3 supported")
            };
        }
    }
}
