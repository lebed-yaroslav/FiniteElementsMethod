using System.Diagnostics;
using Model.Fem.Integrator;
using Telma;

namespace Model.Fem.Basis;


public static class SegmentBasis
{
    /// <summary>N0(ξ) = 1 - ξ</summary>
    public static readonly IBasisFunction1D N0 = new SegmentN0();
    /// <summary>N1(ξ) = ξ</summary>
    public static readonly IBasisFunction1D N1 = new SegmentN1();
    /// <summary>N0N1(ξ) = ξ (1 - ξ)</summary>
    public static readonly IBasisFunction1D N0N1 = new SegmentN0N1();
    /// <summary>N0N1N0SubN1(ξ) = ξ (1 - ξ)(2ξ - 1)</summary>
    public static readonly IBasisFunction1D N0N1N0SubN1 = new SegmentN0N1N0SubN1();

    private readonly struct SegmentN0 : IAnalyticalBasisFunction1D
    {
        public double Value(Vector1D point) => 1 - point;
        public Vector1D Derivatives(Vector1D _) => -1;
        public IPolynomial Polynomial => SegmentPolynomialBasis.N0;
    }

    private readonly struct SegmentN1 : IAnalyticalBasisFunction1D
    {
        public double Value(Vector1D point) => point;
        public Vector1D Derivatives(Vector1D _) => 1;
        public IPolynomial Polynomial => SegmentPolynomialBasis.N1;
    }

    private readonly struct SegmentN0N1 : IAnalyticalBasisFunction1D
    {
        public double Value(Vector1D point) => point * (1 - point);
        public Vector1D Derivatives(Vector1D point) => 1 - 2 * point;
        public IPolynomial Polynomial => SegmentPolynomialBasis.N0N1;
    }

    private readonly struct SegmentN0N1N0SubN1 : IAnalyticalBasisFunction1D
    {
        public double Value(Vector1D point) => point * (1 - point) * (2 * point - 1);
        public Vector1D Derivatives(Vector1D point) => -6 * point * (point - 1) - 1;
        public IPolynomial Polynomial => SegmentPolynomialBasis.N0N1N0SubN1;
    }

    public readonly struct Lagrange1D(double[] nodes, int index) : IAnalyticalBasisFunction1D
    {
        public static Lagrange1D Create(int order, int index)
        {
            Debug.Assert(0 < order);
            Debug.Assert(0 <= index && index <= order);

            var nodes = new double[order + 1];
            double h = 1.0 / order;

            for (int i = 0; i < order; i++)
                nodes[i] = i * h;
            nodes[order] = 1;

            return new(nodes, index);
        }

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
        public IPolynomial Polynomial
        {
            get
            {
                int order = nodes.Length - 1;
                return order switch
                {
                    2 => index switch
                    {
                        0 => SegmentPolynomialBasis.L2_0,
                        1 => SegmentPolynomialBasis.L2_1,
                        2 => SegmentPolynomialBasis.L2_2,
                        _ => throw new IndexOutOfRangeException()
                    },
                    3 => index switch
                    {
                        0 => SegmentPolynomialBasis.L3_0,
                        1 => SegmentPolynomialBasis.L3_1,
                        2 => SegmentPolynomialBasis.L3_2,
                        3 => SegmentPolynomialBasis.L3_3,
                        _ => throw new IndexOutOfRangeException()
                    },
                    _ => throw new NotSupportedException($"Supports only 2 and 3 order.")
                };
            }
        }
    }

    /// <summary>
    /// Эрмитовы базисные функции для одномерного интервала, определенные по 4 функциям: H00, H10, H01, H11.
    /// </summary>
    public readonly struct Hermite1D(int index) : IAnalyticalBasisFunction1D
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
        public IPolynomial Polynomial
        {
            get
            {
                return index switch
                {
                    0 => SegmentPolynomialBasis.H00,
                    1 => SegmentPolynomialBasis.H10,
                    2 => SegmentPolynomialBasis.H01,
                    3 => SegmentPolynomialBasis.H11,
                    _ => throw new ArgumentOutOfRangeException(nameof(index), "Only indices 0 to 3 supported")
                };
        }
        }

    }
    public static class SegmentPolynomialBasis
    {
        /// <summary>
        /// N0(ξ) = 1 - ξ
        /// </summary>
        public static readonly IPolynomial N0 = new Polynomial
        {
            Summands = [(0, 0, 1.0), (1, 0, -1.0)]
        };

        /// <summary>
        /// N1(ξ) = ξ
        /// </summary>
        public static readonly IPolynomial N1 = new Polynomial
        {
            Summands = [(1, 0, 1.0)]
        };

        /// <summary>
        /// N0N1(ξ) = ξ(1 - ξ) = ξ - ξ^2
        /// </summary>
        public static readonly IPolynomial N0N1 = new Polynomial
        {
            Summands = [(1, 0, 1.0), (2, 0, -1.0)]
        };

        /// <summary>
        /// N0N1N0SubN1(ξ) = (ξ - ξ^2)(2ξ - 1) = -2ξ^3 + 3ξ^2 - ξ
        /// </summary>
        public static readonly IPolynomial N0N1N0SubN1 = new Polynomial
        {
            Summands = [(3, 0, -2.0), (2, 0, 3.0), (1, 0, -1.0)]
        };
        //Hermite
        /// <summary>
        /// H00 = 1 - 3ξ^2 + 2ξ^3
        /// </summary>
        public static readonly IPolynomial H00 = new Polynomial
        { 
            Summands = [(3, 0, 2.0), (2, 0, -3.0), (0, 0, 1.0)] 
        };
        /// <summary>
        /// H10 = ξ - 2ξ^2 + ξ^3
        /// </summary>
        public static readonly IPolynomial H10 = new Polynomial 
        { 
            Summands = [(3, 0, 1.0), (2, 0, -2.0), (1, 0, 1.0)] 
        };
        /// <summary>
        /// H01 = 3ξ^2 - 2ξ^3
        /// </summary>
        public static readonly IPolynomial H01 = new Polynomial
        { 
            Summands = [(3, 0, -2.0), (2, 0, 3.0)]
        };
        /// <summary>
        /// H11 = -ξ^2 + ξ^3
        /// </summary>
        public static readonly IPolynomial H11 = new Polynomial
        { 
            Summands = [(3, 0, 1.0), (2, 0, -1.0)]
        };


        // Лагранжевы базисы для degree=2 (узлы: 0, 0.5, 1)

        /// <summary>
        /// index 0: 2ξ^2 - 3ξ + 1
        /// </summary>
        public static readonly IPolynomial L2_0 = new Polynomial 
        { 
            Summands = [(2, 0, 2.0), (1, 0, -3.0), (0, 0, 1.0)] 
        };
        /// <summary>
        /// index 1: -4ξ^2 + 4ξ
        /// </summary>
        public static readonly IPolynomial L2_1 = new Polynomial 
        {
            Summands = [(2, 0, -4.0), (1, 0, 4.0)] 
        };
        /// <summary>
        /// index 2: 2ξ^2 - ξ
        /// </summary>
        public static readonly IPolynomial L2_2 = new Polynomial 
        { 
            Summands = [(2, 0, 2.0), (1, 0, -1.0)] 
        };


        // Лагранжевы базисы для degree=3 (узлы: 0, 1/3, 2/3, 1) 

        /// <summary>
        /// index 0: -13.5ξ^3 + 27ξ^2 - 16.5ξ + 1
        /// </summary>
        public static readonly IPolynomial L3_0 = new Polynomial 
        { 
            Summands = [(3, 0, -13.5), (2, 0, 27.0), (1, 0, -16.5), (0, 0, 1.0)] 
        };
        /// <summary>
        /// index 1: 13.5ξ^3 - 22.5ξ^2 + 9ξ
        /// </summary>
        public static readonly IPolynomial L3_1 = new Polynomial 
        { 
            Summands = [(3, 0, 13.5), (2, 0, -22.5), (1, 0, 9.0)] 
        };
        /// <summary>
        /// index 2: -13.5ξ^3 + 18ξ^2 - 4.5ξ
        /// </summary>
        public static readonly IPolynomial L3_2 = new Polynomial 
        { 
            Summands = [(3, 0, -13.5), (2, 0, 18.0), (1, 0, -4.5)]
        };
        /// <summary>
        /// index 3: 4.5ξ^3 - 4.5ξ^2 + ξ
        /// </summary>
        public static readonly IPolynomial L3_3 = new Polynomial 
        { 
            Summands = [(3, 0, 4.5), (2, 0, -4.5), (1, 0, 1.0)]
        };
    }
}
