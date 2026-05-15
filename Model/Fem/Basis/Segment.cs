using System.Diagnostics;
using Model.Fem.Integrator;
using Telma;

namespace Model.Fem.Basis;


public static class SegmentBasis
{
    /// <summary>N0(ξ) = 1 - ξ</summary>
    public static readonly Polynomial1D N0 = new()
    {
        Summands = {
            [0] = 1.0,
            [1] = -1.0
        }
    };
    /// <summary>N1(ξ) = ξ</summary>
    public static readonly Polynomial1D N1 = new()
    {
        Summands = {
            [1] = 1.0
        }
    };
    /// <summary>N0N1(ξ) = ξ (1 - ξ)</summary>
    public static readonly Polynomial1D N0N1 = new()
    {
        Summands = {
            [1] = 1.0,
            [2] = -1.0
        }
    };
    /// <summary>N0N1N0SubN1(ξ) = ξ (1 - ξ)(2ξ - 1)</summary>
    public static readonly Polynomial1D N0N1N0SubN1 = new()
    {
        Summands = {
            [3] = -2.0,
            [2] = 3.0,
            [1] = -1.0
        }
    };

    public static class Lagrange1D
    {
        public static Polynomial1D Create(int order, int index)
        {
            Debug.Assert(0 < order);
            Debug.Assert(0 <= index && index <= order);

            var nodes = new double[order + 1];
            double h = 1.0 / order;

            for (int i = 0; i < order; i++)
                nodes[i] = i * h;
            nodes[order] = 1;
            return CreatePoly(nodes, index);
        }

        public static Polynomial1D CreatePoly(double[] nodes, int index)
        {
            double xi = nodes[index];
            var res = new Polynomial1D();
            res.Summands.Add(0, 1.0);

            for (int j = 0; j < nodes.Length; j++)
            {
                if (j == index) continue;
                double xj = nodes[j];

                var fraction = new Polynomial1D();
                fraction.Summands.Add(1, 1.0 / (xi - xj));
                fraction.Summands.Add(0, -xj / (xi - xj));
                res.Mult(fraction);
            }

            res.Delete_Nulls();
            return res;
        }
    }

    /// <summary>
    /// Эрмитовы базисные функции для одномерного интервала, определенные по 4 функциям: H00, H10, H01, H11.
    /// </summary>
    public static class Hermite1D
    {
        public static Polynomial1D CreatePoly(int index)
        {
            var poly = new Polynomial1D();

            switch (index)
            {
                case 0: // H00 = 1 - 3x² + 2x³
                    poly.Summands.Add(0, 1.0);
                    poly.Summands.Add(2, -3.0);
                    poly.Summands.Add(3, 2.0);
                    break;

                case 1: // H10 = x - 2x² + x³
                    poly.Summands.Add(1, 1.0);
                    poly.Summands.Add(2, -2.0);
                    poly.Summands.Add(3, 1.0);
                    break;

                case 2: // H01 = 3x² - 2x³
                    poly.Summands.Add(2, 3.0);
                    poly.Summands.Add(3, -2.0);
                    break;

                case 3: // H11 = -x² + x³
                    poly.Summands.Add(2, -1.0);
                    poly.Summands.Add(3, 1.0);
                    break;

                default:
                    throw new ArgumentOutOfRangeException(nameof(index), "Only indices 0 to 3 supported");
            }

            return poly;
        }
    }
}
