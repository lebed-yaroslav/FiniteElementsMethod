using Model.Fem.Integrator;
using Telma;

namespace Model.Fem.Basis;

public static partial class TriangleBasis
{
    public static class Lagrange
    {
        /// <summary>L1L1L1 = 0.5 * (L1) * (3 * L1 - 1) * (3 * L1 - 2)</summary>
        public static readonly IBasisFunction2D L1L1L1 = new TriangleLagrangeL1L1L1();
        /// <summary>L2L2L2 = 0.5 * (L2) * (3 * L2 - 1) * (3 * L2 - 2)</summary>
        public static readonly IBasisFunction2D L2L2L2 = new TriangleLagrangeL2L2L2();
        /// <summary>L3L3L3 = 0.5 * (L3) * (3 * L3 - 1) * (3 * L3 - 2)</summary>
        public static readonly IBasisFunction2D L3L3L3 = new TriangleLagrangeL3L3L3();
        /// <summary>L1L2L1 = 4.5 * (L1) * (L2) * (3 * L1 - 1)</summary>
        public static readonly IBasisFunction2D L1L2L1 = new TriangleLagrangeL1L2L1();
        /// <summary>L1L2L2 = 4.5 * (L1) * (L2) * (3 * L2 - 1)</summary>
        public static readonly IBasisFunction2D L1L2L2 = new TriangleLagrangeL1L2L2();
        /// <summary>L2L3L2 = 4.5 * (L2) * (L3) * (3 * L2 - 1)</summary>
        public static readonly IBasisFunction2D L2L3L2 = new TriangleLagrangeL2L3L2();
        /// <summary>L2L3L3 = 4.5 * (L2) * (L3) * (3 * L3 - 1)</summary>
        public static readonly IBasisFunction2D L2L3L3 = new TriangleLagrangeL2L3L3();
        /// <summary>L3L1L3 = 4.5 * (L3) * (L1) * (3 * L3 - 1)</summary>
        public static readonly IBasisFunction2D L3L1L3 = new TriangleLagrangeL3L1L3();
        /// <summary>L3L1L1 = 4.5 * (L3) * (L1) * (3 * L1 - 1)</summary>
        public static readonly IBasisFunction2D L3L1L1 = new TriangleLagrangeL3L1L1();
        /// <summary>L1L2L3 = 27.0 * (L1) * (L2) * (L3)</summary>
        public static readonly IBasisFunction2D L1L2L3 = new TriangleLagrangeL1L2L3();
    }

    private readonly struct TriangleLagrangeL1L1L1 : IAnalyticalBasisFunction2D
    {
        
        public double Value(Vector2D localCoords)
        {
            var xi = localCoords.X;
            return 0.5 * xi * (3 * xi - 1) * (3 * xi - 2);
        }

        public Vector2D Derivatives(Vector2D localCoords)
        {
            var xi = localCoords.X;
            return new(0.5 * (27 * xi * xi - 18 * xi + 2), 0.0); // [d/dξ, d/dη]
        }
        public IPolynomial Polynomial => TriangleLargangePolynomialBasis.L1L1L1;
    }

    private readonly struct TriangleLagrangeL2L2L2 : IAnalyticalBasisFunction2D
    {
        public double Value(Vector2D localCoords)
        {
            var eta = localCoords.Y;
            return 0.5 * eta * (3 * eta - 1) * (3 * eta - 2);
        }

        public Vector2D Derivatives(Vector2D localCoords)
        {
            var eta = localCoords.Y;
            return new(0.0, 0.5 * (27 * eta * eta - 18 * eta + 2)); // [d/dξ, d/dη]
        }
        public IPolynomial Polynomial => TriangleLargangePolynomialBasis.L2L2L2;
    }

    private readonly struct TriangleLagrangeL3L3L3 : IAnalyticalBasisFunction2D
    {
        public double Value(Vector2D localCoords)
        {
            var zeta = 1 - localCoords.X - localCoords.Y;
            return 0.5 * zeta * (3 * zeta - 1) * (3 * zeta - 2);
        }

        public Vector2D Derivatives(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return new(
                0.5 * (-27 * xi * xi - 54 * xi * eta + 36 * xi - 27 * eta * eta + 36 * eta - 11), // d/dξ
                0.5 * (-27 * xi * xi - 54 * xi * eta + 36 * xi - 27 * eta * eta + 36 * eta - 11) // d/dη
            );
        }
        public IPolynomial Polynomial => TriangleLargangePolynomialBasis.L3L3L3;
    }

    private readonly struct TriangleLagrangeL1L2L1 : IAnalyticalBasisFunction2D
    {
        public double Value(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return 4.5 * xi * eta * (3 * xi - 1);
        }

        public Vector2D Derivatives(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return new(
                4.5 * (6 * xi * eta - eta), // d/dξ
                4.5 * (3 * xi * xi - xi) // d/dη
            );
        }
        public IPolynomial Polynomial => TriangleLargangePolynomialBasis.L1L2L1;
    }

    private readonly struct TriangleLagrangeL1L2L2 : IAnalyticalBasisFunction2D
    {
        public double Value(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return 4.5 * xi * eta * (3 * eta - 1);
        }

        public Vector2D Derivatives(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return new(
                4.5 * (3 * eta * eta - eta), // d/dξ
                4.5 * (6 * xi * eta - xi) // d/dη
            );
        }
        public IPolynomial Polynomial => TriangleLargangePolynomialBasis.L1L2L2;
    }

    private readonly struct TriangleLagrangeL2L3L2 : IAnalyticalBasisFunction2D
    {
        public double Value(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return 4.5 * eta * (1 - xi - eta) * (3 * eta - 1);
        }

        public Vector2D Derivatives(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return new(
                4.5 * (-3 * eta * eta + eta), // d/dξ
                4.5 * (-6 * xi * eta + xi - 9 * eta * eta + 8 * eta - 1) // d/dη
            );
        }
        public IPolynomial Polynomial => TriangleLargangePolynomialBasis.L2L3L2;
    }

    private readonly struct TriangleLagrangeL2L3L3 : IAnalyticalBasisFunction2D
    {
        public double Value(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            var zeta = 1 - xi - eta;
            return 4.5 * eta * zeta * (3 * zeta - 1);
        }

        public Vector2D Derivatives(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return new(
                4.5 * (6 * eta * eta + 6 * xi * eta - 5 * eta), // d/dξ
                4.5 * (3 * xi * xi + 12 * xi * eta - 5 * xi + 9 * eta * eta - 10 * eta + 2) // d/dη
            );
        }
        public IPolynomial Polynomial => TriangleLargangePolynomialBasis.L2L3L3;
    }

    private readonly struct TriangleLagrangeL3L1L3 : IAnalyticalBasisFunction2D
    {
        public double Value(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            var zeta = 1 - xi - eta;
            return 4.5 * zeta * xi * (3 * zeta - 1);
        }

        public Vector2D Derivatives(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return new(
                4.5 * (9 * xi * xi + 12 * xi * eta - 10 * xi + 3 * eta * eta - 5 * eta + 2), // d/dξ
                4.5 * (6 * xi * xi + 6 * xi * eta - 5 * xi) // d/dη
            );
        }
        public IPolynomial Polynomial => TriangleLargangePolynomialBasis.L3L1L3;
    }

    private readonly struct TriangleLagrangeL3L1L1 : IAnalyticalBasisFunction2D
    {
        public double Value(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return 4.5 * (1 - xi - eta) * xi * (3 * xi - 1);
        }

        public Vector2D Derivatives(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return new(
                4.5 * (-9 * xi * xi - 6 * xi * eta + 8 * xi + 1 * eta - 1), // d/dξ
                4.5 * (-3 * xi * xi + xi) // d/dη
            );
        }
        public IPolynomial Polynomial => TriangleLargangePolynomialBasis.L3L1L1;
    }

    private readonly struct TriangleLagrangeL1L2L3 : IAnalyticalBasisFunction2D
    {
        public double Value(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return 27.0 * xi * eta * (1 - xi - eta);
        }

        public Vector2D Derivatives(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return new(
                27.0 * (-2 * xi * eta - eta * eta + eta), // d/dξ
                27.0 * (-2 * xi * eta - xi * xi + xi) // d/dη
            );
        }
        public IPolynomial Polynomial => TriangleLargangePolynomialBasis.L1L2L3;
    }
    public static class TriangleLargangePolynomialBasis
    {
        /// <summary>L1L1L1 = 0.5 * (L1) * (3 * L1 - 1) * (3 * L1 - 2)</summary>
        public static readonly IPolynomial L1L1L1 = new Polynomial()
        {
            Summands = [(3, 0, 4.5), (2, 0, -4.5), (1, 0, 1.0)]
        };

        /// <summary>L2L2L2 = 0.5 * (L2) * (3 * L2 - 1) * (3 * L2 - 2)</summary>
        public static readonly IPolynomial L2L2L2 = new Polynomial()
        {
            Summands = [(0, 3, 4.5), (0, 2, -4.5), (0, 1, 1.0)]
        };

        /// <summary>L3L3L3 = 0.5 * (L3) * (3 * L3 - 1) * (3 * L3 - 2)</summary>
        public static readonly IPolynomial L3L3L3 = new Polynomial
        {
            Summands = [
                (0, 0, 1.0),
            (1, 0, -5.5),  (0, 1, -5.5),
            (2, 0, 9.0),   (1, 1, 18.0),  (0, 2, 9.0),
            (3, 0, -4.5),  (2, 1, -13.5), (1, 2, -13.5), (0, 3, -4.5)
            ]
        };

        /// <summary>
        /// L1L2L1 = 4.5 * ξ * η * (3ξ - 1) = 13.5ξ^2η - 4.5ξη
        /// </summary>
        public static readonly IPolynomial L1L2L1 = new Polynomial
        {
            Summands = [(2, 1, 13.5), (1, 1, -4.5)]
        };

        /// <summary>
        /// L1L2L2 = 4.5 * ξ * η * (3η - 1) = 13.5ξη^2 - 4.5ξη
        /// </summary>
        public static readonly IPolynomial L1L2L2 = new Polynomial
        {
            Summands = [(1, 2, 13.5), (1, 1, -4.5)]
        };

        /// <summary>
        /// L2L3L2 = 4.5 * η * (1-ξ-η) * (3η - 1) = 18η^2 - 4.5η - 13.5ξη^2 + 4.5ξη - 13.5η^3
        /// </summary>
        public static readonly IPolynomial L2L3L2 = new Polynomial
        {
            Summands = [(0, 2, 18.0), (0, 1, -4.5), (1, 2, -13.5), (1, 1, 4.5), (0, 3, -13.5)]
        };

        /// <summary>
        /// L2L3L3 = 4.5 * η * (1-ξ-η) * (2-3ξ-3η)
        /// </summary>
        public static readonly IPolynomial L2L3L3 = new Polynomial
        {
            Summands = [(0, 1, 9.0), (1, 1, -22.5), (0, 2, -22.5), (2, 1, 13.5), (1, 2, 27.0), (0, 3, 13.5)]
        };

        /// <summary>
        /// L3L1L3 = 4.5 * (1-ξ-η) * ξ * (2-3ξ-3η)
        /// </summary>
        public static readonly IPolynomial L3L1L3 = new Polynomial
        {
            Summands = [(1, 0, 9.0), (2, 0, -22.5), (1, 1, -22.5), (3, 0, 13.5), (2, 1, 27.0), (1, 2, 13.5)]
        };

        /// <summary>
        /// L3L1L1 = 4.5 * (1-ξ-η) * ξ * (3ξ - 1)
        /// </summary>
        public static readonly IPolynomial L3L1L1 = new Polynomial
        {
            Summands = [(2, 0, 18.0), (1, 0, -4.5), (3, 0, -13.5), (2, 1, -13.5), (1, 1, 4.5)]
        };

        /// <summary>
        /// L1L2L3 = 27.0 * ξ * η * (1-ξ-η) = 27ξη - 27ξ^2η - 27ξη^2
        /// </summary>
        public static readonly IPolynomial L1L2L3 = new Polynomial
        {
            Summands = [(1, 1, 27.0), (2, 1, -27.0), (1, 2, -27.0)]
        };

    }

}
