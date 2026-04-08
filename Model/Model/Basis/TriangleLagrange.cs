using Telma;

namespace Model.Model.Basis;

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

    private readonly struct TriangleLagrangeL1L1L1 : IBasisFunction2D
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
    }

    private readonly struct TriangleLagrangeL2L2L2 : IBasisFunction2D
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
    }

    private readonly struct TriangleLagrangeL3L3L3 : IBasisFunction2D
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
    }

    private readonly struct TriangleLagrangeL1L2L1 : IBasisFunction2D
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
    }

    private readonly struct TriangleLagrangeL1L2L2 : IBasisFunction2D
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
    }

    private readonly struct TriangleLagrangeL2L3L2 : IBasisFunction2D
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
    }

    private readonly struct TriangleLagrangeL2L3L3 : IBasisFunction2D
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
    }

    private readonly struct TriangleLagrangeL3L1L3 : IBasisFunction2D
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
    }

    private readonly struct TriangleLagrangeL3L1L1 : IBasisFunction2D
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
    }

    private readonly struct TriangleLagrangeL1L2L3 : IBasisFunction2D
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
    }
}
