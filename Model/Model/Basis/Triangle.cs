using Telma;

namespace Model.Model.Basis;

public static partial class TriangleBasis
{
    // Basis functions for hierarchical triangles, formulas are taken from (12.23) [1, p594]

    // Linear:

    /// <summary>L1(ξ, η) = ξ</summary>
    public static readonly IBasisFunction L1 = new TriangleL1();
    /// <summary>L2(ξ, η) = η</summary>
    public static readonly IBasisFunction L2 = new TriangleL2();
    /// <summary>L3(ξ, η) = 1 - ξ - η</summary>
    public static readonly IBasisFunction L3 = new TriangleL3();

    // Quadratic:

    /// <summary>L1L2(ξ, η) = ξ * η</summary>
    public static readonly IBasisFunction L1L2 = new TriangleL1L2();
    /// <summary>L2L3(ξ, η) = η * (1 - ξ - η)</summary>
    public static readonly IBasisFunction L2L3 = new TriangleL2L3();
    /// <summary>L3L1(ξ, η) = ξ * (1 - ξ - η)</summary>
    public static readonly IBasisFunction L3L1 = new TriangleL3L1();

    // Cubic:

    /// <summary>L1L3L1SubL3(ξ, η) = ξ * η * (η - ξ)</summary>
    public static readonly IBasisFunction L1L2L2SubL1 = new TriangleL1L2L2SubL1();
    /// <summary>L1L3L1SubL3(ξ, η) = ξ * (1 - ξ - η) * (2ξ + η - 1)</summary>
    public static readonly IBasisFunction L1L3L1SubL3 = new TriangleL1L3L1SubL3();
    /// <summary>L1L3L1SubL3(ξ, η) = η * (1 - ξ - η) * (ξ + 2η - 1)</summary>
    public static readonly IBasisFunction L2L3L2SubL3 = new TriangleL2L3L2SubL3();
    /// <summary>L1L2L3(ξ, η) = ξ * η * (1 - ξ - η)</summary>
    public static readonly IBasisFunction L1L2L3 = new TriangleL1L2L3();

    private readonly struct TriangleL1 : IBasisFunction
    {
        public double Value(Vector2D localCoords) => localCoords.X; // ξ
        public Vector2D Derivatives(Vector2D localCoords) => Vector2D.XAxis; // [d/dξ, d/dη]
    }

    private readonly struct TriangleL2 : IBasisFunction
    {
        public double Value(Vector2D localCoords) => localCoords.Y; // η
        public Vector2D Derivatives(Vector2D localCoords) => Vector2D.YAxis; // [d/dξ, d/dη]
    }

    private readonly struct TriangleL3 : IBasisFunction
    {
        public double Value(Vector2D localCoords) => 1 - localCoords.X - localCoords.Y; // (1 - ξ - η)
        public Vector2D Derivatives(Vector2D localCoords) => new(-1.0, -1.0); // [d/dξ, d/dη]
    }

    private readonly struct TriangleL1L2 : IBasisFunction
    {
        public double Value(Vector2D localCoords) => localCoords.X * localCoords.Y; // ξ * η
        public Vector2D Derivatives(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return new(eta, xi); // [d/dξ, d/dη]
        }
    }

    private readonly struct TriangleL2L3 : IBasisFunction
    {
        public double Value(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return eta * (1 - xi - eta);
        }

        public Vector2D Derivatives(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return new(-eta, 1 - xi - 2 * eta); // [d/dξ, d/dη]
        }
    }

    private readonly struct TriangleL3L1 : IBasisFunction
    {
        public double Value(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return xi * (1 - xi - eta);
        }

        public Vector2D Derivatives(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return new(1 - 2 * xi - eta, -xi); // [d/dξ, d/dη]
        }
    }

    private readonly struct TriangleL1L2L3 : IBasisFunction
    {
        public double Value(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return xi * eta * (1 - xi - eta);
        }

        public Vector2D Derivatives(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return new(eta * (1 - 2 * xi - eta), xi * (1 - xi - 2 * eta)); // [d/dξ, d/dη]
        }
    }

    private struct TriangleL1L2L2SubL1() : IBasisFunction
    {
        public readonly double Value(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return xi * eta * (eta - xi);
        }

        public readonly Vector2D Derivatives(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return new(
                eta * (eta - 2 * xi), // d/dξ
                xi * (2 * eta - xi) // d/dη
            );
        }
    }

    private struct TriangleL1L3L1SubL3() : IBasisFunction
    {
        public readonly double Value(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return xi * (1 - xi - eta) * (2 * xi + eta - 1);
        }

        public readonly Vector2D Derivatives(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return new(
                6 * (1 - xi - eta) * xi + (1 - eta) * (eta - 1), // d/dξ
                xi * (2 - 2 * eta - 3 * xi) // d/dη
            );
        }
    }

    private struct TriangleL2L3L2SubL3() : IBasisFunction
    {
        public readonly double Value(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return eta * (1 - xi - eta) * (xi + 2 * eta - 1);
        }

        public readonly Vector2D Derivatives(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return new(
                eta * (2 - 2 * xi - 3 * eta), // d/dξ
                6 * (1 - xi - eta) * eta + (1 - xi) * (xi - 1) // d/dη
            );
        }
    }
}
