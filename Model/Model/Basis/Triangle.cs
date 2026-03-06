using Telma;

namespace Model.Model.Basis;

public static class TriangleBasis
{
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
    /// <summary>L2L2(ξ, η) = η * (1 - ξ - η)</summary>
    public static readonly IBasisFunction L2L3 = new TriangleL2L3();
    /// <summary>L3L1(ξ, η) = ξ * (1 - ξ - η)</summary>
    public static readonly IBasisFunction L3L1 = new TriangleL3L1();

    // Cubic:

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
}
