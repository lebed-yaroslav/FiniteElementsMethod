using Telma;

namespace Model.Model.Basis;

public static class TriangleBasis
{
    // Linear:

    /// <summary>L1(xi, eta) = xi</summary>
    public static readonly IBasisFunction L1 = new TriangleL1();
    /// <summary>L2(xi, eta) = eta</summary>
    public static readonly IBasisFunction L2 = new TriangleL2();
    /// <summary>L3(xi, eta) = 1 - xi - eta</summary>
    public static readonly IBasisFunction L3 = new TriangleL3();

    // Quadratic:

    /// <summary>L1L2(xi, eta) = xi * eta</summary>
    public static readonly IBasisFunction L1L2 = new TriangleL1L2();
    /// <summary>L2L2(xi, eta) = eta * (1 - xi - eta)</summary>
    public static readonly IBasisFunction L2L3 = new TriangleL2L3();
    /// <summary>L3L1(xi, eta) = eta * (1 - xi - eta)</summary>
    public static readonly IBasisFunction L3L1 = new TriangleL3L1();

    // Cubic:

    /// <summary>L1L2L3(xi, eta) = xi * eta * (1 - xi - eta)</summary>
    public static readonly IBasisFunction L1L2L3 = new TriangleL1L2L3();


    private readonly struct TriangleL1 : IBasisFunction
    {
        public double Value(Vector2D localCoords) => localCoords.X; // xi
        public Vector2D Derivatives(Vector2D localCoords) => Vector2D.XAxis; // [d/dxi, d/deta]
    }

    private readonly struct TriangleL2 : IBasisFunction
    {
        public double Value(Vector2D localCoords) => localCoords.Y; // eta
        public Vector2D Derivatives(Vector2D localCoords) => Vector2D.YAxis; // [d/dxi, d/deta]
    }

    private readonly struct TriangleL3 : IBasisFunction
    {
        public double Value(Vector2D localCoords) => 1 - localCoords.X - localCoords.Y; // eta
        public Vector2D Derivatives(Vector2D localCoords) => new(-1.0, -1.0); // [d/dxi, d/deta]
    }

    private readonly struct TriangleL1L2 : IBasisFunction
    {
        public double Value(Vector2D localCoords) => localCoords.X * localCoords.Y; // xi * eta
        public Vector2D Derivatives(Vector2D localCoords)
        {
            (var xi, var eta) = localCoords;
            return new(eta, xi); // [d/dxi, d/deta]
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
            return new(-eta, 1 - xi - 2 * eta); // [d/dxi, d/deta]
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
            return new(1 - 2 * xi - eta, -xi); // [d/dxi, d/deta]
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
            return new(eta * (1 - 2 * xi - eta), xi * (1 - xi - 2 * eta)); // [d/dxi, d/deta]
        }
    }
}
