using Model.Fem.Integrator;
using Telma;

namespace Model.Fem.Basis;


public static partial class TriangleBasis
{
    // Basis functions for hierarchical triangles, formulas are taken from (12.23) [1, p594]

    // Linear:
    private static readonly Polynomial TriangleL1 = new()
    {
        Summands = {
            [(1, 0)] = 1.0
        }
    };

    private static readonly Polynomial TriangleL2 = new()
    {
        Summands = {
            [(0, 1)] = 1.0
        }
    };

    private static readonly Polynomial TriangleL3 = new()
    {
        Summands = {
            [(0, 0)] = 1.0,
            [(1, 0)] = -1.0,
            [(0, 1)] = -1.0,
        }
    };

    /// <summary>L1(ξ, η) = ξ</summary>
    public static readonly IBasisFunction2D L1 = TriangleL1;
    /// <summary>L2(ξ, η) = η</summary>
    public static readonly IBasisFunction2D L2 = TriangleL2;
    /// <summary>L3(ξ, η) = 1 - ξ - η</summary>
    public static readonly IBasisFunction2D L3 = TriangleL3;

    // Quadratic:
    private static readonly Polynomial TriangleL1L2 = new()
    {
        Summands = {
            [(1, 1)] = 1.0
        }
    };

    private static readonly Polynomial TriangleL2L3 = new()
    {
        Summands = {
            [(1, 1)] = -1.0,
            [(0, 2)] = -1.0,
            [(0, 1)] = 1.0
        }
    };

    private static readonly Polynomial TriangleL3L1 = new()
    {
        Summands = {
            [(1, 1)] = -1.0,
            [(2, 0)] = -1.0,
            [(1, 0)] = 1.0
        }
    };

    /// <summary>L1L2(ξ, η) = ξ * η</summary>
    public static readonly IBasisFunction2D L1L2 = TriangleL1L2;
    /// <summary>L2L3(ξ, η) = η * (1 - ξ - η)</summary>
    public static readonly IBasisFunction2D L2L3 = TriangleL2L3;
    /// <summary>L3L1(ξ, η) = ξ * (1 - ξ - η)</summary>
    public static readonly IBasisFunction2D L3L1 = TriangleL3L1;

    // Cubic:
    private static readonly Polynomial TriangleL1L2L3 = new()
    {
        Summands = {
            [(1, 1)] = 1.0,
            [(2, 1)] = -1.0,
            [(1, 2)] = -1.0
        }
    };

    private static readonly Polynomial TriangleL1L2L2SubL1 = new()
    {
        Summands = {
            [(1, 2)] = 1.0,
            [(2, 1)] = -1.0,
        }
    };

    private static readonly Polynomial TriangleL1L3L1SubL3 = new()
    {
        Summands = {
            [(3, 0)] = -2.0,
            [(2, 1)] = -3.0,
            [(2, 0)] = 3.0,
            [(1, 2)] = -1.0,
            [(1, 1)] = 2.0,
            [(1, 0)] = -1.0
        }
    };

    private static readonly Polynomial TriangleL2L3L2SubL3 = new()
    {
        Summands = {
            [(2, 1)] = -1.0,
            [(1, 2)] = -3.0,
            [(1, 1)] = 2.0,
            [(0, 3)] = -2.0,
            [(0, 2)] = 3.0,
            [(0, 1)] = -1.0
        }
    };

    /// <summary>L1L3L1SubL3(ξ, η) = ξ * η * (η - ξ)</summary>
    public static readonly IBasisFunction2D L1L2L2SubL1 = TriangleL1L2L2SubL1;
    /// <summary>L1L3L1SubL3(ξ, η) = ξ * (1 - ξ - η) * (2ξ + η - 1)</summary>
    public static readonly IBasisFunction2D L1L3L1SubL3 = TriangleL1L3L1SubL3;
    /// <summary>L1L3L1SubL3(ξ, η) = η * (1 - ξ - η) * (ξ + 2η - 1)</summary>
    public static readonly IBasisFunction2D L2L3L2SubL3 = TriangleL2L3L2SubL3;
    /// <summary>L1L2L3(ξ, η) = ξ * η * (1 - ξ - η)</summary>
    public static readonly IBasisFunction2D L1L2L3 = TriangleL1L2L3;

}
