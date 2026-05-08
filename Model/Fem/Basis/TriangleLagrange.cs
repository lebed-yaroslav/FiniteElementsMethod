using Model.Fem.Integrator;
using Telma;

namespace Model.Fem.Basis;

public static partial class TriangleBasis
{
    public static class Lagrange
    {
        /// <summary>L1L1L1 = 0.5 * (L1) * (3 * L1 - 1) * (3 * L1 - 2)</summary>
        public static readonly IBasisFunction2D L1L1L1 = TriangleLagrangeL1L1L1;
        /// <summary>L2L2L2 = 0.5 * (L2) * (3 * L2 - 1) * (3 * L2 - 2)</summary>
        public static readonly IBasisFunction2D L2L2L2 = TriangleLagrangeL2L2L2;
        /// <summary>L3L3L3 = 0.5 * (L3) * (3 * L3 - 1) * (3 * L3 - 2)</summary>
        public static readonly IBasisFunction2D L3L3L3 = TriangleLagrangeL3L3L3;
        /// <summary>L1L2L1 = 4.5 * (L1) * (L2) * (3 * L1 - 1)</summary>
        public static readonly IBasisFunction2D L1L2L1 = TriangleLagrangeL1L2L1;
        /// <summary>L1L2L2 = 4.5 * (L1) * (L2) * (3 * L2 - 1)</summary>
        public static readonly IBasisFunction2D L1L2L2 = TriangleLagrangeL1L2L2;
        /// <summary>L2L3L2 = 4.5 * (L2) * (L3) * (3 * L2 - 1)</summary>
        public static readonly IBasisFunction2D L2L3L2 = TriangleLagrangeL2L3L2;
        /// <summary>L2L3L3 = 4.5 * (L2) * (L3) * (3 * L3 - 1)</summary>
        public static readonly IBasisFunction2D L2L3L3 = TriangleLagrangeL2L3L3;
        /// <summary>L3L1L3 = 4.5 * (L3) * (L1) * (3 * L3 - 1)</summary>
        public static readonly IBasisFunction2D L3L1L3 = TriangleLagrangeL3L1L3;
        /// <summary>L3L1L1 = 4.5 * (L3) * (L1) * (3 * L1 - 1)</summary>
        public static readonly IBasisFunction2D L3L1L1 = TriangleLagrangeL3L1L1;
        /// <summary>L1L2L3 = 27.0 * (L1) * (L2) * (L3)</summary>
        public static readonly IBasisFunction2D L1L2L3 = TriangleLagrangeL1L2L3;
    }

    private static readonly Polynomial TriangleLagrangeL1L1L1 = new()
    {
        Summands = {
            [(3, 0)] = 4.5,
            [(2, 0)] = -4.5,
            [(1, 0)] = 1.0
        }
    };

    private static readonly Polynomial TriangleLagrangeL2L2L2 = new()
    {
        Summands = {
            [(0, 3)] = 4.5,
            [(0, 2)] = -4.5,
            [(0, 1)] = 1.0
        }
    };

    private static readonly Polynomial TriangleLagrangeL3L3L3 = new()
    {
        Summands = {
            [(0, 0)] = 1.0,

            [(1, 0)] = -5.5,
            [(0, 1)] = -5.5,

            [(2, 0)] = 9.0,
            [(1, 1)] = 18.0,
            [(0, 2)] = 9.0,

            [(3, 0)] = -4.5,
            [(2, 1)] = -13.5,
            [(1, 2)] = -13.5,
            [(0, 3)] = -4.5
        }
    };

    private static readonly Polynomial TriangleLagrangeL1L2L1 = new()
    {
        Summands = {
            [(2, 1)] = 13.5,
            [(1, 1)] = -4.5
        }
    };

    private static readonly Polynomial TriangleLagrangeL1L2L2 = new()
    {
        Summands = {
            [(1, 2)] = 13.5,
            [(1, 1)] = -4.5
        }
    };

    public static readonly Polynomial TriangleLagrangeL2L3L2 = new()
    {
        Summands = {
            [(0, 2)] = 18.0,
            [(0, 1)] = -4.5,

            [(1, 2)] = -13.5,
            [(1, 1)] = 4.5,
            [(0, 3)] = -13.5
        }
    };

    public static readonly Polynomial TriangleLagrangeL2L3L3 = new()
    {
        Summands = {
            [(0, 1)] = 9.0,
            [(1, 1)] = -22.5,
            [(0, 2)] = -22.5,

            [(2, 1)] = 13.5,
            [(1, 2)] = 27.0,
            [(0, 3)] = 13.5
        }
    };

    public static readonly Polynomial TriangleLagrangeL3L1L3 = new()
    {
        Summands = {
            [(1, 0)] = 9.0,
            [(2, 0)] = -22.5,
            [(1, 1)] = -22.5,

            [(3, 0)] = 13.5,
            [(2, 1)] = 27.0,
            [(1, 2)] = 13.5
        }
    };

    public static readonly Polynomial TriangleLagrangeL3L1L1 = new()
    {
        Summands = {
            [(2, 0)] = 18.0,
            [(1, 0)] = -4.5,
            [(3, 0)] = -13.5,
            [(2, 1)] = -13.5,
            [(1, 1)] = 4.5
        }
    };

    public static readonly Polynomial TriangleLagrangeL1L2L3 = new()
    {
        Summands = {
            [(1, 1)] = 27.0,
            [(2, 1)] = -27.0,
            [(1, 2)] = -27.0
        }
    };
}
