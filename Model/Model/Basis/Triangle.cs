using Telma;

namespace Model.Model.Basis;

public static class TriangleBasis
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

   //Lagrange:

   /// <summary>L1L1L1 = 1.0 / 2.0 * (L1) * (3 * L1 - 1) * (3 * L1 - 2)</summary>
   public static readonly IBasisFunction LagrangeL1L1L1 = new TriangleLagrangeL1L1L1();
   /// <summary>L2L2L2 = 1.0 / 2.0 * (L2) * (3 * L2 - 1) * (3 * L2 - 2)</summary>
   public static readonly IBasisFunction LagrangeL2L2L2 = new TriangleLagrangeL2L2L2();
   /// <summary>L3L3L3 = 1.0 / 2.0 * (L3) * (3 * L3 - 1) * (3 * L3 - 2)</summary>
   public static readonly IBasisFunction LagrangeL3L3L3 = new TriangleLagrangeL3L3L3();
   /// <summary>L1L2L1 = 9.0 / 2.0 * (L1) * (L2) * (3 * L1 - 1)</summary>
   public static readonly IBasisFunction LagrangeL1L2L1 = new TriangleLagrangeL1L2L1();
   /// <summary>L1L2L2 = 9.0 / 2.0 * (L1) * (L2) * (3 * L2 - 1)</summary>
   public static readonly IBasisFunction LagrangeL1L2L2 = new TriangleLagrangeL1L2L2();
   /// <summary>L2L3L2 = 9.0 / 2.0 * (L2) * (L3) * (3 * L2 - 1)</summary>
   public static readonly IBasisFunction LagrangeL2L3L2 = new TriangleLagrangeL2L3L2();
   /// <summary>L2L3L3 = 9.0 / 2.0 * (L2) * (L3) * (3 * L3 - 1)</summary>
   public static readonly IBasisFunction LagrangeL2L3L3 = new TriangleLagrangeL2L3L3();
   /// <summary>L3L1L3 = 9.0 / 2.0 * (L3) * (L1) * (3 * L3 - 1)</summary>
   public static readonly IBasisFunction LagrangeL3L1L3 = new TriangleLagrangeL3L1L3();
   /// <summary>L3L1L1 = 9.0 / 2.0 * (L3) * (L1) * (3 * L1 - 1)</summary>
   public static readonly IBasisFunction LagrangeL3L1L1 = new TriangleLagrangeL3L1L1();
   /// <summary>L1L2L3 = 27.0 * (L1) * (L2) * (L3)</summary>
   public static readonly IBasisFunction LagrangeL1L2L3 = new TriangleLagrangeL1L2L3();

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
                eta * (eta - 2 * xi ), // d/dξ
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


   // Lagrange 
   private readonly struct TriangleLagrangeL1L1L1 : IBasisFunction
   {
      public double Value(Vector2D localCoords) => 1.0 / 2.0 * (localCoords.Y) * (3 * localCoords.Y - 1) * (3 * localCoords.Y - 2);
      public Vector2D Derivatives(Vector2D localCoords) => new(0.5 * (27 * localCoords.Y * localCoords.Y - 18 * localCoords.Y + 2), 0.0);
   }

   private readonly struct TriangleLagrangeL2L2L2 : IBasisFunction
   {
      public double Value(Vector2D localCoords) => 1.0 / 2.0 * (localCoords.Y) * (3 * localCoords.Y - 1) * (3 * localCoords.Y - 2);
      public Vector2D Derivatives(Vector2D localCoords) => new(0.0, 0.5 * (27 * localCoords.Y * localCoords.Y - 18 * localCoords.Y + 2));
   }

   private readonly struct TriangleLagrangeL3L3L3 : IBasisFunction
   {
      public double Value(Vector2D localCoords)
      {
         (var xi, var eta) = localCoords;
         return 1.0 / 2.0 * (1 - xi - eta) * (3 * (1 - xi - eta) - 1) * (3 * (1 - xi - eta) - 2);
      }
      public Vector2D Derivatives(Vector2D localCoords)
      {
         (var xi, var eta) = localCoords;
         return new(0.5 * (-27 * xi * xi - 54 * xi * eta + 36 * xi - 27 * eta * eta + 36 * eta - 11),
                    0.5 * (-27 * xi * xi - 54 * xi * eta + 36 * xi - 27 * eta * eta + 36 * eta - 11)); // [d/dxi, d/deta]
      }
   }

   private readonly struct TriangleLagrangeL1L2L1 : IBasisFunction
   {
      public double Value(Vector2D localCoords)
      {
         (var xi, var eta) = localCoords;
         return 9.0 / 2.0 * (xi) * (eta) * (3 * xi - 1);
      }
      public Vector2D Derivatives(Vector2D localCoords)
      {
         (var xi, var eta) = localCoords;
         return new(9.0 / 2.0 * (6 * xi * eta - eta),
                    9.0 / 2.0 * (3 * xi * xi - xi)); // [d/dxi, d/deta]
      }
   }

   private readonly struct TriangleLagrangeL1L2L2 : IBasisFunction
   {
      public double Value(Vector2D localCoords)
      {
         (var xi, var eta) = localCoords;
         return 9.0 / 2.0 * (xi) * (eta) * (3 * eta - 1);
      }
      public Vector2D Derivatives(Vector2D localCoords)
      {
         (var xi, var eta) = localCoords;
         return new(9.0 / 2.0 * (3 * eta * eta - eta),
                    9.0 / 2.0 * (6 * xi * eta - xi)); // [d/dxi, d/deta]
      }
   }

   private readonly struct TriangleLagrangeL2L3L2 : IBasisFunction
   {
      public double Value(Vector2D localCoords)
      {
         (var xi, var eta) = localCoords;
         return 9.0 / 2.0 * (eta) * (1 - xi - eta) * (3 * eta - 1);
      }
      public Vector2D Derivatives(Vector2D localCoords)
      {
         (var xi, var eta) = localCoords;
         return new(9.0 / 2.0 * (-3 * eta * eta + eta),
                    9.0 / 2.0 * (-6 * xi * eta + xi - 9 * eta * eta + 8 * eta - 1)); // [d/dxi, d/deta]
      }
   }

   private readonly struct TriangleLagrangeL2L3L3 : IBasisFunction
   {
      public double Value(Vector2D localCoords)
      {
         (var xi, var eta) = localCoords;
         return 9.0 / 2.0 * (eta) * (1 - xi - eta) * (3 * (1 - xi - eta) - 1);
      }
      public Vector2D Derivatives(Vector2D localCoords)
      {
         (var xi, var eta) = localCoords;
         return new(9.0 / 2.0 * (6 * eta * eta + 6 * xi * eta - 5 * eta),
                    9.0 / 2.0 * (3 * xi * xi + 12 * xi * eta - 5 * xi + 9 * eta * eta - 10 * eta + 2)); // [d/dxi, d/deta]
      }
   }

   private readonly struct TriangleLagrangeL3L1L3 : IBasisFunction
   {
      public double Value(Vector2D localCoords)
      {
         (var xi, var eta) = localCoords;
         return 9.0 / 2.0 * (1 - xi - eta) * (xi) * (3 * (1 - xi - eta) - 1);
      }
      public Vector2D Derivatives(Vector2D localCoords)
      {
         (var xi, var eta) = localCoords;
         return new(9.0 / 2.0 * (9 * xi * xi + 12 * xi * eta - 10 * xi + 3 * eta * eta - 5 * eta + 2),
                    9.0 / 2.0 * (6 * xi * xi + 6 * xi * eta - 5 * xi)); // [d/dxi, d/deta]
      }
   }

   private readonly struct TriangleLagrangeL3L1L1 : IBasisFunction
   {
      public double Value(Vector2D localCoords)
      {
         (var xi, var eta) = localCoords;
         return 9.0 / 2.0 * (1 - xi - eta) * (xi) * (3 * xi - 1);
      }
      public Vector2D Derivatives(Vector2D localCoords)
      {
         (var xi, var eta) = localCoords;
         return new(9.0 / 2.0 * (-9 * xi * xi - 6 * xi * eta + 8 * xi + 1 * eta - 1),
                    9.0 / 2.0 * (-3 * xi * xi + xi)); // [d/dxi, d/deta]
      }
   }

   private readonly struct TriangleLagrangeL1L2L3 : IBasisFunction
   {
      public double Value(Vector2D localCoords)
      {
         (var xi, var eta) = localCoords;
         return 27.0 * (xi) * (eta) * (1 - xi - eta);
      }
      public Vector2D Derivatives(Vector2D localCoords)
      {
         (var xi, var eta) = localCoords;
         return new(27.0 * (-2 * xi * eta - eta * eta + eta),
                    27.0 * (-2 * xi * eta - xi * xi + xi)); // [d/dxi, d/deta]
      }
   }
}


