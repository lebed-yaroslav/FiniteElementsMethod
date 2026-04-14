using Model.Fem.Elements.Quadrangle;
using Model.Fem.Elements.Segment;
using Model.Fem.Elements.Triangle;

namespace Model.Fem.Elements;

public static class FiniteElements
{
    public static class Segment
    {
        public static readonly IBoundaryElementFactory2D Linear = new LinearSegmentFactory();
        public static readonly IBoundaryElementFactory2D HierarchicalQuadratic = new HierarchicalQuadraticSegmentFactory();
        public static readonly IBoundaryElementFactory2D LagrangeCubic = new LagrangeCubicSegmentFactory();
        public static readonly IBoundaryElementFactory2D LagrangeQuadratic = new LagrangeQuadraticSegmentFactory();
        public static readonly IBoundaryElementFactory2D Hermite = new HermiteSegmentFactory();
    }

    public static class Triangle
    {
        public static readonly IFiniteElementFactory2D Linear = new LinearTriangleFactory();
        public static readonly IFiniteElementFactory2D HierarchicalQuadratic = new HierarchicalQuadraticTriangleFactory();
        public static readonly IFiniteElementFactory2D HierarchicalCubic = new HierarchicalCubicTriangleFactory();
        public static readonly IFiniteElementFactory2D LagrangeCubic = new LagrangeCubicTriangleFactory();
    }

    public static class Quadrangle
    {
        public static readonly IFiniteElementFactory2D Bilinear = new BilinearQuadrangleFactory();
        public static readonly IFiniteElementFactory2D LagrangeQuadratic = new BiQuadraticLagrangeQuadrangleFactory();
        public static readonly IFiniteElementFactory2D LagrangeCubic = new BicubicLagrangeQuadrangleFactory();
        public static readonly IFiniteElementFactory2D Hermite = new HermiteQuadrangleFactory();
    }
}
