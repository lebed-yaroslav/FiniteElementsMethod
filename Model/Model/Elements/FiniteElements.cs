using Model.Model.Elements.Triangle;
using Telma;

namespace Model.Model.Elements;

public static class FiniteElements
{
    public static readonly IFiniteElementFactory2D LinearTriangle = new LinearTriangleFactory();
    public static readonly IFiniteElementFactory2D HierarchicalQuadraticTriangle = new HierarchicalQuadraticTriangleFactory();
}
