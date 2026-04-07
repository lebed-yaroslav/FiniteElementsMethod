using System.Diagnostics;
using Model.Model.Elements;
using Model.Model.Mesh;
using Telma;
using Telma.Extensions;

namespace Model.Core.Util;

/// <summary>
/// Axis aligned bounding box: https://en.wikipedia.org/wiki/Minimum_bounding_box
/// </summary>
public struct BoundingBox<TSpace>
    where TSpace : IVectorBase<TSpace>
{
    public TSpace Min { get; private set; }
    public TSpace Max { get; private set; }

    public BoundingBox(TSpace min, TSpace max)
    {
        Debug.Assert(min <= max);
        Min = min;
        Max = max;
    }

    public readonly void Deconstruct(out TSpace min, out TSpace max)
    {
        min = Min; max = Max;
    }

    public readonly TSpace Center => (Min + Max) * 0.5;

    public static BoundingBox<TSpace> FromPoints(IEnumerable<TSpace> points)
    {
        var result = new BoundingBox<TSpace>();
        foreach (var point in points) result.IncludePoint(point);
        return result;
    }

    public readonly bool Contains(TSpace point)
        => Min <= point && point <= Max;

    public readonly bool Contains(BoundingBox<TSpace> other)
        => Min <= other.Min && other.Max <= Max;

    public readonly bool Intersects(BoundingBox<TSpace> other)
        => Max >= other.Min && other.Max >= Min;

    public void IncludePoint(TSpace point)
    {
        Min = TSpace.Min(Min, point);
        Max = TSpace.Max(Max, point);
    }
}

public static class BoundingBoxExtensions
{
    extension<TSpace>(IMesh<TSpace> mesh)
        where TSpace : IVectorBase<TSpace>
    {
        public BoundingBox<TSpace> GetBoundingBox()
        {
            var boundingBox = new BoundingBox<TSpace>();
            for (int i = 0; i < mesh.VertexCount; i++)
                boundingBox.IncludePoint(point: mesh[i]);
            return boundingBox;
        }
    }

    extension<TSpace, TBoundary>(IFiniteElementBase<TSpace, TBoundary> element)
        where TSpace : IVectorBase<TSpace>
        where TBoundary : IVectorBase<TBoundary>
    {
        public BoundingBox<TSpace> GetBoundingBox()
            => BoundingBox<TSpace>.FromPoints(element.VertexCoords);
    }
}
