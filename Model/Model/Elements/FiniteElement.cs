using Model.Model.Mesh;
using Telma.Extensions;

namespace Model.Model.Elements;


public sealed record FiniteElementBase<TSpace>(
    IElementGeometry<TSpace> Geometry,
    IDofManager DOF
) where TSpace : IVectorBase<TSpace>;


public interface IFiniteElement<TSpace>
    where TSpace : IVectorBase<TSpace>
{
    IVolumeElementGeometry<TSpace> Geometry { get; }
    IDofManager DOF { get; }
    IBasisSet<TSpace> BasisSet { get; }
    int MaterialIndex { get; }
}


public interface IBoundaryElement<TSpace, TBoundary>
    where TSpace : IVectorBase<TSpace>
    where TBoundary : IVectorBase<TBoundary>
{
    IBoundaryElementGeometry<TSpace, TBoundary> Geometry { get; }
    IDofManager DOF { get; }
    IBasisSet<TBoundary> BasisSet { get; }
    int BoundaryIndex { get; }
}


public interface IFiniteElementFactory<TSpace>
    where TSpace : IVectorBase<TSpace>
{
    IFiniteElement<TSpace> CreateElement(IMesh<TSpace> mesh, int[] vertices, int materialIndex);
}


public interface IBoundaryElementFactory<TSpace, TBoundary>
    where TSpace : IVectorBase<TSpace>
    where TBoundary : IVectorBase<TBoundary>
{
    IBoundaryElement<TSpace, TBoundary> CreateBoundary(IMesh<TSpace> mesh, int[] vertices, int boundaryIndex);
}


public sealed record class FiniteElement<TSpace>(
    IVolumeElementGeometry<TSpace> Geometry,
    IDofManager DOF,
    IBasisSet<TSpace> BasisSet,
    int MaterialIndex
) : IFiniteElement<TSpace>
    where TSpace : IVectorBase<TSpace>;


public sealed record class BoundaryElement<TSpace, TBoundary>(
    IBoundaryElementGeometry<TSpace, TBoundary> Geometry,
    IDofManager DOF,
    IBasisSet<TBoundary> BasisSet,
    int BoundaryIndex
) : IBoundaryElement<TSpace, TBoundary>
    where TSpace : IVectorBase<TSpace>
    where TBoundary : IVectorBase<TBoundary>;
