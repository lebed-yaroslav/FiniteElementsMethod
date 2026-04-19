using Model.Fem.Mesh;
using Telma.Extensions;

namespace Model.Fem.Elements;


public sealed record FiniteElementBase<TSpace>(
    IElementGeometryBase<TSpace> Geometry,
    IElementDof DOF
) where TSpace : IVectorBase<TSpace>;

public interface IFiniteElementBase<TSpace, TBoundary>
    where TSpace : IVectorBase<TSpace>
    where TBoundary : IVectorBase<TBoundary>
{
    IElementGeometry<TSpace, TBoundary> Geometry { get; }
    IElementDof DOF { get; }
    IBasisSet<TBoundary> BasisSet { get; }
}

public interface IFiniteElement<TSpace> : IFiniteElementBase<TSpace, TSpace>
    where TSpace : IVectorBase<TSpace>
{
    int MaterialIndex { get; }
}

public interface IBoundaryElement<TSpace, TBoundary> : IFiniteElementBase<TSpace, TBoundary>
    where TSpace : IVectorBase<TSpace>
    where TBoundary : IVectorBase<TBoundary>
{
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


public sealed record class VolumeElement<TSpace>(
    IElementGeometry<TSpace, TSpace> Geometry,
    IElementDof DOF,
    IBasisSet<TSpace> BasisSet,
    int MaterialIndex
) : IFiniteElement<TSpace>
    where TSpace : IVectorBase<TSpace>;


public sealed record class FiniteElement<TSpace, TBoundary>(
    IElementGeometry<TSpace, TBoundary> Geometry,
    IElementDof DOF,
    IBasisSet<TBoundary> BasisSet,
    int BoundaryIndex
) : IBoundaryElement<TSpace, TBoundary>
    where TSpace : IVectorBase<TSpace>
    where TBoundary : IVectorBase<TBoundary>;
