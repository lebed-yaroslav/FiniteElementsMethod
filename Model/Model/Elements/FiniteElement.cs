global using IFiniteElement1D = Model.Model.Elements.IFiniteElement<Telma.Vector1D>;
global using IFiniteElement2D = Model.Model.Elements.IFiniteElement<Telma.Vector2D>;
global using IFiniteElement3D = Model.Model.Elements.IFiniteElement<Telma.Vector3D>;
global using FiniteElement1D = Model.Model.Elements.VolumeElement<Telma.Vector1D>;
global using FiniteElement2D = Model.Model.Elements.VolumeElement<Telma.Vector2D>;
global using FiniteElement3D = Model.Model.Elements.VolumeElement<Telma.Vector3D>;

global using IFiniteElementFactory1D = Model.Model.Elements.IFiniteElementFactory<Telma.Vector1D>;
global using IFiniteElementFactory2D = Model.Model.Elements.IFiniteElementFactory<Telma.Vector2D>;
global using IFiniteElementFactory3D = Model.Model.Elements.IFiniteElementFactory<Telma.Vector3D>;

global using IBoundaryElement2D = Model.Model.Elements.IBoundaryElement<Telma.Vector2D, Telma.Vector1D>;
global using IBoundaryElement3D = Model.Model.Elements.IBoundaryElement<Telma.Vector3D, Telma.Vector2D>;
global using BoundaryElement2D = Model.Model.Elements.FiniteElement<Telma.Vector2D, Telma.Vector1D>;
global using BoundaryElement3D = Model.Model.Elements.FiniteElement<Telma.Vector3D, Telma.Vector2D>;

global using IBoundaryElementFactory2D = Model.Model.Elements.IBoundaryElementFactory<Telma.Vector2D, Telma.Vector1D>;
global using IBoundaryElementFactory3D = Model.Model.Elements.IBoundaryElementFactory<Telma.Vector3D, Telma.Vector2D>;

using Model.Model.Mesh;
using Telma.Extensions;

namespace Model.Model.Elements;


public sealed record FiniteElementBase<TSpace>(
    IElementGeometryBase<TSpace> Geometry,
    IDofManager DOF
) where TSpace : IVectorBase<TSpace>;

public interface IFiniteElementBase<TSpace, TBoundary>
    where TSpace : IVectorBase<TSpace>
    where TBoundary : IVectorBase<TBoundary>
{
    IElementGeometry<TSpace, TBoundary> Geometry { get; }
    IDofManager DOF { get; }
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
    IDofManager DOF,
    IBasisSet<TSpace> BasisSet,
    int MaterialIndex
) : IFiniteElement<TSpace>
    where TSpace : IVectorBase<TSpace>;


public sealed record class FiniteElement<TSpace, TBoundary>(
    IElementGeometry<TSpace, TBoundary> Geometry,
    IDofManager DOF,
    IBasisSet<TBoundary> BasisSet,
    int BoundaryIndex
) : IBoundaryElement<TSpace, TBoundary>
    where TSpace : IVectorBase<TSpace>
    where TBoundary : IVectorBase<TBoundary>;
