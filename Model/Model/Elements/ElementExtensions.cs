using Model.Core.CoordinateSystem;
using Model.Model.Basis;
using Model.Model.Mesh;
using Telma;
using Telma.Extensions;

namespace Model.Model.Elements;


public static class FiniteElementExtensions
{
    extension<TSpace>(IFiniteElement<TSpace> self)
        where TSpace : IVectorBase<TSpace>
    {
        public IMesh<TSpace> Mesh => self.Geometry.Mesh;
        public ReadOnlySpan<int> Vertices => self.Geometry.Vertices;
        public IEnumerable<Edge> Edges => self.Geometry.Edges;
        public int EdgeCount => self.Geometry.EdgeCount;
        public ICoordinateTransform<TSpace, TSpace> MasterElementCoordinateSystem => self.Geometry.MasterElementCoordinateSystem;

        public IEnumerable<Quadratures.Node<TSpace>> Quadratures => self.BasisSet.Quadratures;
        public ReadOnlySpan<IBasisFunction<TSpace>> Basis => self.BasisSet.Basis;
    }

    extension<TSpace, TBoundary>(IBoundaryElement<TSpace, TBoundary> self)
        where TSpace : IVectorBase<TSpace>
        where TBoundary : IVectorBase<TBoundary>
    {
        public IMesh<TSpace> Mesh => self.Geometry.Mesh;
        public ReadOnlySpan<int> Vertices => self.Geometry.Vertices;
        public IEnumerable<Edge> Edges => self.Geometry.Edges;
        public int EdgeCount => self.Geometry.EdgeCount;
        public ICoordinateTransform<TSpace, TBoundary> MasterElementCoordinateSystem => self.Geometry.MasterElementCoordinateSystem;

        public IEnumerable<Quadratures.Node<TBoundary>> Quadratures => self.BasisSet.Quadratures;
        public ReadOnlySpan<IBasisFunction<TBoundary>> Basis => self.BasisSet.Basis;
    }

    extension(IDofManager self)
    {
        public int Count => self.Dof.Length;
    }

    extension<TSpace>(IElementGeometryBase<TSpace> self)
        where TSpace : IVectorBase<TSpace>
    {
        public int VertexCount => self.Vertices.Length;
    }
}
