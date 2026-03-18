using Model.Model.Elements;
using Telma.Extensions;

namespace Model.Model.Mesh;

public static class MeshExtensions
{
    extension<TSpace, TBoundary>(IMeshWithBoundaries<TSpace, TBoundary> self)
        where TSpace : IVectorBase<TSpace>
        where TBoundary : IVectorBase<TBoundary>
    {
        public IEnumerable<FiniteElementBase<TSpace>> AllElements => self.FiniteElements
            .Select(e => new FiniteElementBase<TSpace>(e.Geometry, e.DOF))
            .Concat(self.BoundaryElements.Select(e => new FiniteElementBase<TSpace>(e.Geometry, e.DOF)));

        public IEnumerable<IDofManager> ElementsDof => self.FiniteElements
            .Select(e => e.DOF)
            .Concat(self.BoundaryElements.Select(e => e.DOF));
    }
}
