using Model.Fem.Elements;
using Model.Fem.Problem;
using Telma.Extensions;

namespace Model.Fem.Mesh;


public static class MeshExtensions
{
    extension<TSpace, TBoundary>(IMeshWithBoundaries<TSpace, TBoundary> self)
        where TSpace : IVectorBase<TSpace>
        where TBoundary : IVectorBase<TBoundary>
    {
        public IEnumerable<FiniteElementBase<TSpace>> AllElements => self.FiniteElements
            .Select(e => new FiniteElementBase<TSpace>(e.Geometry, e.DOF))
            .Concat(self.BoundaryElements.Select(e => new FiniteElementBase<TSpace>(e.Geometry, e.DOF)));

        public IEnumerable<IElementDof> AllElementsDof => self.FiniteElements
            .Select(e => e.DOF)
            .Concat(self.BoundaryElements.Select(e => e.DOF));

        public IEnumerable<IBoundaryElement<TSpace, TBoundary>> FixedElementsBy(BoundaryCondition<TSpace>[] boundaryConditions)
            => self.BoundaryElements
                .Where(e => boundaryConditions[e.BoundaryIndex] is BoundaryCondition<TSpace>.Dirichlet);
    }

    extension<TSpace, TBoundary>(Mesh<TSpace, TBoundary> self)
        where TSpace : IVectorBase<TSpace>
        where TBoundary : IVectorBase<TBoundary>
    {
        public void AddVertices(params IEnumerable<TSpace> vertices)
        {
            foreach (var vertex in vertices)
                self.AddVertex(vertex);
        }
    }
}
