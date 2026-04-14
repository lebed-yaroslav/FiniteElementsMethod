using System.Diagnostics;
using Model.Core.Util;
using Model.Fem.Elements;
using Telma.Extensions;

namespace Model.Fem.Mesh;


public interface ISearchTree<TSpace>
    where TSpace : IVectorBase<TSpace>
{
    IFiniteElement<TSpace>? FindElementAt(TSpace point);
    IEnumerable<IFiniteElement<TSpace>> QueryIntersection(BoundingBox<TSpace> bounds);
}


public sealed class SearchTree<TSpace> : ISearchTree<TSpace>
    where TSpace : IVectorBase<TSpace>
{
    public BoundingBox<TSpace> Bounds => _root.Bounds;
    private readonly SpatialTreeNode _root;

    private SearchTree(SpatialTreeNode root) => _root = root;

    public static SearchTree<TSpace> BuildFor(
        IMesh<TSpace> mesh,
        int maxElementsPerNode = 4,
        int maxDepth = 8
    )
    {
        var root = new SpatialTreeNode(mesh.GetBoundingBox());
        foreach (var element in mesh.FiniteElements)
            if (!root.TryInsert(element, element.GetBoundingBox(), maxElementsPerNode, maxDepth))
                Debug.WriteLine($"Failed to insert element {element} into search tree");
        return new(root);
    }

    public IFiniteElement<TSpace>? FindElementAt(TSpace point)
        => _root.FindElementAt(point);

    public IEnumerable<IFiniteElement<TSpace>> QueryIntersection(BoundingBox<TSpace> bounds)
        => _root.QueryIntersection(bounds);

    private sealed class SpatialTreeNode(BoundingBox<TSpace> bounds, int depth = 0)
    {
        public bool IsLeaf => _children == null;

        public BoundingBox<TSpace> Bounds { get; } = bounds;
        public int Depth { get; } = depth;

        private SpatialTreeNode[]? _children = null;
        private readonly List<(IFiniteElement<TSpace>, BoundingBox<TSpace>)> _elements = [];

        public bool TryInsert(
            IFiniteElement<TSpace> element,
            BoundingBox<TSpace> elementBounds,
            int maxElementsPerNode,
            int maxDepth
        )
        {
            if (!Bounds.Intersects(elementBounds)) return false;

            if (IsLeaf)
            {
                if (_elements.Count < maxElementsPerNode || Depth >= maxDepth)
                {
                    _elements.Add((element, elementBounds));
                    return true;
                }
                Split(maxElementsPerNode, maxDepth);
            }

            return TryInsertToNonLeaf(element, elementBounds, maxElementsPerNode, maxDepth);
        }

        public IFiniteElement<TSpace>? FindElementAt(TSpace point)
        {
            if (!Bounds.Contains(point)) return null;

            // 1. Check parent
            foreach ((var element, _) in _elements)
                if (element.Geometry.ContainsPoint(point))
                    return element;
            if (IsLeaf) return null;

            // 2. Check children
            int childIndex = GetChildIndex(point);
            var result = _children![childIndex].FindElementAt(point);
            if (result != null) return result;

            // 3. Rare edge-case fallback (point lies on tree boundary)
            foreach (var sibling in _children)
            {
                if (sibling == _children![childIndex]) continue;
                result = sibling.FindElementAt(point);
                if (result != null) return result;
            }
            return null;
        }

        public IEnumerable<IFiniteElement<TSpace>> QueryIntersection(BoundingBox<TSpace> queryBounds)
        {
            if (!Bounds.Intersects(queryBounds))
                yield break;

            foreach (var (element, elementBounds) in _elements)
                if (elementBounds.Intersects(queryBounds))
                    yield return element;

            if (IsLeaf) yield break;

            foreach (var child in _children!)
                foreach (var element in child.QueryIntersection(queryBounds))
                    yield return element;
        }

        // Splits node into sub nodes and redistribute elements between children
        private void Split(int maxElementsPerNode, int maxDepth)
        {
            Debug.Assert(IsLeaf);

            var center = Bounds.Center;
            int childCount = 1 << TSpace.Dimensions; // 2^Dimensions (quadtree for 2d, octree for 3d)
            _children = new SpatialTreeNode[childCount];

            for (int i = 0; i < childCount; i++)
            {
                var childBounds = CreateChildBounds(center, i);
                _children[i] = new SpatialTreeNode(childBounds, Depth + 1);
            }

            // Redistribute existing elements
            for (int i  = _elements.Count - 1; i >= 0; --i)
            {
                (var element, var elementBounds) = _elements[i];
                int index = GetChildIndexContaining(elementBounds);
                if (index < 0) continue; // Element lies on children intersection, keep
                if (_children[index].TryInsert(element, elementBounds, maxElementsPerNode, maxDepth))
                {
                    _elements[i] = _elements[^1];
                    _elements.RemoveAt(_elements.Count - 1);
                }
            }
        }

        private BoundingBox<TSpace> CreateChildBounds(TSpace center, int childIndex)
        {
            (var min, var max) = Bounds;

            var newMin = TSpace.Zero;
            var newMax = TSpace.Zero;

            for (int dim = 0; dim < TSpace.Dimensions; dim++)
            {
                bool useMaxHalf = ((childIndex >> dim) & 1) == 1;

                double minVal = useMaxHalf ? center[dim] : min[dim];
                double maxVal = useMaxHalf ? max[dim] : center[dim];

                newMin = newMin.WithComponent(dim, minVal);
                newMax = newMax.WithComponent(dim, maxVal);
            }

            return new(newMin, newMax);
        }

        private bool TryInsertToNonLeaf(
            IFiniteElement<TSpace> element,
            BoundingBox<TSpace> elementBounds,
            int maxElementsPerNode,
            int maxDepth
        )
        {
            Debug.Assert(!IsLeaf);
            int index = GetChildIndexContaining(elementBounds);

            if (index >= 0)
                return _children![index].TryInsert(element, elementBounds, maxElementsPerNode, maxDepth); 
            _elements.Add((element, elementBounds)); // Element lies on children intersection
            return true;
        }

        private int GetChildIndexContaining(BoundingBox<TSpace> elementBounds)
        {
            Debug.Assert(!IsLeaf);
            for (int i = 0; i < _children!.Length; ++i)
                if (_children![i].Bounds.Contains(elementBounds))
                    return i;
            return -1;
        }

        private int GetChildIndex(TSpace point)
        {
            var center = Bounds.Center;
            int index = 0;

            for (int dim = 0; dim < TSpace.Dimensions; dim++)
            {
                double coord = point[dim];
                double centerCoord = center[dim];

                if (coord >= centerCoord)
                    index |= (1 << dim);
            }

            return index;
        }
    }
}
