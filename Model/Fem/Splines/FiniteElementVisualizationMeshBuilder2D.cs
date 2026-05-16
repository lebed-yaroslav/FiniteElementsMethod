using Model.Fem.Elements;
using Model.Fem.Mesh;
using Telma;

namespace Model.Fem.Splines;

/// <summary>
/// "Разбиватель четырехугольников 3000"
/// v01 ----- v11
///  | \     / |
///  |  \   /  |
///  |    S    |
///  |  /   \  |
///  | /     \ |
/// v00 ----- v10
/// 
/// треугольники 
/// (v00, v10, S)
/// (v10, v11, S)
/// (v11, v01, S)
/// (v01, v00, S)
/// 
/// строит треугольную сетку для визуализации сплайна
/// </summary> 
public sealed class FiniteElementVisualizationMeshBuilder2D
{
    private readonly FiniteElementFieldEvaluator2D _evaluator;

    /// <summary>
    /// builder со стандартным вычислителем сплайна
    /// </summary>
    public FiniteElementVisualizationMeshBuilder2D() : this(new FiniteElementFieldEvaluator2D())
    {
    }

    /// <summary>
    /// builder с переданным вычислителем сплайна
    /// </summary>
    /// <param name="evaluator"> Объект, который считает значение поля в локальной точке конечного элемента. </param>
    public FiniteElementVisualizationMeshBuilder2D(FiniteElementFieldEvaluator2D evaluator)
    {
        _evaluator = evaluator ?? throw new ArgumentNullException(nameof(evaluator));
    }

    /// <summary>
    /// строит визуализационную треугольную сетку по исходной КЭ сетке
    /// </summary>
    /// <param name="mesh">сетка </param>
    /// <param name="solution"> Вектор решения, полученный после решения СЛАУ </param>
    /// <param name="subdivision">количество разбиений каждого конечного элемента по локальным координатам </param>
    /// <returns>Визуализационная сетка, содержащая вершины и треугольники.</returns>
    public VisualizationMesh2D BuildMesh(IMesh<Vector2D> mesh, ReadOnlySpan<double> solution, int subdivision)
    {
        if (mesh is null)
        {
            throw new ArgumentNullException(nameof(mesh));
        }

        return BuildMesh(mesh.FiniteElements, solution, subdivision);
    }
    /// <summary> Строит визуализационную треугольную сетку по набору КЭ</summary>
    /// <param name="elements">Набор КЭ </param>
    /// <param name="solution"> Вектор решения, полученный после решения СЛАУ </param>
    /// <param name="subdivision">количество разбиений каждого конечного элемента по локальным координатам </param>
    /// <returns>Визуализационная сетка, содержащая вершины и треугольники.</returns>
    public VisualizationMesh2D BuildMesh(IEnumerable<IFiniteElement<Vector2D>> elements, ReadOnlySpan<double> solution, int subdivision)
    {
        if (elements is null)
        {
            throw new ArgumentNullException(nameof(elements));
        }

        if (subdivision <= 0)
        {
            throw new ArgumentOutOfRangeException(nameof(subdivision), "Деление должно быть положительным целым числом.");
        }

        var result = new VisualizationMesh2D
        {
            Subdivision = subdivision,
        };
        int sourceElementId = 0;

        foreach (var element in elements)
        {
            AddElement(result, sourceElementId, element, solution, subdivision);
            sourceElementId++;
        }

        return result;
    }

    /// <summary>
    /// Добавляет в визуализационную сетку вершины и треугольники одного конечного элемента.
    /// </summary>
    /// <param name="mesh"> Визуализационная сетка, в которую добавляются новые данные </param>
    /// <param name="sourceElementId"> Номер исходного конечного элемента. </param>
    /// <param name="element">Конечный элемент, который разбивается на треугольники.</param>
    /// <param name="solution"> Вектор решения, по которому вычисляются значения поля. </param>
    /// <param name="subdivision"> Количество разбиений элемента по локальным координатам.</param>
    private void AddElement(VisualizationMesh2D mesh, int sourceElementId, IFiniteElement<Vector2D> element, ReadOnlySpan<double> solution, int subdivision)
    {
        int[,] grid = new int[subdivision + 1, subdivision + 1];

        List<SplineSample2D> samples = _evaluator.EvaluateGrid(solution, element, subdivision);

        for (int j = 0; j <= subdivision; j++)
        {
            for (int i = 0; i <= subdivision; i++)
            {
                int sampleIndex = j * (subdivision + 1) + i;
                SplineSample2D sample = samples[sampleIndex];

                grid[j, i] = AddVertex(mesh, sourceElementId, sample);
            }
        }
        for (int j = 0; j < subdivision; j++)
        {
            for (int i = 0; i < subdivision; i++)
            {
                int v00 = grid[j, i];
                int v10 = grid[j, i + 1];
                int v01 = grid[j + 1, i];
                int v11 = grid[j + 1, i + 1];

                // wtynhs rf;ljuj xtnshe[eujkmybrf, или же центры каждого четырехугольника
                double ξ_S = (i + 0.5) / subdivision;
                double η_S = (j + 0.5) / subdivision;

                SplineSample2D centerSample = _evaluator.EvaluateSample(ξ_S, η_S, solution, element);
                int center = AddVertex(mesh, sourceElementId, centerSample);

                AddTriangle(mesh, sourceElementId, v00, v10, center);
                AddTriangle(mesh, sourceElementId, v10, v11, center);
                AddTriangle(mesh, sourceElementId, v11, v01, center);
                AddTriangle(mesh, sourceElementId, v01, v00, center);
            }
        }
    }
    /// <summary>Добавляет одну вершину в сетку. </summary>
    /// <param name="mesh"> Визуализационная сетка</param>
    /// <param name="sourceElementId">Номер исходного конечного элемента.</param>
    /// <param name="sample"> Посчитанная точка сплайна: координаты и значение поля.</param>
    /// <returns>Индекс добавленной вершины. </returns>
    private static int AddVertex(VisualizationMesh2D mesh, int sourceElementId, SplineSample2D sample)
    {
        int id = mesh.Vertices.Count;

        mesh.Vertices.Add(new VisualizationVertex2D(
            Id: id,
            SourceElementId: sourceElementId,
            X: sample.X,
            Y: sample.Y,
            Value: sample.Value)
        );

        return id;
    }
    /// <summary>
    /// Добавляет треугольник в сетку, используя индексы вершин.
    /// </summary>
    /// <param name="mesh"> Визуализационная сетка</param>
    /// <param name="sourceElementId">Номер исходного конечного элемента.</param>
    /// <param name="v0">Индекс первой вершины треугольника.</param>
    /// <param name="v1">Индекс второй вершины треугольника.</param>
    /// <param name="v2">Индекс третьей вершины треугольника.</param>
    private static void AddTriangle(VisualizationMesh2D mesh, int sourceElementId, int v0, int v1, int v2)
    {
        int id = mesh.Triangles.Count;

        mesh.Triangles.Add(new VisualizationTriangle2D(
            Id: id,
            SourceElementId: sourceElementId,
            V0: v0,
            V1: v1,
            V2: v2)
        );
    }
}
