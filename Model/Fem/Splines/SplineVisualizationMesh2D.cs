namespace Model.Fem.Splines;

/// <summary>
/// Вершина сетки, которая будет использоваться для визуализации.
/// </summary>
/// <param name="Id">  Номер вершины в списке вершин.</param>
/// <param name="SourceElementId"> Номер исходного конечного элемента, из которого получена эта вершина.</param>
/// <param name="X"> Координата X вершины.</param>
/// <param name="Y"> Координата Y вершины.</param>
/// <param name="Value"> значение поля u^h в этой точке.</param>
public readonly record struct VisualizationVertex2D
(
    int Id,
    int SourceElementId,
    double X,
    double Y,
    double Value
);

/// <summary>
/// Треугольник для отрисовки.
/// </summary>
/// <param name="Id">Номер треугольника в списке треугольников.</param>
/// <param name="SourceElementId">Номер исходного конечного элемента, из которого получен этот треугольник.</param>
/// <param name="V0">Индекс первой вершины треугольника.</param>
/// <param name="V1">Индекс второй вершины треугольника.</param>
/// <param name="V2">Индекс третьей вершины треугольника.</param>
public readonly record struct VisualizationTriangle2D(
    int Id,
    int SourceElementId,
    int V0,
    int V1,
    int V2
);

/// <summary>
/// Сетка, подготовленная для визуализации поля.
/// </summary>
public sealed class VisualizationMesh2D
{
    /// <summary>
    /// Количество разбиений одного конечного элемента по каждой координате.
    /// </summary>
    public int Subdivision {  get; init; }
    /// <summary>
    /// Список вершин визуализационной сетки.
    /// </summary>
    public List<VisualizationVertex2D> Vertices { get; } = new();
    /// <summary>
    /// Список треугольников визуализационной сетки.
    /// </summary>
    public List<VisualizationTriangle2D> Triangles { get; } = new();
}
