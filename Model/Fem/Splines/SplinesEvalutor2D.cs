using System;
using System.Collections.Generic;
using System.ComponentModel.DataAnnotations;
using System.Net.WebSockets;
using System.Text;
using Model.Fem.Elements;
using Telma;

namespace Model.Fem.Splines;

/// <summary>
/// Значение сплайна в локальной точке элемента и соответствующей точке сетке.
/// ξ, η координаты шаблонного квадрата [0; 1] x [0; 1].
/// X, Y — координаты в сетке.
/// Value — значение u^h в этой точке.
/// </summary>
public readonly record struct SplineSample2D
(
    double ξ,
    double η,
    double X,
    double Y,
    double Value
);

/// <summary>
/// Вычисление значений сплайна в локальных точках элемента и соответствующих точках сетки
/// с помощью эрмитовых элементов на четырехугольных конечных элементах.
/// </summary>
public sealed class HermiteSplineEvaluator2D
{
    /// <summary>
    /// Вычисляем значение u^h(ξ, η) внутри элемента
    /// </summary>
    /// <param name="ξ">Координата ξ в локальной системе элемента</param>
    /// <param name="η">Координата η в локальной системе элемента</param>
    /// <param name="solution">Массив значений решения для всех степеней свободы</param>
    /// <param name="element">Элемент конечного элемента, в котором вычисляется значение</param>
    /// <returns>Значение u^h(ξ, η) в указанной точке</returns>
    /// <exception cref="InvalidOperationException"></exception>
    /// <exception cref="ArgumentOutOfRangeException"></exception>
    public double Evaluate(double ξ, double η, ReadOnlySpan<double> solution, IFiniteElement<Vector2D> element)
    {
        ValidateLocalPoint(ξ, η);

        var localPoint = new Vector2D(ξ, η);
        var basis = element.BasisSet.Basis;
        var dof = element.DOF.Dof;

        if(basis.Length != dof.Length)
        {
            throw new InvalidOperationException($"Количество базисных функций ({basis.Length}) не совпадает с количеством степеней свободы ({dof.Length}).");
        }

        double value = 0.0;

        for (int localDof = 0; localDof < dof.Length; localDof++)
        {
            int globalDof = dof[localDof];

            if(globalDof < 0 || globalDof >= solution.Length)
            {
                throw new ArgumentOutOfRangeException(nameof(solution), $"Глобальный индекс степени свободы ({globalDof}) выходит за пределы массива решения (длина {solution.Length}).");
            }

            double q = solution[globalDof];
            double ψ = basis[localDof].Value(localPoint);

            value += q * ψ;
        }
        return value;
    }

    /// <summary>
    /// Вычисляет сплайновое решение по указанным локальным координатам в пределах одного конечного элемента
    /// </summary>
    /// <remarks>
    /// Возвращаемая SplineSample2D содержит как локальные (ξ, η), так и глобальные (X, Y) координаты, а также
    /// вычисленное значение решения в этой точке. Этот метод потмо будет использоан в обработке и визуализации решений.
    /// </remarks>
    /// <param name="ξ">Координата ξ в локальной системе элемента</param>
    /// <param name="η">Координата η в локальной системе элемента</param>
    /// <param name="solution">Массив значений решения для всех степеней свободы</param>
    /// <param name="element">Элемент конечного элемента, в котором вычисляется значение</param>
    /// <returns>
    /// Экземпляр SplineSample2D, содержащий локальные координаты, соответствующие глобальные координаты и вычисленное значение
    /// solution в указанной точке.
    /// </returns>
    public SplineSample2D EvaluateSample(double ξ, double η, ReadOnlySpan<double> solution, IFiniteElement<Vector2D> element)
    {
        ValidateLocalPoint(ξ, η); // он вызывается еще Evaluate, но пусть будет, от ошибок в globalPoint.
        var localPoint = new Vector2D(ξ, η);
        var globalPoint = element.Geometry.MasterElementCoordinateSystem.InverseTransform(localPoint);
        double value = Evaluate(ξ, η, solution, element);

        return new SplineSample2D(
            ξ: ξ,
            η: η,
            X: globalPoint.X,
            Y: globalPoint.Y,
            Value: value
        );
            
    }
    
    /// <summary>
    /// Вычисляет значения двумерной сплайн-функции на равномерной сетке внутри конечного элемента.
    /// </summary>
    /// <remarks>
    /// в будущем пир отрисовке будет (вероятно) рисоватья градиент(цветом) и для верного перехода можно разбивать КЭ
    /// на более мелкие части, а не рисовать по узлам КЭ. 
    /// </remarks>
    /// <param name="solution">Вектор коэффициентов решения, используемый для вычисления значений сплайна.</param>
    /// <param name="element">Конечный элемент, в котором производится вычисление значений сплайна.</param>
    /// <param name="subdivision">Количество разбиений по каждой координате для построения сетки. Должно быть положительным целым числом.</param>
    /// <returns>
    /// Список образцов сплайна, вычисленных в узлах равномерной сетки внутри элемента.
    /// Размер списка равен (subdivision + 1) × (subdivision + 1).
    /// </returns>
    /// <exception cref="ArgumentOutOfRangeException">Выбрасывается, если значение параметра subdivision меньше или равно нулю.</exception>
    public List<SplineSample2D> EvaluateGrid(ReadOnlySpan<double> solution, IFiniteElement<Vector2D> element, int subdivision)
    {
        if (subdivision <= 0)
        {
            throw new ArgumentOutOfRangeException(nameof(subdivision), "Деление должно быть положительным целым числом.");
        }

        var samples = new List<SplineSample2D>((subdivision + 1) * (subdivision + 1));

        for(int i = 0; i <= subdivision; i++)
        {
            double η = (double)i / subdivision;
            for(int j = 0; j <= subdivision; j++)
            {
                double ξ = (double)j / subdivision;
                var sample = EvaluateSample(ξ, η, solution, element);
                samples.Add(sample);
            }
        }
        return samples;
    }

    /// <summary>
    /// Валидация локальных координат (ξ, η) для шаблонного квадрата [0; 1] x [0; 1].
    /// </summary>
    /// <param name="ξ">Локальная координата ξ в пределах элемента.</param>
    /// <param name="η">Локальная координата η в пределах элемента.</param>
    /// <exception cref="ArgumentOutOfRangeException">Выбрасывается, если локальная точка находится за пределами [0; 1] x [0; 1].</exception>
    private static void ValidateLocalPoint(double ξ, double η)
    {
        const double eps = 1e-12;

        if (double.IsNaN(ξ) || double.IsNaN(η) ||
            ξ < -eps || ξ > 1.0 + eps ||
            η < -eps || η > 1.0 + eps)
        {
            throw new ArgumentOutOfRangeException(
                $"Локальная точка ({ξ}, {η}) должна принадлежать [0; 1] x [0; 1]." 
            );
        }
    }
}
