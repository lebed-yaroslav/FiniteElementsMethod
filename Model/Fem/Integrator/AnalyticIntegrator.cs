using Model.Core.CoordinateSystem;
using Model.Core.Matrix;
using Model.Fem.Basis;
using Model.Fem.Elements;
using Model.Fem.Elements.Segment;
using Model.Fem.Elements.Triangle;
using Telma;
using Telma.Extensions;

namespace Model.Fem.Integrator;


public interface IAnalyticalIntegrator
{
    double IntegrateTriangle(IPolynomial2D poly);
    double IntegrateQuadrangle(IPolynomial2D poly);
    double IntegrateBoundary(IPolynomial1D poly);
}

public static class AnalyticalIntegration
{
    static readonly int D = 3 * 2 + 2; // max poly degree after integration
    static readonly int[,] C = new int[D, D]; // matrix of coefficients

    static AnalyticalIntegration()
    {
        for (int i = 0; i < D; i++)
        {
            C[i, 0] = 1;
            C[i, i] = 1;

            for (int j = 1; j < i; j++)
            {
                C[i, j] = C[i - 1, j - 1] + C[i - 1, j];
            }
        }
    }

    public static double IntegrateTriangle(IPolynomial2D poly)
    {
        double res = 0.0;

        foreach (var i in poly.Summands)
        {
            double tmp = 0.0;
            for (int k = 0; k <= i.Key.q + 1; k++)
            {
                tmp += Math.Pow(-1.0, k) * C[i.Key.q + 1, k] * 1.0 / ((i.Key.q + 1) * (k + i.Key.p + 1));
            }

            res += i.Value * tmp;
            tmp = 0.0;
        }

        return res;
    }

    public static double IntegrateQuadrangle(IPolynomial2D poly)
    {
        double res = 0.0;

        foreach (var i in poly.Summands)
        {
            res += i.Value / ((i.Key.q + 1) * (i.Key.p + 1));
        }

        return res;
    }

    public static double IntegrateBoundary(IPolynomial1D poly)
    {
        double res = 0.0;

        foreach (var i in poly.Summands)
        {
            res += i.Value / (i.Key + 1);
        }

        return res;
    }
}

public class AnalyticIntegrator<TSpace, TBoundary, TOps> : IIntegrator<TSpace, TBoundary, TOps>
    where TSpace : struct, IVectorBase<TSpace>
    where TBoundary : IVectorBase<TBoundary>
    where TOps : IMatrixOperations<TSpace, TSpace, TOps>
{
    public static readonly AnalyticIntegrator<TSpace, TBoundary, TOps> Instance = new();
    private AnalyticIntegrator() { }

    public LocalMatrix CalculateLocalStiffness(
       IFiniteElementBase<TSpace, TSpace> element,
       Func<TSpace, double> lambda
    )
    {
        AssertCartesianMesh(element);
        AssertSupportedTopology(element);

        int n = element.DOF.Count;
        var stiffness = new LocalMatrix(n);

        var masterCs = element.MasterElementCoordinateSystem;
        var meshCs = element.Mesh.CoordinateSystem;

        // Якобиан должен быть константой, так что берём значение в нуле
        var masterInvJ = masterCs.InverseJacoby();
        var meshInvJ = meshCs.InverseJacoby();

        var invJ = masterInvJ.MulAt<TSpace, TSpace, TSpace, TOps>(TSpace.Zero, meshInvJ, TSpace.Zero);
        // M = J ^ (-T) * J ^ (-1)
        double m00 = invJ[0, 0] * invJ[0, 0] + invJ[0, 1] * invJ[0, 1];
        double m01 = invJ[0, 0] * invJ[1, 0] + invJ[0, 1] * invJ[1, 1];
        double m11 = invJ[1, 0] * invJ[1, 0] + invJ[1, 1] * invJ[1, 1];

        var jacobian = Math.Abs(1.0 / invJ.Det());
        var mp = masterCs.InverseTransform(TSpace.Zero);
        var C = lambda(mp) * jacobian;

        var grads = new (IPolynomial2D gx, IPolynomial2D gy)[n];
        for (int i = 0; i < n; i++)
        {
            var grad = AsPoly2D(element.Basis[i]).Gradient();
            grads[i] = (grad[0], grad[1]);
        }

        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j <= i; j++)
            {
                // grad(phi_i)^T * J^(-T) * J^(-1) * grad(phi_j)
                var integrand = new Polynomial2D();
                Polynomial2D.AddScaled(integrand, Polynomial2D.Mult(grads[i].gx, grads[j].gx), m00);
                Polynomial2D.AddScaled(integrand, Polynomial2D.Mult(grads[i].gx, grads[j].gy), m01);
                Polynomial2D.AddScaled(integrand, Polynomial2D.Mult(grads[i].gy, grads[j].gx), m01);
                Polynomial2D.AddScaled(integrand, Polynomial2D.Mult(grads[i].gy, grads[j].gy), m11);

                double value = C * AnalyticalIntegration.IntegrateTriangle(integrand);
                stiffness[i, j] = value;
                stiffness[j, i] = value;
            }
        }

        return stiffness;
    }

    public LocalMatrix CalculateLocalMass(
        IFiniteElementBase<TSpace, TBoundary> element, 
        Func<TSpace, double> gamma
    ) => CalculateLocalMass<TBoundary>(element, gamma);

    public LocalMatrix CalculateLocalMass(
        IFiniteElementBase<TSpace, TSpace> element, 
        Func<TSpace, double> gamma
       ) => CalculateLocalMass<TSpace>(element, gamma);

    private static LocalMatrix CalculateLocalMass<TB>(
        IFiniteElementBase<TSpace, TB> element, 
        Func<TSpace, double> gamma)
        where TB : IVectorBase<TB>
    {
        AssertCartesianMesh(element);
        AssertSupportedTopology(element);

        int n = element.DOF.Count;
        var mass = new LocalMatrix(n);

        var masterCs = element.MasterElementCoordinateSystem;
        var meshCs = element.Mesh.CoordinateSystem;

        // Якобиан должен быть константой, так что берём значение в нуле
        var zeroPoint = default(TB);
        var mp = masterCs.InverseTransform(zeroPoint);
        var jacobian = Math.Abs(masterCs.Jacobian(zeroPoint) * meshCs.Jacobian(mp));

        var C = gamma(mp) * jacobian;

        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j <= i; j++)
            {
                double integration_res = 0.0;

                // Интегрируем либо по треугольнику, либо по сегменту
                if (typeof(TB) == typeof(TSpace))
                {
                    var integrand = Polynomial2D.Mult(AsPoly2D(element.Basis[i]), AsPoly2D(element.Basis[j]));
                    integration_res = AnalyticalIntegration.IntegrateTriangle(integrand);
                }
                else if (typeof(TB) == typeof(TBoundary))
                {
                    var integrand = Polynomial1D.Mult(AsPoly1D(element.Basis[i]), AsPoly1D(element.Basis[j]));
                    integration_res = AnalyticalIntegration.IntegrateBoundary(integrand);
                }

                double value = C * integration_res;
                mass[i, j] = value;
                mass[j, i] = value;
            }
        }

        return mass;
    }

    public void CalculateLocalLoad(
        IFiniteElementBase<TSpace, TBoundary> element,
        Func<TSpace, double> source,
        Span<double> outLoad
    ) => NumericIntegrator<TSpace, TBoundary, TOps>.Instance.CalculateLocalLoad(element, source, outLoad);

    public void CalculateLocalLoad(
        IFiniteElementBase<TSpace, TSpace> element,
        Func<TSpace, double> source,
        Span<double> outLoad
    ) => NumericIntegrator<TSpace, TBoundary, TOps>.Instance.CalculateLocalLoad(element, source, outLoad);

    /// <summary> Проверяет, что пришла декартова система координат </summary>
    private static void AssertCartesianMesh<TB>(IFiniteElementBase<TSpace, TB> element) where TB : IVectorBase<TB>
    {
        if (element.Mesh.CoordinateSystem is not IdentityTransform<TSpace>)
            throw new NotSupportedException($"AnalyticIntegrator supports only Cartesian mesh coordinate system.");
    }

    /// <summary> Проверяет, что элементы являются треугольниками или сегментами </summary>
    private static void AssertSupportedTopology<TB>(IFiniteElementBase<TSpace, TB> element)
    where TB : IVectorBase<TB>
    {
        if (typeof(TB) == typeof(TSpace))
        {
            if (element.Geometry is not TriangleGeometry)
                throw new NotSupportedException($"AnalyticIntegrator supports only triangle elements, got {element.Geometry.GetType().Name}");
        }

        else if (typeof(TB) == typeof(TBoundary))
        {
            if (element.Geometry is not SegmentGeometry<TSpace>)
                throw new NotSupportedException($"AnalyticIntegrator supports only segment boundary elements, got {element.Geometry.GetType().Name}");
        }
    }

    /// <summary> Проверяет, что пришел 2D полином </summary>
    private static IPolynomial2D AsPoly2D(object b)
    {
        if (b is IPolynomial2D p) return p;
        if (b is OrientedPolynomialBasisFunction2D oriented_p) return (IPolynomial2D)oriented_p.Poly;
        throw new NotSupportedException($"AnalyticIntegrator requires IPolynomial2D basis functions, got {b.GetType().Name}");
    }
    /// <summary> Проверяет, что пришел 1D полином </summary>
    private static IPolynomial1D AsPoly1D(object b)
    {
        if (b is IPolynomial1D p) return p;
        if (b is OrientedPolynomialBasisFunction1D oriented_p) return (IPolynomial1D)oriented_p.Poly;
        throw new NotSupportedException($"AnalyticIntegrator requires IPolynomial1D basis functions, got {b.GetType().Name}");
    }
}
