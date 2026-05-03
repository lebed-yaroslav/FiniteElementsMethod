using Model.Core.CoordinateSystem;
using Model.Core.Matrix;
using Model.Fem.Basis;
using Model.Fem.Elements;
using Model.Fem.Elements.Quadrangle;
using Model.Fem.Elements.Segment;
using Model.Fem.Elements.Triangle;
using Telma;
using Telma.Extensions;
using static Model.Core.CoordinateSystem.MatrixOperations;
namespace Model.Fem.Integrator;

public interface IPolynomial : IBasisFunction2D
{
    int Degree { get; }
    List<(int p, int q, double coeff)> Summands { get; set; } //p - х degree, q - у degree
    void Add(IPolynomial poly); //add to the polynomial
    void Mult(IPolynomial poly); //multiply the polynomial
}

public class Polynomial : IPolynomial
{
    public List<(int p, int q, double coeff)> Summands { get; set; } = [];
    public int Degree
    {
        get
        {
            int d = Summands.Max(i => i.p + i.q);
            return d;
        }
    }

    public static IPolynomial Sum(IPolynomial poly1, IPolynomial poly2)
    {
        var res = new Dictionary<(int p, int q), double>();
        foreach (var i in poly1.Summands)
        {
            if (!res.TryAdd((i.p, i.q), i.coeff)) res[(i.p, i.q)] += i.coeff;
        }
        foreach (var i in poly2.Summands)
        {
            if (!res.TryAdd((i.p, i.q), i.coeff)) res[(i.p, i.q)] += i.coeff;
        }


        var res_poly = new Polynomial();
        foreach (var i in res)
        {
            res_poly.Summands.Add((i.Key.p, i.Key.q, i.Value));
        }

        return res_poly;
    }
    public void Add(IPolynomial poly)
    {
        var res = new Dictionary<(int p, int q), double>();
        foreach (var i in Summands)
        {
            if (!res.TryAdd((i.p, i.q), i.coeff)) res[(i.p, i.q)] += i.coeff;
        }
        foreach (var i in poly.Summands)
        {
            if (!res.TryAdd((i.p, i.q), i.coeff)) res[(i.p, i.q)] += i.coeff;
        }

        Summands.Clear();

        foreach (var i in res)
        {
            Summands.Add((i.Key.p, i.Key.q, i.Value));
        }
    }

    public static IPolynomial Mult(IPolynomial poly1, IPolynomial poly2)
    {
        var res = new Dictionary<(int p, int q), double>();
        foreach (var i in poly1.Summands)
        {
            foreach (var j in poly2.Summands)
            {
                if (!res.TryAdd((i.p + j.p, i.q + j.q), i.coeff * j.coeff)) res[(i.p + j.p, i.q + j.q)] += i.coeff * j.coeff;
            }
        }

        var res_poly = new Polynomial();
        foreach (var i in res)
        {
            res_poly.Summands.Add((i.Key.p, i.Key.q, i.Value));
        }

        return res_poly;
    }
    public void Mult(IPolynomial poly)
    {
        var res = new Dictionary<(int p, int q), double>();
        foreach (var i in Summands)
        {
            foreach (var j in poly.Summands)
            {
                if (!res.TryAdd((i.p + j.p, i.q + j.q), i.coeff * j.coeff)) res[(i.p + j.p, i.q + j.q)] += i.coeff * j.coeff;
            }
        }

        Summands.Clear();

        foreach (var i in res)
        {
            Summands.Add((i.Key.p, i.Key.q, i.Value));
        }
    }

    public double Value(Vector2D point)
    {
        double res = 0.0;
        foreach (var i in Summands)
        {
            res += i.coeff * Math.Pow(point.X, i.p) * Math.Pow(point.Y, i.q);
        }
        return res;
    }

    public Vector2D Derivatives(Vector2D point)
    {
        var grad = Gradient();
        var res = new Vector2D(grad[0].Value(point), grad[1].Value(point));

        return res;
    }

    public ReadOnlySpan<IBasisFunction2D> Gradient()
    {
        var grad = new IPolynomial[2];
        foreach (var i in Summands)
        {
            grad[0].Summands.Add((i.p - 1, i.q, i.coeff * i.p));
            grad[1].Summands.Add((i.p, i.q - 1, i.coeff * i.q));
        }

        return grad;
    }
}

[Obsolete("This feature is under development.")]
public class AnalyticalIntegrator<TSpace, TBoundary, TOps> : IIntegrator<TSpace, TBoundary, TOps>
    where TSpace : IVectorBase<TSpace>
    where TBoundary : IVectorBase<TBoundary>
    where TOps : IMatrixOperations<TSpace, TSpace, TOps>
{
    static readonly int d = 3 * 2 + 2; // max poly degree after integration
    static readonly int[,] C = new int[d, d];

    static AnalyticalIntegrator()
    {
        for (int i = 0; i < d; i++)
        {
            C[i, 0] = 1;
            C[i, i] = 1;

            for (int j = 1; j < i; j++)
            {
                C[i, j] = C[i - 1, j - 1] + C[i - 1, j];
            }
        }
    }
  
    public static double IntegrateTriangle(IPolynomial poly, IPolynomial jacobian)
    {
        double res = 0.0;
        var polyJ = Polynomial.Mult(jacobian, poly);

        foreach (var i in polyJ.Summands)
        {
            for (int k = 0; k <= i.q + 1; k++)
            {
                res += 1.0 / ((i.q + 1) * (k + i.p + 1)) * C[i.q + 1, k];
            }

            res *= i.coeff;
        }

        return res;
    }

    public static double IntegrateQuadrangle(IPolynomial poly, IPolynomial jacobian)
    {
        double res = 0.0;
        var polyJ = Polynomial.Mult(jacobian, poly);

        foreach (var i in polyJ.Summands)
        {
            for (int k = 0; k <= i.q + 1; k++)
            {
                res += 1.0 / ((i.q + 1) * (i.p + 1));
            }

            res *= i.coeff;
        }

        return res;
    }
    public static double IntegrateSegment(IPolynomial poly, IPolynomial jacobian)
    {
        double res = 0.0;
        var polyJ = Polynomial.Mult(jacobian, poly);
        foreach (var i in polyJ.Summands)
        {
            res += i.coeff/(i.p + 1);
        }
        return res;
    }
    [Obsolete("This method is under development and may work incorrectly")]
    private Polynomial InterpolateFunc<TB>(IFiniteElementBase<TSpace, TB> element, Func<TSpace, double> func)
        where TB : IVectorBase<TB>
    {

        int n = element.DOF.Count;
        var resPolynomial = new Polynomial(); 
        var basisFuncs = element.Basis;
        var analyticalBasis = new IPolynomial[n];
        for (int i = 0; i < n; i++)
        {
            if (basisFuncs[i] is IAnalyticalBasisFunction<TSpace> analyticalFunc)
            {
                analyticalBasis[i] = analyticalFunc.Polynomial;
            }
            else
            {
                throw new InvalidCastException($"Basis {basisFuncs[i].GetType().Name} is not an analytical basis function.");
            }
        }
        // Moving through all nodes functions
        // For each node we calculate func value at the coordinates of this node and multiply by corresponding basis polynomial
        for (int k = 0; k < n; k++)
        { 
            var mp = element.Geometry.Mesh[k]; 
            double funcValue = func(mp);
            var basisPolynomial = analyticalBasis[k];
            var termPolynomial = Polynomial.Mult(new Polynomial { Summands = [(0, 0, funcValue)] }, basisPolynomial);
            resPolynomial.Add(termPolynomial); 
        }
        return resPolynomial;
    }

    public LocalMatrix CalculateLocalStiffness(
    IFiniteElementBase<TSpace, TSpace> element,
    Func<TSpace, double> lambda)
    {
        var basis = element.Basis;
        int n = basis.Length;
        var stiffness = new LocalMatrix(n);
        var masterCs = element.MasterElementCoordinateSystem;
        var meshCs = element.Mesh.CoordinateSystem;

        var epZero = TSpace.Zero;
        var masterInvJ = masterCs.InverseJacoby();
        var meshInvJ = meshCs.InverseJacoby();

        var invJ = masterInvJ.MulAt<TSpace,TSpace,TSpace,TOps>(epZero, meshInvJ, meshCs.InverseTransform(epZero));

        double detJ = Math.Abs(1.0 / invJ.Det());
        var jacobian = new Polynomial { Summands = [(0, 0, detJ)] };

        var lambdaPolynomial = InterpolateFunc(element, lambda);
        var analyticalBasis = new Polynomial[n];
    
        for (int i = 0; i < n; i++)
            analyticalBasis[i] = (Polynomial)((IAnalyticalBasisFunction<TSpace>)basis[i]).Polynomial;

        for (int i = 0; i < n; i++)
        {
            // Local gradients 
            var gradI_xi = analyticalBasis[i].Gradient();
            // Transform to physical gradients (multiplication by the transposed invJ)
            // grad_x = invJ[0,0] * d/dxi + invJ[1,0] * d/deta
            // grad_y = invJ[0,1] * d/dxi + invJ[1,1] * d/deta
            var gradI_x = Polynomial.Sum(MultScalar((IPolynomial)gradI_xi[0], invJ[0, 0]), MultScalar((IPolynomial)gradI_xi[1], invJ[1, 0]));
            var gradI_y = Polynomial.Sum(MultScalar((IPolynomial)gradI_xi[0], invJ[0, 1]), MultScalar((IPolynomial)gradI_xi[1], invJ[1, 1]));

            for (int j = 0; j <= i; j++)
            {
                var gradJ_xi = analyticalBasis[j].Gradient();

                var gradJ_x = Polynomial.Sum(MultScalar((IPolynomial)gradJ_xi[0], invJ[0, 0]), MultScalar((IPolynomial)gradJ_xi[1], invJ[1, 0]));
                var gradJ_y = Polynomial.Sum(MultScalar((IPolynomial)gradJ_xi[0], invJ[0, 1]), MultScalar((IPolynomial)gradJ_xi[1], invJ[1, 1]));
                // Scalar product of gradients: (gradI_x * gradJ_x) + (gradI_y * gradJ_y)
                var dotProduct = Polynomial.Sum(
                    Polynomial.Mult(gradI_x, gradJ_x),
                    Polynomial.Mult(gradI_y, gradJ_y)
                );

                var integrand = Polynomial.Mult(lambdaPolynomial, dotProduct);

                double value = 0;
                if (element.Geometry is TriangleGeometry)
                {
                    value = IntegrateTriangle(integrand, jacobian);
                }
                else if (element.Geometry is QuadrangleGeometry)
                {
                    value = IntegrateQuadrangle(integrand, jacobian);
                }

                stiffness[i, j] = value;
                stiffness[j, i] = value;
            }
        }
        return stiffness;
    }
    private IPolynomial MultScalar(IPolynomial poly, double scalar)
    {
        return Polynomial.Mult(poly, new Polynomial { Summands = [(0, 0, scalar)] });
    }
 

  public LocalMatrix CalculateLocalMass<TB>(IFiniteElementBase<TSpace, TB> element, Func<TSpace, double> gamma)
        where TB : IVectorBase<TB>
    {
        var n = element.DOF.Count;
        var basis = element.Basis;
        var analyticalBasis = new IPolynomial[n];
        var mass = new LocalMatrix(n);
        var masterCs = element.MasterElementCoordinateSystem;
        var meshCs = element.Mesh.CoordinateSystem;
        var epZero = TB.Zero;
        var mpZero = masterCs.InverseTransform(epZero);
        double detJ = Math.Abs(masterCs.Jacobian(epZero) * meshCs.Jacobian(mpZero));
    
        var jacobian = new Polynomial { Summands = [(0, 0, detJ)] };
        var gammaPolynomial = InterpolateFunc(element, gamma);
        for (int i = 0; i < n; i++)
            analyticalBasis[i] = (Polynomial)((IAnalyticalBasisFunction<TSpace>)basis[i]).Polynomial;
        for (int i = 0; i < n; i++)
        { 
            for (int j = 0; j <= i; j++)
            {
                double value = 0;
                var phiI = analyticalBasis[i];
                var phiJ = analyticalBasis[j];
                var product = Polynomial.Mult(phiI, phiJ);
                var integrand = Polynomial.Mult(product, gammaPolynomial);
                if (element.Geometry is TriangleGeometry)
                {
                    value = IntegrateTriangle(integrand, jacobian);
                }
                else if (element.Geometry is QuadrangleGeometry)
                {
                    value = IntegrateQuadrangle(integrand, jacobian);
                }
                else if (element.Geometry is BoundaryElementGeometry2D)
                {
                    value = IntegrateSegment(integrand, jacobian);
                }
                mass[i,j] = value;
                mass[j,i] = value;
            }
        }
        return mass;
    }
    public LocalMatrix CalculateLocalMass(IFiniteElementBase<TSpace, TSpace> element, Func<TSpace, double> gamma) =>
        CalculateLocalMass<TSpace>(element, gamma);
    public LocalMatrix CalculateLocalMass(IFiniteElementBase<TSpace, TBoundary> element, Func<TSpace, double> gamma) =>
        CalculateLocalMass<TBoundary>(element, gamma);

    public void CalculateLocalLoad(IFiniteElementBase<TSpace, TBoundary> element, Func<TSpace, double> source, Span<double> outLoad) => CalculateLocalLoad<TBoundary>(element, source, outLoad);
    public void CalculateLocalLoad(IFiniteElementBase<TSpace, TSpace> element, Func<TSpace, double> source, Span<double> outLoad) => CalculateLocalLoad<TSpace>(element, source, outLoad);
    
    public void CalculateLocalLoad<TB>(IFiniteElementBase<TSpace, TB> element, Func<TSpace, double> source, Span<double> outLoad) 
        where TB : IVectorBase<TB>
    {
        var n = element.DOF.Count;
        var basis = element.Basis;
        var analyticalBasis = new IPolynomial[n];
        var masterCs = element.MasterElementCoordinateSystem;
        var meshCs = element.Mesh.CoordinateSystem;
        var epZero = TB.Zero;
        var mpZero = masterCs.InverseTransform(epZero);
        double detJ = Math.Abs(masterCs.Jacobian(epZero) * meshCs.Jacobian(mpZero));
    
        var jacobian = new Polynomial { Summands = [(0, 0, detJ)] };
        var sourcePolynomial = InterpolateFunc(element, source);
        for (int i = 0; i < n; i++)
            analyticalBasis[i] = (Polynomial)((IAnalyticalBasisFunction<TSpace>)basis[i]).Polynomial;
        
        for (int i = 0; i < n; i++)
        {
            double value = 0;
            var phiI = analyticalBasis[i];
            var integrand = Polynomial.Mult(phiI, sourcePolynomial);
            if (element.Geometry is TriangleGeometry)
            {
                value = IntegrateTriangle(integrand, jacobian);
            }
            else if (element.Geometry is QuadrangleGeometry)
            {
                value = IntegrateQuadrangle(integrand, jacobian);
            }
            else if (element.Geometry is BoundaryElementGeometry2D)
            {
                value = IntegrateSegment(integrand, jacobian);
            }
            outLoad[i] = value;
        }
    }
    
}

