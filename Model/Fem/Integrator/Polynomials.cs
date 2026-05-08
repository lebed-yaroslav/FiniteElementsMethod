using Model.Core.Matrix;
using Model.Fem.Basis;
using Telma;

namespace Model.Fem.Integrator;

public interface IPolynomial : IBasisFunction2D
{
    int Degree { get; }
    Dictionary<(int p, int q), double> Summands { get; set; } //p - х degree, q - у degree
    void Add(IPolynomial poly); //adds to the polynomial
    void Mult(IPolynomial poly); //multiplies the polynomial
    void Delete_Nulls(); //removes summands with 0 value;
    ReadOnlySpan<IBasisFunction2D> Gradient(); //calculates polynomail's gradient
    bool IsPolynomial(); //checks for negative degrees
}

public interface IAnalyticalIntegrator
{
    double IntegrateTriangle(IPolynomial poly, IPolynomial Jacobian);
    double IntegrateQuadrangle(IPolynomial poly, IPolynomial Jacobian);
    double IntegrateBoundary(IPolynomial poly, IPolynomial Jacobian, Vector2D startPoint, Vector2D endPoint);
}

public class Polynomial : IPolynomial
{
    public Dictionary<(int p, int q), double> Summands { get; set; } = [];
    public int Degree
    {
        get
        {
            int d = Summands.Max(i => i.Key.p + i.Key.q);
            return d;
        }
    }

    public static IPolynomial Sum(IPolynomial poly1, IPolynomial poly2)
    {
        var res_poly = new Polynomial();
        res_poly.Add(poly1);
        res_poly.Add(poly2);
        res_poly.Delete_Nulls();

        return res_poly;
    }
    public void Add(IPolynomial poly)
    {
        foreach (var i in poly.Summands)
        {
            if (!Summands.TryAdd(i.Key, i.Value)) Summands[i.Key] += i.Value;
        }

        Delete_Nulls();
    }

    public static IPolynomial Mult(IPolynomial poly1, IPolynomial poly2)
    {
        var res_poly = new Polynomial();

        foreach (var i in poly1.Summands)
        {
            foreach (var j in poly2.Summands)
            {
                if (!res_poly.Summands.TryAdd((i.Key.p + j.Key.p, i.Key.q + j.Key.q), i.Value * j.Value)) 
                    res_poly.Summands[(i.Key.p + j.Key.p, i.Key.q + j.Key.q)] += i.Value * j.Value;
            }
        }

        res_poly.Delete_Nulls();
        return res_poly;
    }
    public void Mult(IPolynomial poly)
    {
        var res = new Dictionary<(int p, int q), double>();
        foreach (var i in Summands)
        {
            foreach (var j in poly.Summands)
            {
                if (!res.TryAdd((i.Key.p + j.Key.p, i.Key.q + j.Key.q), i.Value * j.Value)) 
                    res[(i.Key.p + j.Key.p, i.Key.q + j.Key.q)] += i.Value * j.Value;
            }
        }

        Summands = res;
        Delete_Nulls();
    }

    public double Value(Vector2D point)
    {
        double res = 0.0;
        foreach (var i in Summands)
        {
            res += i.Value * Math.Pow(point.X, i.Key.p) * Math.Pow(point.Y, i.Key.q);
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
        var grad = new IPolynomial[2] { new Polynomial(), new Polynomial() };
        foreach (var i in Summands)
        {
            grad[0].Summands.Add((i.Key.p - 1, i.Key.q), i.Value * i.Key.p);
            grad[1].Summands.Add((i.Key.p, i.Key.q - 1), i.Value * i.Key.q);
        }

        grad[0].Delete_Nulls();
        grad[1].Delete_Nulls();
        return grad;
    }

    public void Delete_Nulls()
    {
        var summ_list = new List<(int i, int j)>();

        foreach (var i in Summands)
        {
            if (i.Value == 0) summ_list.Add((i.Key.p, i.Key.q));
        }

        foreach (var i in summ_list)
        {
            Summands.Remove(i);
        }
    }

    public bool IsPolynomial() //for rational functions
    {
        foreach (var i in Summands)
        {
            if (i.Key.p < 0 || i.Key.q < 0) return false;
        }

        return true;
    }
}


public static class AnalyticalIntegration
{
    static readonly int D = 3 * 2 + 2; // max poly degree after integration
    static readonly int[,] C = new int[D, D];

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

    public static double IntegrateTriangle(IPolynomial poly, IPolynomial jacobian)
    {
        double res = 0.0, tmp = 0.0;
        var polyJ = Polynomial.Mult(jacobian, poly);
        if (!polyJ.IsPolynomial()) throw new NotImplementedException();

        foreach (var i in polyJ.Summands)
        {
            for (int k = 0; k <= i.Key.q + 1; k++)
            {
                tmp += Math.Pow(-1.0, k) * C[i.Key.q + 1, k] * 1.0 / ((i.Key.q + 1) * (k + i.Key.p + 1));
            }

            res += i.Value * tmp;
            tmp = 0.0;
        }

        return res;
    }

    public static double IntegrateQuadrangle(IPolynomial poly, IPolynomial jacobian)
    {
        double res = 0.0;
        var polyJ = Polynomial.Mult(jacobian, poly);
        if (!polyJ.IsPolynomial()) throw new NotImplementedException();

        foreach (var i in polyJ.Summands)
        {
            res += i.Value / ((i.Key.q + 1) * (i.Key.p + 1));
        }

        return res;
    }

    public static double IntegrateBoundary(IPolynomial poly, IPolynomial jacobian, Vector2D startPoint, Vector2D endPoint)
    {
        double res = 0.0, tmp = 0.0;
        var polyJ = Polynomial.Mult(jacobian, poly);
        if (!polyJ.IsPolynomial()) throw new NotImplementedException();

        double deltaX = endPoint.X - startPoint.X;
        double deltaY = endPoint.Y - startPoint.Y;
        double root = Math.Sqrt(Math.Pow(deltaX, 2) + Math.Pow(deltaY, 2));

        foreach (var item in polyJ.Summands)
        {
            for (int i = 0; i <= item.Key.p; i++)
            {
                for (int j = 0; j <= item.Key.q; j++)
                {
                    tmp += C[item.Key.p, i] * Math.Pow(startPoint.X, item.Key.p - i) * Math.Pow(deltaX, i)
                         * C[item.Key.q, j] * Math.Pow(startPoint.Y, item.Key.q - j) * Math.Pow(deltaY, j)
                         / (i + j + 1);
                }

            }

            res += tmp * root * item.Value;
            tmp = 0.0;
        }

        return res;
    }
}

