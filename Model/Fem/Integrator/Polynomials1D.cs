using Model.Core.Matrix;
using Model.Fem.Basis;
using Telma;

namespace Model.Fem.Integrator;

public interface IPolynomial1D : IPolynomial<Vector1D>
{
    Dictionary<int, double> Summands { get; } //p - х degree
    void Add(IPolynomial1D poly); //adds to the Polynomial1D
    void Mult(IPolynomial1D poly); //multiplies the Polynomial1D
    void Delete_Nulls(); //removes summands with 0 value;
    IPolynomial1D Gradient(); //calculates polynomail's gradient
}

public class Polynomial1D : IPolynomial1D
{
    public Dictionary<int, double> Summands { get; } = [];
    public int Degree => Summands.Count == 0 ? 0 : Summands.Max(i => i.Key);

    public static IPolynomial1D Sum(IPolynomial1D poly1, IPolynomial1D poly2)
    {
        var res_poly = new Polynomial1D();
        res_poly.Add(poly1);
        res_poly.Add(poly2);
        res_poly.Delete_Nulls();

        return res_poly;
    }
    public void Add(IPolynomial1D poly)
    {
        foreach (var i in poly.Summands)
        {
            if (!Summands.TryAdd(i.Key, i.Value)) Summands[i.Key] += i.Value;
        }

        Delete_Nulls();
    }

    public static IPolynomial1D Mult(IPolynomial1D poly1, IPolynomial1D poly2)
    {
        var res_poly = new Polynomial1D();

        foreach (var i in poly1.Summands)
        {
            foreach (var j in poly2.Summands)
            {
                if (!res_poly.Summands.TryAdd(i.Key + j.Key, i.Value * j.Value))
                    res_poly.Summands[i.Key + j.Key] += i.Value * j.Value;
            }
        }

        res_poly.Delete_Nulls();
        return res_poly;
    }

    public void Mult(IPolynomial1D poly)
    {
        var res = new Dictionary<int, double>();
        foreach (var i in Summands)
        {
            foreach (var j in poly.Summands)
            {
                if (!res.TryAdd(i.Key + j.Key, i.Value * j.Value))
                    res[(i.Key + j.Key)] += i.Value * j.Value;
            }
        }

        Summands.Clear();

        foreach (var item in res)
        {
            Summands.Add(item.Key, item.Value);
        }

        Delete_Nulls();
    }


    public static Polynomial1D ScalMult(Polynomial1D poly, double C)
    {
        var res_poly = new Polynomial1D();

        foreach (var i in poly.Summands)
        {
            res_poly.Summands.Add(i.Key, i.Value * C);
        }

        res_poly.Delete_Nulls();
        return res_poly;
    }

    public double Value(Vector1D point)
    {
        double res = 0.0;
        foreach (var i in Summands)
        {
            res += i.Value * Math.Pow(point, i.Key);
        }
        return res;
    }

    public Vector1D Derivatives(Vector1D point)
    {
        var grad = Gradient();
        var res = new Vector1D(grad.Value(point));

        return res;
    }

    public IPolynomial1D Gradient()
    {
        var grad = new Polynomial1D();
        foreach (var i in Summands)
        {
            grad.Summands.Add(i.Key - 1, i.Value * i.Key);
        }

        grad.Delete_Nulls();
        return grad;
    }

    public void Delete_Nulls()
    {
        var summ_list = new List<int>();

        foreach (var i in Summands)
        {
            if (i.Value == 0.0) summ_list.Add(i.Key);
        }

        foreach (var i in summ_list)
        {
            Summands.Remove(i);
        }
    }
}
