using Model.Core.Matrix;
using Model.Fem.Basis;
using Telma;
using Telma.Extensions;

namespace Model.Fem.Integrator;

public interface IPolynomial<TSpace> : IBasisFunction<TSpace>
    where TSpace : IVectorBase<TSpace>
{
    int Degree { get; }  //polynomial degree
    void Delete_Nulls(); //removes summands with 0 value
    IPolynomial<TSpace> Scale(double scal); //multiplies the Polynomial by scal
}

public interface IPolynomial2D : IPolynomial<Vector2D>
{
    Dictionary<(int p, int q), double> Summands { get; } //p - х degree, q - у degree
    void Add(IPolynomial2D poly); //adds to the Polynomial2D
    void Mult(IPolynomial2D poly); //multiplies the Polynomial2D
    ReadOnlySpan<IPolynomial2D> Gradient(); //calculates polynomial's gradient
}

public class Polynomial2D : IPolynomial2D
{
    public Dictionary<(int p, int q), double> Summands { get; } = [];
    public int Degree => Summands.Count == 0 ? 0 : Summands.Max(i => i.Key.p + i.Key.q);

    public static IPolynomial2D Sum(IPolynomial2D poly1, IPolynomial2D poly2)
    {
        var res_poly = new Polynomial2D();
        res_poly.Add(poly1);
        res_poly.Add(poly2);
        res_poly.Delete_Nulls();

        return res_poly;
    }

    public void Add(IPolynomial2D poly)
    {
        foreach (var i in poly.Summands)
        {
            if (!Summands.TryAdd(i.Key, i.Value)) Summands[i.Key] += i.Value;
        }

        Delete_Nulls();
    }

    public static IPolynomial2D Mult(IPolynomial2D poly1, IPolynomial2D poly2)
    {
        var res_poly = new Polynomial2D();

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

    public void Mult(IPolynomial2D poly)
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

        Summands.Clear();

        foreach (var item in res)
        {
            Summands.Add(item.Key, item.Value);
        }

        Delete_Nulls();
    }

    public static Polynomial2D TensorMult(IPolynomial1D poly1, IPolynomial1D poly2)
    {
        var res_poly = new Polynomial2D();

        foreach (var i in poly1.Summands)
        {
            foreach (var j in poly2.Summands)
            {
                if (!res_poly.Summands.TryAdd((i.Key, j.Key), i.Value * j.Value))
                    res_poly.Summands[(i.Key, j.Key)] += i.Value * j.Value;
            }
        }

        res_poly.Delete_Nulls();
        return res_poly;
    }

    public static Polynomial2D ScalMult(Polynomial2D poly, double C)
    {
        var res_poly = new Polynomial2D();

        foreach (var i in poly.Summands)
        {
            res_poly.Summands.Add(i.Key, i.Value * C);
        }

        res_poly.Delete_Nulls();
        return res_poly;
    }

    public static void AddScaled(Polynomial2D res_poly, IPolynomial2D poly, double C)
    {
        if (C == 0.0) return;

        foreach (var i in poly.Summands)
        {
            if (!res_poly.Summands.TryAdd(i.Key, i.Value * C)) res_poly.Summands[i.Key] += i.Value * C;
        }

        res_poly.Delete_Nulls();
    }

    public IPolynomial<Vector2D> Scale(double scal) => ScalMult(this, scal);

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

    public ReadOnlySpan<IPolynomial2D> Gradient()
    {
        var grad = new IPolynomial2D[2] { new Polynomial2D(), new Polynomial2D() };
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
            if (i.Value == 0.0) summ_list.Add((i.Key.p, i.Key.q));
        }

        foreach (var i in summ_list)
        {
            Summands.Remove(i);
        }
    }
}
