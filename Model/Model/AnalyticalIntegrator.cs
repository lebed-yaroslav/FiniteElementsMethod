using Model.Core.Matrix;
using Model.Model.Basis;
using Telma;

namespace Model.Model;

public interface IPolynomial : IBasisFunction
{
   int Degree { get; }
   List<(int p, int q, double coeff)> Summands { get; set; } //p - х degree, q - у degree
   void Add(IPolynomial poly); //add to the polynomial
   void Mult(IPolynomial poly); //multiply the polynomial
}

public interface IAnalyticalIntegrator
{
   double IntegrateTriangle(IPolynomial poly, IPolynomial Jacobian);
   double IntegrateQuadrangle(IPolynomial poly, IPolynomial Jacobian);
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

   public ReadOnlySpan<IBasisFunction> Gradient()
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


public static class AnalyticalIntegrator
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
}
