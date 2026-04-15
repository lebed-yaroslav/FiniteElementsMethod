using System.Diagnostics;
using System.Security.Cryptography;

namespace Model.Core.Vector;

public static class SpanAsVectorExtensions
{
    extension(ReadOnlySpan<double> self)
    {
        public double Dot(ReadOnlySpan<double> rhs)
        {
            Debug.Assert(self.Length == rhs.Length);
            double res = 0;
            for (int i = 0; i < self.Length; i++)
                res += self[i] * rhs[i];
            return res;
        }

        public double Norm() => Math.Sqrt(self.Dot(self));

        public void AddScaled(double alpha, ReadOnlySpan<double> rhs, Span<double> res)
        {
            Debug.Assert(rhs.Length == self.Length);
            Debug.Assert(res.Length == self.Length);

            for (int i = 0; i < self.Length; i++)
                res[i] = self[i] + alpha * rhs[i];
        }

        public void Sub(ReadOnlySpan<double> rhs, Span<double> res)
        {
            Debug.Assert(rhs.Length == self.Length);
            Debug.Assert(res.Length == self.Length);

            for (int i = 0; i < self.Length; ++i)
                res[i] = self[i] - rhs[i];
        }
    }

    extension (Span<double> self)
    {
        public void AddScaled(double alpha, ReadOnlySpan<double> vec)
            => self.AddScaled(alpha, vec, self);

        public void AssignScaled(double alpha, ReadOnlySpan<double> vec)
        {
            Debug.Assert(self.Length == vec.Length);

            for (int i = 0; i < self.Length; ++i)
                self[i] = alpha * vec[i];
        }
    }
}
