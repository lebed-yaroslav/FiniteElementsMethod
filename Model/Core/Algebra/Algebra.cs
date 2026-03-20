using System;
using System.Collections.Generic;
using System.Text;

namespace Model.Core.Algebra
{
    public static class Algebra
    {
        public static double ScalarDot(this ReadOnlySpan<double> a, ReadOnlySpan<double> b)
        {
            double d = 0;
            for (int i = 0; i < a.Length; i++) d += a[i] * b[i];
            return d;
        }
        public static double Norm(this ReadOnlySpan<double> a) => Math.Sqrt(a.ScalarDot(a));
    }
}
