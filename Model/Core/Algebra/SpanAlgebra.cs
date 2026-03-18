using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;

namespace Model.Core.Algebra;
public static class SpanAlgebra
{
    public static double ScalarDot(this ReadOnlySpan<double> a, ReadOnlySpan<double> b)
    {
        double d = 0;
        for (int i = 0; i < a.Length; i++) d += a[i] * b[i];
        return d;
    }
    public static double Norm(this ReadOnlySpan<double> a) => Math.Sqrt(a.ScalarDot(a));

    public static double[] SummVec(this ReadOnlySpan<double> a, ReadOnlySpan<double> b)
    {
        var outVec = new double[a.Length];

        for (int i = 0; i < a.Length; ++i)
        {
            outVec[i] = a[i] + b[i];
        }
        return outVec;
    }
    public static double[] DiffVec(this ReadOnlySpan<double> a, ReadOnlySpan<double> b)
    {
        var outVec = new double[a.Length];

        for (int i = 0; i < a.Length; ++i)
        {
            outVec[i] = a[i] - b[i];
        }
        return outVec;
    }
    public static double[] MultiplyByNumber(this ReadOnlySpan<double> a,double number)
    {
        var outVec = new double[a.Length];
        for (int i = 0;i < a.Length; i++)
        {
            outVec[i] = a[i] * number;
        }
        return outVec;
    }
}

