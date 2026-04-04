//using Model.Core.CoordinateSystem;
//using Model.Core.Matrix;
//using Model.Core.Solver.Precondition;
//using Model.Model;
//using Model.Model.Elements;
//using Model.Model.Elements.Segment;
//using Model.Model.Mesh;
//using Telma;

//namespace UnitTests;

//public class CsrILUFactorizationTests
//{
//    [Fact]
//    public void LUPreconditioner_Solves2x2SystemExactly()
//    {
//        double[,] a =
//        {
//            { 1.0, 1.0 ,-1.0},
//            { 1.0, -2.0, 3.0 },
//            {2.0,3.0,1.0 }
//        };

//        var xExpected = new[] { 2.0, -1.0 ,4.0};
//        var b = MultiplyDense(a, xExpected);

//        var csr = BuildSymmetricCsr(a);
//        var ilu = CsrILUFactorization.Create(csr, 1e-14);
//        double[] y = new double[csr.Size];
//        double[] x = new double[csr.Size];
//        ilu.LInvMul(b, y);
//        ilu.UInvMul(y, x);

//        AssertVectorClose(xExpected, x, 1e-12);
//       // AssertResidualSmall(a, x, b, 1e-12);
//    }

//    //[Fact]
//    //public void LUPreconditioner_Solves3x3SystemExactly()
//    //{
//    //    double[,] a =
//    //    {
//    //        { 6.0, 2.0, 1.0 },
//    //        { 2.0, 5.0, 2.0 },
//    //        { 1.0, 2.0, 4.0 }
//    //    };

//    //    var xExpected = new[] { 1.5, -2.0, 0.75 };
//    //    var b = MultiplyDense(a, xExpected);

//    //    var csr = BuildSymmetricCsr(a);
//    //    var ilu = CsrILUFactorization.Factory.Instance.Factorize(csr, 1e-14);
//    //    var preconditioner = new LUPreconditioner(ilu);

//    //    var x = new double[xExpected.Length];
//    //    preconditioner.Apply(b, x);

//    //    AssertVectorClose(xExpected, x, 1e-12);
//    //    AssertResidualSmall(a, x, b, 1e-12);
//    //}

//    //[Fact]
//    //public void LUPreconditioner_ReusesFactorizationForSeveralRightHandSides()
//    //{
//    //    double[,] a =
//    //    {
//    //        { 7.0, 1.0, 0.0 },
//    //        { 1.0, 8.0, 2.0 },
//    //        { 0.0, 2.0, 5.0 }
//    //    };

//    //    var csr = BuildSymmetricCsr(a);
//    //    var ilu = CsrILUFactorization.Factory.Instance.Factorize(csr, 1e-14);
//    //    var preconditioner = new LUPreconditioner(ilu);

//    //    var expectedSolutions = new[]
//    //    {
//    //        new[] { 1.0, 0.0, -1.0 },
//    //        new[] { -0.5, 2.0, 1.5 },
//    //        new[] { 3.0, -1.0, 0.25 }
//    //    };

//    //    foreach (var xExpected in expectedSolutions)
//    //    {
//    //        var b = MultiplyDense(a, xExpected);
//    //        var x = new double[xExpected.Length];

//    //        preconditioner.Apply(b, x);

//    //        AssertVectorClose(xExpected, x, 1e-12);
//    //        AssertResidualSmall(a, x, b, 1e-12);
//    //    }
//    //}

//    private static CsrMatrix BuildSymmetricCsr(double[,] dense)
//    {
//        int n = dense.GetLength(0);
//        if (n != dense.GetLength(1))
//            throw new ArgumentException("Matrix must be square.");

//        var ig = new int[n + 1];
//        var jgList = new List<int>();

//        for (int i = 0; i < n; i++)
//        {
//            ig[i] = jgList.Count;
//            for (int j = 0; j < i; j++)
//            {
//                if (Math.Abs(dense[i, j]) > 0.0)
//                    jgList.Add(j);
//            }
//        }
//        ig[n] = jgList.Count;

//        var matrix = new CsrMatrix(new CsrMatrix.Portrait(ig, jgList.ToArray()));

//        for (int i = 0; i < n; i++)
//            matrix.Di[i] = dense[i, i];

//        for (int i = 0; i < n; i++)
//        {
//            for (int p = matrix.Ig[i]; p < matrix.Ig[i + 1]; p++)
//            {
//                int j = matrix.Jg[p];
//                matrix.Ggl[p] = dense[i, j];
//            }
//        }

//        return matrix;
//    }

//    private static double[] MultiplyDense(double[,] matrix, IReadOnlyList<double> vector)
//    {
//        int n = matrix.GetLength(0);
//        var result = new double[n];

//        for (int i = 0; i < n; i++)
//        {
//            double sum = 0.0;
//            for (int j = 0; j < n; j++)
//                sum += matrix[i, j] * vector[j];
//            result[i] = sum;
//        }

//        return result;
//    }

//    private static void AssertResidualSmall(double[,] a, IReadOnlyList<double> x, IReadOnlyList<double> b, double tolerance)
//    {
//        var ax = MultiplyDense(a, x);
//        double residualNormSquared = 0.0;

//        for (int i = 0; i < ax.Length; i++)
//        {
//            double ri = b[i] - ax[i];
//            residualNormSquared += ri * ri;
//        }

//        Assert.True(Math.Sqrt(residualNormSquared) < tolerance,
//            $"Residual is too large: {Math.Sqrt(residualNormSquared)}");
//    }

//    private static void AssertVectorClose(IReadOnlyList<double> expected, IReadOnlyList<double> actual, double eps)
//    {
//        Assert.Equal(expected.Count, actual.Count);

//        for (int i = 0; i < expected.Count; i++)
//            Assert.True(Math.Abs(expected[i] - actual[i]) < eps,
//                $"Mismatch at {i}: expected {expected[i]}, actual {actual[i]}");
//    }
//}
