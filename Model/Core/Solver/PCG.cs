using System.Diagnostics;
using Model.Core.Matrix;
using Model.Core.Solver.Precondition;
using Model.Core.Vector;

namespace Model.Core.Solver;

/// <summary>Preconditioned conjugate gradient solver</summary>
/// <param name="Matrix">System matrix</param>
/// <param name="Factorization">Matrix factorization</param>
public sealed record PCGSolver(
    IGlobalMatrix Matrix,
    IFactorization Factorization
) : ISolver
{
    public (double residual, int iterations) Solve(
        ReadOnlySpan<double> rhsVector,
        Span<double> solution,
        ISolver.SolverParams paramz = default
    ) 
    {
        Debug.Assert(rhsVector.Length == Matrix.Size);
        Debug.Assert(solution.Length == Matrix.Size);

        double rhsNorm = rhsVector.Norm();

        if (rhsNorm < paramz.Eps)
        {
            solution.Clear();
            return (0.0, 0);
        }

        int n = Matrix.Size;
        var r = new double[n]; // residual
        var z = new double[n]; // conjugate direction
        var az = new double[n];
        var mInvR = new double[n];

        // r = f - ax
        Matrix.MulVec(solution, r);
        rhsVector.Sub(r, r);

        // z = (U^-1) * (L^-1) * (f - ax)
        Factorization.LInvMul(r, z);
        Factorization.UInvMul(z, z);

        // mInvR = (U^-1) * (L^-1) * r
        Array.Copy(z, mInvR, n);

        double dotRlR = mInvR.Dot(r);

        double rNorm = 0;
        for (int iter = 0; iter < paramz.MaxIterations; ++iter)
        {
            Matrix.MulVec(z, az);

            double alpha = dotRlR / az.Dot(z);
            solution.AddScaled(alpha, z, solution); // x = x + a * z
            r.AddScaled(-alpha, az, r); // r = r - a * z

            rNorm = r.Norm();
            if (rNorm < paramz.Eps * rhsNorm)
                return (rNorm, iter);

            // mInvR = (U^-1) * (L^-1) * r
            Factorization.LInvMul(r, mInvR);
            Factorization.UInvMul(mInvR, mInvR);

            double newDotRlR = mInvR.Dot(r);
            double beta = newDotRlR / dotRlR; // b = (mInvR_k, r_k) / (mInvR_{k-1}, r_{k-1})
            mInvR.AddScaled(beta, z, z); // z = mInvR + b * z
            dotRlR = newDotRlR;
        }

        return (rNorm, paramz.MaxIterations);
    }
}
