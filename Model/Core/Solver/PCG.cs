using System.Diagnostics;
using Model.Core.Matrix;
using Model.Core.Solver.Precondition;
using Model.Core.Vector;

namespace Model.Core.Solver;

/// <summary>Preconditioned conjugate gradient solver</summary>
public sealed record PCGSolver(
    PreconditionerCreator PreconditionerCreator
) : ISolver
{
    private IPreconditioner? _preconditioner;

    public IGlobalMatrix? Matrix {
        set
        {
            if (field != null)
                _preconditioner = PreconditionerCreator((IGlobalMatrix)field.Clone());
            else
                _preconditioner = null;
            field = value;
        }
        private get;
    }

    public (double residual, int iterations) Solve(
        ReadOnlySpan<double> rhsVector,
        Span<double> solution,
        ISolver.SolverParams paramz = default
    ) 
    {
        if (Matrix == null)
            throw new InvalidOperationException("Cannot solve system: matrix has not been properly initialized");

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
        var p = new double[n]; // search direction
        var ap = new double[n];
        var z = new double[n]; // preconditioned residual

        // r = f - ax
        Matrix.MulVec(solution, r);
        rhsVector.Sub(r, r);

        // p = (M^-1) * (f - ax)
        _preconditioner!.Apply(r, p);

        // z = (M^-1) * r
        Array.Copy(p, z, n);

        double dotRlR = z.Dot(r);

        double rNorm = 0;
        for (int iter = 0; iter < paramz.MaxIterations; ++iter)
        {
            Matrix.MulVec(p, ap);

            double alpha = dotRlR / ap.Dot(p);
            solution.AddScaled(alpha, p, solution); // x = x + a * z
            r.AddScaled(-alpha, ap, r); // r = r - alpha * ap

            rNorm = r.Norm();
            if (rNorm < paramz.Eps * rhsNorm)
                return (rNorm, iter);

            // z = (M^-1) * r
            _preconditioner!.Apply(r, z);

            double newDotRlR = z.Dot(r);
            double beta = newDotRlR / dotRlR; // b = (z_k, r_k) / (z_{k-1}, r_{k-1})
            z.AddScaled(beta, p, p); // p = z + b * p
            dotRlR = newDotRlR;
        }

        return (rNorm, paramz.MaxIterations);
    }

    public void Dispose() { }
}
