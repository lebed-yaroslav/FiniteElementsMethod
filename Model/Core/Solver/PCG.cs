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

    public IGlobalMatrix? Matrix
    {
        set
        {
            if (field != null)
            {
                _preconditioner = PreconditionerCreator((IGlobalMatrix)field.Clone());
                int n = field.Size;
                _r = new double[n];
                _p = new double[n];
                _ap = new double[n];
                _z = new double[n];
            }
            field = value;
        }
        private get;
    }

    // Pre-Allocated buffers
    private double[] _r = []; // residual
    private double[] _p = []; // search direction
    private double[] _ap = [];
    private double[] _z = []; // preconditioned residual

    public (double residual, int iterations) Solve(
        ReadOnlySpan<double> rhsVector,
        Span<double> solution,
        ISolver.Params paramz = default
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

        // r = f - ax
        Matrix.MulVec(solution, _r);
        rhsVector.Sub(_r, _r);

        // p = (M^-1) * (f - ax)
        _preconditioner!.Apply(_r, _p);

        // z = (M^-1) * r
        Array.Copy(_p, _z, n);

        double dotRlR = _z.Dot(_r);

        double rNorm = 0;
        for (int iter = 0; iter < paramz.MaxIterations; ++iter)
        {
            Matrix.MulVec(_p, _ap);

            double alpha = dotRlR / _ap.Dot(_p);
            solution.AddScaled(alpha, _p, solution); // x = x + a * z
            _r.AddScaled(-alpha, _ap, _r); // r = r - alpha * ap

            rNorm = _r.Norm();
            if (rNorm < paramz.Eps * rhsNorm)
                return (rNorm, iter);

            // z = (M^-1) * r
            _preconditioner!.Apply(_r, _z);

            double newDotRlR = _z.Dot(_r);
            double beta = newDotRlR / dotRlR; // b = (z_k, r_k) / (z_{k-1}, r_{k-1})
            _z.AddScaled(beta, _p, _p); // p = z + b * p
            dotRlR = newDotRlR;
        }

        return (rNorm, paramz.MaxIterations);
    }

    public void Dispose() { }
}
