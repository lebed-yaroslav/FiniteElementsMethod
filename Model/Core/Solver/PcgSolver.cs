using System.Diagnostics;
using Model.Core.Matrix;
using Model.Core.Vector;

namespace Model.Core.Solver;

/// <summary>Preconditioned conjugate gradient method</summary>
/// <param name="Matrix"></param>
/// <param name="Factorization"></param>
public sealed record PcgSolver(
    CsrMatrix Matrix,
    IFactorization Factorization
) : ISolver
{
    double[] ax = new double[Matrix.Size];
    double[] r = new double[Matrix.Size];
    double[] rl = new double[Matrix.Size];
    double[] z = new double[Matrix.Size];
    double[] item = new double[Matrix.Size];

    double b = 0.0;
    double a = 0.0;
    double rNorm = 0.0;


    public Span<double> Ax => ax;
    public Span<double> R => r; //Вектор невязки
    public Span<double> RL => rl; //Вектор невязки
    public Span<double> Z => z; //Вектор спуска
    public Span<double> Item => item;

    double scalarAX = 0.0;


    public (double residual, int iterations) Solve(ReadOnlySpan<double> rhsVector, Span<double> solution, ISolver.SolverParams paramz = default) 
    {
        Debug.Assert(rhsVector.Length == Matrix.Size);
        Debug.Assert(solution.Length == Matrix.Size);

        double norm = rhsVector.Norm();

        if (norm < paramz.Eps)
        {
            solution.Clear();
            return (0.0, 0);
        }

        Matrix.MulVec(solution, Ax);
        rhsVector.Sub(Ax, R);
        Factorization.LInvMul(R, Item);
        Factorization.UInvMul(Item, Z);

        for (int iter = 0; iter < paramz.MaxIterations; ++iter)
        {
            Matrix.MulVec(Z, Ax);
            Factorization.LInvMul(R, Item);
            Factorization.UInvMul(Item, RL);
            b = RL.Dot(R);

            scalarAX = Ax.Dot(Z);
            a = b / scalarAX;

            solution.AddScaled(a, Z, solution);
            R.AddScaled(-a, Z, R);

            Factorization.LInvMul(R, Item);
            Factorization.UInvMul(Item, RL);

            b = RL.Dot(R) / b;

            RL.AddScaled(b, Z, Z);
 
            rNorm = R.Norm();

            if (rNorm < paramz.Eps * norm)
                return (rNorm, iter);
        }

        return (rNorm, paramz.MaxIterations);
    }
}
