using System;
using System.Collections.Generic;
using System.Text;
using Model.Core.Matrix;
using Model.Core.Algebra;

namespace Model.Core.Solver
{
    public sealed class MSG(CsrMatrix CrsMatrix, IFactorizationType Factorization) : ISolver
    {


        double[] ax = new double[CrsMatrix.Size];
        double[] r = new double[CrsMatrix.Size];
        double[] rl = new double[CrsMatrix.Size];
        double[] z = new double[CrsMatrix.Size];
        double[] item = new double[CrsMatrix.Size];

        double b = 0.0;
        double a = 0.0;
        double q = 0.0;


        public Span<double> Ax => ax;
        public Span<double> R => r; //Вектор невязки
        public Span<double> RL => rl; //Вектор невязки
        public Span<double> Z => z; //Вектор спуска
        public Span<double> Item => item;

        double scalarAX = 0.0;


        public (double residual, int iterations) Solve(ReadOnlySpan<double> rhsVector, Span<double> solution, ISolver.SolverParams paramz = default) 
        {
            if (rhsVector.Length != CrsMatrix.Size || solution.Length != CrsMatrix.Size)
                throw new ArgumentException("rhsVector/solution length must equal matrix Size");
            int res_k = 0;
            double norm = rhsVector.Norm();

            if (norm < paramz.Eps)
            {
                solution.Fill(0);
                return (0.0, 0);
            }

            CrsMatrix.MulVec(solution, Ax);
            for (int i = 0; i < CrsMatrix.Size; ++i)
            {
                R[i] = rhsVector[i] - Ax[i];
            }
            Factorization.LMult(R, Item);
            Factorization.UMult(Item, Z);

            for (int kk = 0; kk < paramz.MaxIterations; ++kk)
            {
                CrsMatrix.MulVec(Z, Ax);
                Factorization.LMult(R, Item);
                Factorization.UMult(Item, RL);
                b = RL.ScalarDot(R);

                scalarAX = Ax.ScalarDot(Z);
                a = b / scalarAX;
                for (int i = 0; i < CrsMatrix.Size; ++i)
                {
                    solution[i] = solution[i] + a * Z[i];
                    R[i] = R[i] - a * Ax[i];
                }
                Factorization.LMult(R, Item);
                Factorization.UMult(Item, RL);

                b = RL.ScalarDot(R) / b;

                for (int i = 0; i < CrsMatrix.Size; ++i)
                {
                    Z[i] = RL[i] + b * Z[i];
                }
                q = R.Norm();
                res_k = kk;
                if (q < paramz.Eps * norm) break;
            }
            return (q, res_k);
        }
        public void Dispose() { }
    }
}
