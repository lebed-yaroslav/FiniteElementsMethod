using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using Model.Core.Matrix;
using Model.Core.Algebra;
namespace Model.Core.Solver;

internal class TimeSolver
{
    public double[] NeyavTwoScheme(CsrMatrix G, CsrMatrix M, CsrMatrix MS3, double[] q0, Func<int, double, double[]>getB,
        double dt, int timeSteps)
    {
        //неявная двуслойная схема.
        // A = (1 / dt) * M + G + MS3
        //Если не меняется ни шаг по времени, ни лямбда, ни сигма, ни бета - матрица А остаётся постоянной
        CsrMatrix MdivDt = CsrMatrix.Clone(M);
        M.MultiplyByNumber(1.0 / dt, MdivDt);
        CsrMatrix MdivDtPlusG = CsrMatrix.Clone(M);
        MdivDt.Addition(G, MdivDtPlusG);
        CsrMatrix A = CsrMatrix.Clone(M);
        MdivDtPlusG.Addition(MS3, A);

        double[] qcurrent = new double[q0.Length];
        double[] q_next = new double[q0.Length];
        Array.Copy(q0, qcurrent, q0.Length);
        for (int j = 1; j <= timeSteps; j++)
        {
            double currentTime = j * dt;
            double[] bj = getB(j,currentTime);//Вектор b формируем на каждом временном шаге(j), currentTime-текущее время.
            //формирование правой части d = b(j) + (1/dt) * Mq^(j-1)
            double[] Mq = new double[M.Size];
            M.MulVec(qcurrent, Mq);
            Mq = SpanAlgebra.MultiplyByNumber(Mq, 1.0 / dt);
            double[] d = SpanAlgebra.SummVec(bj, Mq);
            /*Получается слау A*q_next = d
             * Solver.MSG,LOS.. => get q_next*/
            qcurrent = q_next;
            for (int i = 0; i < qcurrent.Length; i++)
                Console.WriteLine($"{qcurrent[i]}");
            //На следующей итерации матрица А остается той же, а правая часть поменяется

        }
        return qcurrent;
    }
} 

