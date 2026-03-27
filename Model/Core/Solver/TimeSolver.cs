using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using Model.Core.Matrix;
using Model.Core.Algebra;
using System.Reflection.Metadata.Ecma335;
namespace Model.Core.Solver;

internal class TimeSolver
{
    public double[] NeyavTwoScheme(CsrMatrix G, CsrMatrix M, CsrMatrix MS3, double[] q0,
        Func<int, double, double[]>getB, double dt, int timeSteps)
    {
        //неявная двуслойная схема.
        // A = (1 / dt) * M + G + MS3
        //Если не меняется ни шаг по времени, ни лямбда, ни сигма, ни бета - матрица А остаётся постоянной
        CsrMatrix A = CsrMatrix.Clone(M);
        M.MultiplyByNumber(1.0 / dt, A);
        A.Addition(G, A);
        A.Addition(MS3, A);

        double[] q_current = new double[q0.Length];
        double[] q_next = new double[q0.Length];
        Array.Copy(q0, q_current, q0.Length);

        Console.WriteLine($"Шаг No 0");
        for (int i = 0; i < q_current.Length; i++)
            Console.Write($"{q_current[i]}" + " ");
        Console.WriteLine();

        for (int j = 1; j <= timeSteps; j++)
        {
            double currentTime = j * dt;
            double[] bj = getB(j,currentTime);//Вектор b формируем на каждом временном шаге(j), currentTime-текущее время.
            //формирование правой части d = b(j) + (1/dt) * Mq^(j-1)
            double[] Mq = new double[q0.Length];
            M.MulVec(q_current, Mq);
            Mq = SpanAlgebra.MultiplyByNumber(Mq, 1.0 / dt);
            double[] d = SpanAlgebra.SummVec(bj, Mq);
            /*Получается слау A*q_next = d
             * Solver.MSG,LOS.. => get q_next*/
            q_current = q_next;
            Console.WriteLine($"Шаг No{j}");
            for (int i = 0; i < q_current.Length; i++)
                Console.Write($"{q_current[i]}" + " ");
            Console.WriteLine();
            //На следующей итерации матрица А остается той же, а правая часть поменяется

        }
        return q_current;
    }
    public double[] NeyavThreeScheme(CsrMatrix G,CsrMatrix M,CsrMatrix MS3, double[] q0,
        Func<int, double, double[]>getB,int timeSteps,double dt) 
     {
        //Неявная трёхслойная схема
        /* Для работы этой схемы нужно еще q1, её можно найти, воспользовавшись неявной двуслойной
        схемой*/
        //Шаг считается постоянным
        double[] q1 = NeyavTwoScheme(G, M, MS3, q0, getB, dt, 1);
        //A = (3/2dt)*M+G+MS3
        
        CsrMatrix A = CsrMatrix.Clone(M);
        M.MultiplyByNumber(3.0 / (2.0 * dt), A);
        A.Addition(G, A);
        A.Addition(MS3, A);

        double[] q_prev = new double[q0.Length]; //q^(j-2)
        double[] q_current = new double[q0.Length]; //q^(j-1)
        double[] q_next = new double[q0.Length]; //q^(j)

        Array.Copy(q0,q_prev,q0.Length);
        Array.Copy(q1, q_current,q0.Length);

        Console.WriteLine($"Шаг No 0");
        for (int i = 0; i < q_prev.Length; i++)
            Console.Write($"{q_prev[i]}" + " ");
        Console.WriteLine();

        Console.WriteLine($"Шаг No 1");
        for (int i = 0; i < q_current.Length; i++)
            Console.Write($"{q_current[i]}" + " ");
        Console.WriteLine();


        for (int j = 2;j<=timeSteps;j++)
        {
            //d = b(j) - (1/2dt)*M*q^(j-2) + (2/dt)*M*q^(j-1)
            double currentTime = j * dt;
            double[] bj = getB(j, currentTime);
            double[] Mq_prev = new double[q0.Length];
            double[] Mq_current = new double[q0.Length];
            M.MulVec(q_prev, Mq_prev);
            M.MulVec(q_current, Mq_current);
            Mq_prev = SpanAlgebra.MultiplyByNumber(Mq_prev, 1.0 / (2.0 * dt));
            Mq_current = SpanAlgebra.MultiplyByNumber(Mq_current, 2.0 / dt);

            double[] d = SpanAlgebra.DiffVec(bj, Mq_prev);
            d = SpanAlgebra.SummVec(d, Mq_current);
            /*Получается слау A*q_next = d
             * Solver.MSG,LOS.. => get q_next*/
            q_prev = q_current;
            q_current = q_next;
            Console.WriteLine($"Шаг No{j}");
            for (int i = 0; i < q_current.Length; i++)
                Console.Write($"{q_current[i]}" + " ");
            Console.WriteLine();
        }
        return q_current;
    }
    public double[] YavTwoScheme(CsrMatrix G, CsrMatrix M, CsrMatrix MS3, double[] q0,
        Func<int, double, double[]> getB, double dt, int timeSteps)
    {
        //явная двуслойная схема
        //A = (M+MS3)/dt
        CsrMatrix A = CsrMatrix.Clone(M);
        M.Addition(MS3, A);
        A.MultiplyByNumber(1.0 / dt, A);

        double[] q_current = new double[q0.Length];
        double[] q_next = new double[q0.Length];
        Array.Copy(q0,q_current,q0.Length);

        Console.WriteLine($"Шаг No 0");
        for (int i = 0; i < q_current.Length; i++)
            Console.Write($"{q_current[i]}" + " ");
        Console.WriteLine();

        for (int j = 1; j <= timeSteps; j++)
        {
            //d = b^(j-1)+((1/dt)*M-G)*q^(j-1)
            double prev_time = (j - 1) * dt;
            double[] b_prev = getB(j-1, prev_time);

            CsrMatrix DiffMG = CsrMatrix.Clone(M);
            M.MultiplyByNumber(1.0/dt, DiffMG);
            DiffMG.Diff(G, DiffMG);

            double[] DiffMGq = new double[q0.Length];
            DiffMG.MulVec(q_current, DiffMGq);

            double[] d = SpanAlgebra.SummVec(b_prev, DiffMGq);

            /*Получается слау A*q_next = d
             * Solver.MSG,LOS.. => get q_next*/
            q_current = q_next;
            Console.WriteLine($"Шаг No{j}");
            for (int i = 0; i < q_current.Length; i++)
                Console.Write($"{q_current[i]}" + " ");
            Console.WriteLine();
        }
        return q_current;
    }
   
} 

