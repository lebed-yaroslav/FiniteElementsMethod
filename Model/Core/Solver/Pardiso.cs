using System;
using System.Collections.Generic;
using System.Text;
using Quasar.Native;

namespace Model.Core.Solver
{
    //Предворительная идея инциализации

    //class TestInit 
    //{ 
    //    public PardisoSolver pardiso;
    //    public TestInit() 
    //    {

    //        var portrait = PardisoSolver.GenerateIAJA([]);
    //        pardiso = new PardisoSolver([], portrait.ia, portrait.ja);
    //        PardisoSolver.PSolve(pardiso!, [], []);
    //    }
    //}

    internal class PardisoSolver : IPardisoMatrix<double>
    {

        double[] A;
        int[] Ia;
        int[] Ja;
        public PardisoSolver(double[] a, int[] ia, int[] ja)
        {
            A = a;
            Ia = ia;
            Ja = ja;
        }

        public static void PSolve(PardisoSolver Matrix, Span<double> Res, ReadOnlySpan<double> RPart)
        {
            var solver = new Pardiso<double>(Matrix);
            solver.Analysis();
            solver.Factorization();
            solver.Solve(RPart, Res);
        }


        public PardisoMatrixType MatrixType => PardisoMatrixType.SymmetricIndefinite;

        public int n { get => ia.Length - 1; }

        public static (int[] ia, int[] ja) GenerateIAJA(IList<HashSet<int>> portrait)
        {
            var PSize = portrait.Count;
            var ig = new int[PSize + 1];
            ig[0] = 0;
            for (int i = 0; i < PSize; i++)
            {
                ig[i + 1] = ig[i] + 1;
                foreach (var l in portrait[i])
                {
                    if (l > i) ig[i + 1]++;
                }
            }
            var jg = new int[ig[PSize]];
            for (int i = 0; i < PSize; i++)
            {
                var jgaddr = ig[i];
                jg[jgaddr++] = i;
                foreach (var j in portrait[i].OrderBy(ll => ll))
                {
                    if (j > i) jg[jgaddr++] = j;
                }
            }
            return (ig, jg);
        }

        public ReadOnlySpan<double> a => A;
        public ReadOnlySpan<int> ia => Ia;
        public ReadOnlySpan<int> ja => Ja;
    }
}
