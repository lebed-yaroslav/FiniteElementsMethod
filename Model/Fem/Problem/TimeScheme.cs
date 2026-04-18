using System.Diagnostics;

namespace Model.Fem.Problem;


public interface ITimeScheme
{
    int SolutionLayerCount { get; }

    double GetStiffnessScale(double dt);
    double GetMassScale(double dt);
    double GetSourceScale(double dt);

    void GetHistoryStiffnessCoefficients(double dt, Span<double> outCoeff);
    void GetHistoryMassCoefficients(double dt, Span<double> outCoeff);
}


public static class TimeSchemes
{
    /// <summary>
    /// Explicit two-layer scheme:
    /// <code>M * u_n = (dt)f_n - (dt)G * u_{n-1} + M * u_{n-1}</code>
    /// </summary>
    public static ITimeScheme ForwardEuler { get; } = new ForwardEulerScheme();

    /// <summary>
    /// Implicit two-layer scheme:
    /// <code>[(dt)G + M] * u_n = (dt)f_n + M * u_{n-1}</code>
    /// </summary>
    public static ITimeScheme BackwardEuler { get; } = new BackwardEulerScheme();

    /// <summary>
    /// Explicit three-layer scheme:
    /// <code>M * u_n = (2dt)f_n - (2dt)G * u_{n-1} + M * u_{n-2}</code>
    /// </summary>
    public static ITimeScheme ExplicitThreeLayer { get; } = new ExplicitThreeLayerScheme();

    /// <summary>
    /// Implicit three-layer scheme:
    /// <code>[(2dt)G + M] * u_n = (2dt)f_n + M * u_{n-2}</code>
    /// </summary>
    public static ITimeScheme ImplicitThreeLayer { get; } = new ImplicitThreeLayerScheme();


    private sealed class ForwardEulerScheme : ITimeScheme
    {
        public int SolutionLayerCount => 2;

        public double GetStiffnessScale(double dt) => 0.0;
        public double GetMassScale(double dt) => 1.0;
        public double GetSourceScale(double dt) => dt;

        public void GetHistoryStiffnessCoefficients(double dt, Span<double> outCoeff)
        {
            Debug.Assert(outCoeff.Length >= 1);
            outCoeff[0] = -dt;
        }

        public void GetHistoryMassCoefficients(double dt, Span<double> outCoeff)
        {
            Debug.Assert(outCoeff.Length >= 1);
            outCoeff[0] = 1.0;
        }
    }

    private sealed class BackwardEulerScheme : ITimeScheme
    {
        public int SolutionLayerCount => 2;

        public double GetStiffnessScale(double dt) => dt;
        public double GetMassScale(double dt) => 1.0;
        public double GetSourceScale(double dt) => dt;

        public void GetHistoryStiffnessCoefficients(double dt, Span<double> outCoeff)
        {
            Debug.Assert(outCoeff.Length >= 1);
            outCoeff[0] = 0.0;
        }

        public void GetHistoryMassCoefficients(double dt, Span<double> outCoeff)
        {
            Debug.Assert(outCoeff.Length >= 1);
            outCoeff[0] = 1.0;
        }
    }

    private sealed class ExplicitThreeLayerScheme : ITimeScheme
    {
        public int SolutionLayerCount => 3;

        public double GetStiffnessScale(double dt) => 0.0;
        public double GetMassScale(double dt) => 1.0;
        public double GetSourceScale(double dt) => 2 * dt;

        public void GetHistoryStiffnessCoefficients(double dt, Span<double> outCoeff)
        {
            Debug.Assert(outCoeff.Length >= 2);
            outCoeff[0] = -2 * dt; // u_{n-1}
            outCoeff[1] = 0.0; // u_{n-2}
        }

        public void GetHistoryMassCoefficients(double dt, Span<double> outCoeff)
        {
            Debug.Assert(outCoeff.Length >= 2);
            outCoeff[0] = 0.0; // u_{n-1}
            outCoeff[1] = 1.0; // u_{n-2}
        }
    }

    private sealed class ImplicitThreeLayerScheme : ITimeScheme
    {
        public int SolutionLayerCount => 3;

        public double GetStiffnessScale(double dt) => 2 * dt;
        public double GetMassScale(double dt) => 1.0;
        public double GetSourceScale(double dt) => 2 * dt;

        public void GetHistoryStiffnessCoefficients(double dt, Span<double> outCoeff)
        {
            Debug.Assert(outCoeff.Length >= 2);
            outCoeff.Clear();
        }

        public void GetHistoryMassCoefficients(double dt, Span<double> outCoeff)
        {
            Debug.Assert(outCoeff.Length >= 2);
            outCoeff[0] = 0.0; // u_{n-1}
            outCoeff[1] = 1.0; // u_{n-2}
        }
    }
}
