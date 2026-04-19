using System.Diagnostics;

namespace Model.Fem.Problem;

/// <summary>
/// Time scheme of type:
/// <code>
/// ∑ [i=0,k | ((αi)G + (βi)M + (ζi)MS3)⋅u_{n-i} + (γi)⋅f_{n-i}] = 0
/// </code>
/// where:
/// <list type="bullet">
/// <item>k - <see cref="Layers"/></item>
/// <item>α - <see cref="GetStiffnessScale"/></item>
/// <item>β - <see cref="GetMassScale"/></item>
/// <item>ζ - <see cref="GetRobinMassScale"/></item>
/// <item>γ - <see cref="GetSourceScale"/></item>
/// </list>
/// </summary>
public interface ITimeScheme
{   
    int Layers { get; }

    void GetStiffnessScale(ReadOnlySpan<double> t, Span<double> outAlpha);
    void GetMassScale(ReadOnlySpan<double> t, Span<double> outBeta);
    void GetSourceScale(ReadOnlySpan<double> t, Span<double> outGamma);
}


public static class TimeSchemes
{
    /// <summary>
    /// <code>[M + MS3]⋅u_n = (dt)b_{n-1} + [-(dt)G + M]⋅u_{n-1}</code>
    /// </summary>
    /// <remarks>Formula [1, 7.12]</remarks>
    public static ITimeScheme ExplicitTwoLayers { get; } = new ForwardEulerScheme();

    /// <summary>
    /// <code>[2M + (dt)G + (dt)MS3]⋅u_n = dt⋅[b_n + b_{n-1}] + [2M - (dt)G - (dt)MS3]⋅u_{n-1}</code>
    /// </summary>
    /// <remarks>Formula [1, 7.13]</remarks>
    public static ITimeScheme ImplicitTwoLayers { get; } = new BackwardEulerScheme();

    /// <summary>
    /// <code>[dt1/(dt⋅dt0)M + MS3]⋅u_n = b_{n-1} + [-G + (dt+dt0)/(dt⋅dt0)M]⋅u_{n-1} + (dt0)/(dt⋅dt1)M⋅u_{n-2}</code>
    /// </summary>
    public static ITimeScheme ExplicitThreeLayer { get; } = new ExplicitThreeLayerScheme();

    /// <summary>
    /// <code>[G + (dt+dt0)/(dt⋅dt0)M + MS3]⋅u_n = b_n + (dt)/(dt1⋅dt0)M⋅u_{n-1} - (dt0)/(dt⋅dt1)M⋅u_{n-2}</code>
    /// </summary>
    /// <remarks>Formula [1, 7.24]</remarks>
    public static ITimeScheme ImplicitThreeLayer { get; } = new ImplicitThreeLayerScheme();


    /// <summary>
    /// <code>[M + MS3]⋅u_n = (dt)b_{n-1} + [-(dt)G + M]⋅u_{n-1}</code>
    /// </summary>
    /// <remarks>Formula [1, 7.12]</remarks>
    private sealed class ForwardEulerScheme : ITimeScheme
    {
        public int Layers => 2;

        public void GetStiffnessScale(ReadOnlySpan<double> t, Span<double> outAlpha)
        {
            outAlpha[0] = 0.0;
            outAlpha[1] = t[1] - t[0];
        }

        public void GetMassScale(ReadOnlySpan<double> t, Span<double> outBeta)
        {
            outBeta[0] = 1.0;
            outBeta[1] = -1.0;
        }

        public void GetSourceScale(ReadOnlySpan<double> t, Span<double> outGamma)
        {
            outGamma[0] = 0.0;
            outGamma[1] = -(t[1] - t[0]);
        }
    }

    /// <summary>
    /// <code>[2M + (dt)G + (dt)MS3]⋅u_n = dt⋅[b_n + b_{n-1}] + [2M - (dt)G - (dt)MS3]⋅u_{n-1}</code>
    /// </summary>
    /// <remarks>Formula [1, 7.13]</remarks>
    private sealed class BackwardEulerScheme : ITimeScheme
    {
        public int Layers => 2;

        public void GetStiffnessScale(ReadOnlySpan<double> t, Span<double> outAlpha)
        {
            var dt = t[1] - t[0];
            outAlpha[0] = dt;
            outAlpha[1] = dt;
        }

        public void GetMassScale(ReadOnlySpan<double> t, Span<double> outBeta)
        {
            outBeta[0] = 2.0;
            outBeta[1] = -2.0;
        }

        public void GetSourceScale(ReadOnlySpan<double> t, Span<double> outGamma)
        {
            var dt = t[1] - t[0];
            outGamma[0] = -dt;
            outGamma[1] = -dt;
        }
    }

    /// <summary>
    /// <code>[dt1/(dt⋅dt0)M]⋅u_n = b_{n-1} + [-G - MS3 + (dt1 - dt0)/(dt1⋅dt0)M]⋅u_{n-1} + (dt0)/(dt⋅dt1)M⋅u_{n-2}</code>
    /// </summary>
    private sealed class ExplicitThreeLayerScheme : ITimeScheme
    {
        public int Layers => 3;

        private static (double dt, double dt0, double dt1) GetTimeSteps(ReadOnlySpan<double> t)
            => (
                dt: t[2] - t[0],
                dt0: t[2] - t[1],
                dt1: t[1] - t[0]
            );

        public void GetStiffnessScale(ReadOnlySpan<double> t, Span<double> outAlpha)
        {
            outAlpha[0] = 0.0;
            outAlpha[1] = 1.0;
            outAlpha[2] = 0.0;
        }

        public void GetMassScale(ReadOnlySpan<double> t, Span<double> outBeta)
        {
            (var dt, var dt0, var dt1) = GetTimeSteps(t);
            outBeta[0] = dt1 / (dt * dt0);
            outBeta[1] = -(dt1 - dt0) / (dt1 * dt0);
            outBeta[2] = -dt0 / (dt * dt1);
        }

        public void GetSourceScale(ReadOnlySpan<double> t, Span<double> outGamma)
        {
            outGamma[0] = -1.0;
            outGamma[1] = 0.0;
            outGamma[2] = 0.0;
        }
    }

    /// <summary>
    /// <code>[G + (dt+dt0)/(dt⋅dt0)M + MS3]⋅u_n = b_n + (dt)/(dt1⋅dt0)M⋅u_{n-1} - (dt0)/(dt⋅dt1)M⋅u_{n-2}</code>
    /// </summary>
    /// <remarks>Formula [1, 7.24]</remarks>
    private sealed class ImplicitThreeLayerScheme : ITimeScheme
    {
        public int Layers => 3;

        private static (double dt, double dt0, double dt1) GetTimeSteps(ReadOnlySpan<double> t) 
            => (
                dt: t[2] - t[0],
                dt0: t[2] - t[1],
                dt1: t[1] - t[0]
            );

        public void GetStiffnessScale(ReadOnlySpan<double> t, Span<double> outAlpha)
        {
            outAlpha[0] = 1.0;
            outAlpha[1] = 0.0;
            outAlpha[2] = 0.0;
        }

        public void GetMassScale(ReadOnlySpan<double> t, Span<double> outBeta)
        {
            (var dt, var dt0, var dt1) = GetTimeSteps(t);
            outBeta[0] = (dt + dt0) / (dt * dt0);
            outBeta[1] = -dt / (dt1 * dt0);
            outBeta[2] = dt0 / (dt * dt1);
        }

        public void GetSourceScale(ReadOnlySpan<double> t, Span<double> outGamma)
        {
            outGamma[0] = -1.0;
            outGamma[1] = 0.0;
            outGamma[2] = 0.0;
        }
    }
}
