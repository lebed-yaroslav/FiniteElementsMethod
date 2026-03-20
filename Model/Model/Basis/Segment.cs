using Telma;

namespace Model.Model.Basis;


public static class SegmentBasis
{
    /// <summary>N0(ξ) = 1 - ξ</summary>
    public static readonly IBasisFunction1D N0 = new SegmentN0();
    /// <summary>N1(ξ) = ξ</summary>
    public static readonly IBasisFunction1D N1 = new SegmentN1();
    /// <summary>N0N1(ξ) = ξ (1 - ξ)</summary>
    public static readonly IBasisFunction1D N0N1 = new SegmentN0N1();

    private readonly struct SegmentN0 : IBasisFunction1D
    {
        public double Value(Vector1D point) => 1 - point;
        public Vector1D Derivatives(Vector1D _) => -1;
    }

    private readonly struct SegmentN1 : IBasisFunction1D
    {
        public double Value(Vector1D point) => point;
        public Vector1D Derivatives(Vector1D _) => 1;
    }

    private readonly struct SegmentN0N1 : IBasisFunction1D
    {
        public double Value(Vector1D point) => point * (1 - point);
        public Vector1D Derivatives(Vector1D point) => 1 - 2 * point;
    }
}
