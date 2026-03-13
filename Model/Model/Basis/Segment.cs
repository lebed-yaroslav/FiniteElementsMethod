using Telma;

namespace Model.Model.Basis;


public static class SegmentBasis
{
    /// <summary>N0(ξ) = 1 - ξ</summary>
    public static readonly IBasisFunction<Vector1D> N0 = new SegmentN0();
    /// <summary>N1(ξ) = ξ</summary>
    public static readonly IBasisFunction<Vector1D> N1 = new SegmentN1();

    private readonly struct SegmentN0 : IBasisFunction<Vector1D>
    {
        public double Value(Vector1D point) => 1 - point;
        public Vector1D Derivatives(Vector1D _) => -1;
    }

    private readonly struct SegmentN1 : IBasisFunction<Vector1D>
    {
        public double Value(Vector1D point) => point;
        public Vector1D Derivatives(Vector1D _) => 1;
    }
}
