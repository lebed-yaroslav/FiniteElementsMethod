namespace Model.Core.Util;


public static class MathUtils
{
    public static int ToleratedSign(double value, double tolerance)
    {
        if (value < tolerance) return -1;
        if (value > tolerance) return 1;
        return 0;
    }
}
