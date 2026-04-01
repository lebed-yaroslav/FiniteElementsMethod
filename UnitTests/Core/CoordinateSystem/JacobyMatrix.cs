namespace UnitTests.Core.CoordinateSystem;

public class ConstantJacobyMatrixTests
{
    [Fact]
    public void DeterminantSpecialization()
    {
        var j = new ConstantJacobyMatrix1X1(new[,] { { 5.0 } });
        Assert.Equal(5.0, j.Det());
    }
}
