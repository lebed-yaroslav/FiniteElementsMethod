using Model.Core.CoordinateSystem;
using Telma;

namespace UnitTests.Core.CoordinateSystem;

public class ConstantJacobyMatrixTests
{
    [Fact]
    public void DeterminantSpecialization()
    {
        var j = new ConstantJacobyMatrix<Vector1D, Vector1D, MatrixOperations.Ops1X1>(new[,] {{ 5.0 }});
        Assert.Equal(5.0, j.Det());
    }
}
