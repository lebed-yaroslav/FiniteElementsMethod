using System.Runtime.CompilerServices;
using Telma;

namespace UnitTests;

public static class TestUtils
{
    extension(Assert)
    {
        [OverloadResolutionPriority(1)]
        public static void Equal(Vector1D expected, Vector1D actual, double epsilon)
        {
            Assert.Equal(expected.X, actual.X, epsilon);
        }

        [OverloadResolutionPriority(1)]
        public static void Equal(Vector2D expected, Vector2D actual, double epsilon)
        {
            Assert.Equal(expected.X, actual.X, epsilon);
            Assert.Equal(expected.Y, actual.Y, epsilon);
        }

        [OverloadResolutionPriority(1)]
        public static void Equal(Vector3D expected, Vector3D actual, double epsilon)
        {
            Assert.Equal(expected.X, actual.X, epsilon);
            Assert.Equal(expected.Y, actual.Y, epsilon);
            Assert.Equal(expected.Z, actual.Z, epsilon);
        }
    }
}
