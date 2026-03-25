
using Model.Core.CoordinateSystem;

using Telma;
namespace QuadrangleTest
{
    public class QTest1
    {
        public bool AreVectorsEqual(Vector2D v1, Vector2D v2, double eps)
        {
            return (Math.Abs(v1.X - v2.X) < eps && Math.Abs(v1.Y - v2.Y) < eps);
        }

        [Fact]
        public void Единичный_Квадрат_В_единичный_квадрат()
        {
            var p1 = new Vector2D(0, 0);
            var p2 = new Vector2D(1, 0);
            var p3 = new Vector2D(0, 1);
            var p4 = new Vector2D(1, 1);

            var point = new QuadrangleCoordinateSystem(p1, p2, p3, p4);
            var p1_res = point.Transform(p1);
            var p2_res = point.Transform(p2);
            var p3_res = point.Transform(p3);
            var p4_res = point.Transform(p4);

            Assert.Equal(p1, p1_res);
            Assert.Equal(p2, p2_res);
            Assert.Equal(p3, p3_res);
            Assert.Equal(p4, p4_res);
        
        }

        [Fact]
        public void Произвольный_Квадрат_В_единичный_квадрат()
        {
            double eps = 1e-13;
            var tp1 = new Vector2D(0, 0);
            var tp2 = new Vector2D(1, 0);
            var tp3 = new Vector2D(0, 1);
            var tp4 = new Vector2D(1, 1);

            var p1 = new Vector2D(1, 5);
            var p2 = new Vector2D(6, 1);
            var p3 = new Vector2D(5, 6);
            var p4 = new Vector2D(7, 3);

            var point = new QuadrangleCoordinateSystem(p1, p2, p3, p4);
            var p1_res = point.Transform(p1);
            var p2_res = point.Transform(p2);
            var p3_res = point.Transform(p3);
            var p4_res = point.Transform(p4);

            Assert.True(AreVectorsEqual(tp1, p1_res, eps));
            Assert.True(AreVectorsEqual(tp2, p2_res, eps));
            Assert.True(AreVectorsEqual(tp3, p3_res, eps));
            Assert.True(AreVectorsEqual(tp4, p4_res, eps));

        }
    }
}
