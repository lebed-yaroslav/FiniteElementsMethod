using Model.Core.Matrix;
using Model.Fem.Basis;
using Telma;
using Model.Fem.Integrator;

namespace UnitTests.AnalyticalIntegrator
{
    public class AnalyticalIntegratorTests
    {
        [Fact]
        public void CorrectPolynomialSum()
        {
            //(x + 1) + (-x + 2y^2 + 3) = 2y^2 + 4
            var p1 = new Polynomial() { Summands = { [(1, 0)] = 1, [(0, 0)] = 1 } };
            var p2 = new Polynomial() { Summands = { [(1, 0)] = -1, [(0, 2)] = 2, [(0, 0)] = 3 } };

            var p3 = Polynomial.Sum(p1, p2);
            p1.Add(p2);

            Assert.Equal(2, p1.Summands.Count); //delete 0 value check
            Assert.Equal(2, p1.Summands[(0, 2)]); // 2y^2
            Assert.Equal(4, p1.Summands[(0, 0)]); // const
            Assert.Equal(2, p1.Degree); // degree

            Assert.Equal(2, p3.Summands.Count); //delete 0 value check
            Assert.Equal(2, p3.Summands[(0, 2)]); // 2y^2
            Assert.Equal(4, p3.Summands[(0, 0)]); // const
            Assert.Equal(2, p3.Degree); // degree
        }

        [Fact]
        public void CorrectPolynomialMult()
        {
            //(x + 1) * (-x + 2y^2 + 3) = -x^2 + 2xy^2 + 2x + 2y^2 + 4
            var p1 = new Polynomial() { Summands = { [(1, 0)] = 1, [(0, 0)] = 1 } };
            var p2 = new Polynomial() { Summands = { [(1, 0)] = -1, [(0, 2)] = 2, [(0, 0)] = 3 } };

            var p3 = Polynomial.Mult(p1, p2);
            p1.Mult(p2);

            Assert.Equal(5, p1.Summands.Count); 
            Assert.Equal(2, p1.Summands[(1, 2)]); // 2y^2
            Assert.Equal(2, p1.Summands[(1, 0)]); // 2x
            Assert.Equal(3, p1.Summands[(0, 0)]); // const
            Assert.Equal(3, p1.Degree); // degree

            Assert.Equal(5, p3.Summands.Count);
            Assert.Equal(2, p3.Summands[(1, 2)]); // 2y^2
            Assert.Equal(2, p3.Summands[(1, 0)]); // 2x
            Assert.Equal(3, p3.Summands[(0, 0)]); // const
            Assert.Equal(3, p3.Degree); // degree
        }

        [Fact]
        public void CorrectPolynomialGradient()
        {
            // d/dx (-x + 2y^2 + 3) = -1
            // d/dy (-x + 2y^2 + 3) = 4y
            var p2 = new Polynomial() { Summands = { [(1, 0)] = -1, [(0, 2)] = 2, [(0, 0)] = 3 } };
            var g = p2.Gradient();

            var dx = (IPolynomial)g[0];
            var dy = (IPolynomial)g[1];

            Assert.Equal(-1, dx.Summands[(0, 0)]);
            Assert.Single(dx.Summands);
            Assert.Equal(4, dy.Summands[(0, 1)]);
            Assert.Single(dy.Summands);
        }

        [Fact]
        public void CorrectPolynomialValue()
        {
            // (-x + 2y^2 + 3)|(1.5, 1) = 3.5
            var p2 = new Polynomial() { Summands = { [(1, 0)] = -1, [(0, 2)] = 2, [(0, 0)] = 3 } };
            var point = new Vector2D(1.5, 1.0);
            double res = p2.Value(point);

            Assert.Equal(3.5, res, 1e-13);
        }

        [Fact]
        public void CorrectPolynomialOnTriangle()
        {
            // Int((-x + 2y^2 + 3)*1) = 1.5
            var J = new Polynomial() { Summands = { [(0, 0)] = 1 } };
            var p2 = new Polynomial() { Summands = { [(1, 0)] = -1, [(0, 2)] = 2, [(0, 0)] = 3 } };
            double res = AnalyticalIntegration.IntegrateTriangle(p2, J);

            Assert.Equal(1.5, res, 1e-13);
        }

        [Fact]
        public void CorrectPolynomialOnQuadrangle()
        {
            // Int((-x + 2y^2 + 3)*1) = 19/6
            var J = new Polynomial() { Summands = { [(0, 0)] = 1 } };
            var p2 = new Polynomial() { Summands = { [(1, 0)] = -1, [(0, 2)] = 2, [(0, 0)] = 3 } };
            double res = AnalyticalIntegration.IntegrateQuadrangle(p2, J);

            Assert.Equal(19.0/6, res, 1e-13);
        }

        [Fact]
        public void CorrectPolynomialOnBound()
        {
            // Int((-x + 2y^2 + 3)*1) = 11/3
            var J = new Polynomial() { Summands = { [(0, 0)] = 3 } };
            var p2 = new Polynomial() { Summands = { [(1, 0)] = -1, [(0, 2)] = 2, [(0, 0)] = 3 } };
            var startPoint = new Vector2D(0.0, 0.0);
            var endPoint = new Vector2D(0.0, 1.0);
            double res = AnalyticalIntegration.IntegrateBoundary(p2, J, startPoint, endPoint);

            Assert.Equal(11.0, res, 1e-13);
        }
    }
}
