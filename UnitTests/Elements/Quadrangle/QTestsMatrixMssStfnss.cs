using Model.Core.CoordinateSystem;
using Model.Model;
using Model.Model.Elements;
using Model.Model.Mesh;
using Telma;

namespace UnitTests.Elements.Quadrangle;
public class QTestBicubicLagrange
{
    public bool AreVectorsEqual(Vector2D v1, Vector2D v2, double eps)
    {
        return (Math.Abs(v1.X - v2.X) < eps && Math.Abs(v1.Y - v2.Y) < eps);
    }

    public Mesh2D CreateUnitMesh() {
        var mesh = new Mesh2D(IdentityTransform<Vector2D>.Instance);
        mesh.AddVertex(new(2, 3));
        mesh.AddVertex(new(4, 1));
        mesh.AddVertex(new(8, 5));
        mesh.AddVertex(new(3, 9));

        mesh.AddElement(FiniteElements.Quadrangle.LagrangeCubic, [0, 1, 2, 3], materialIndex: 0);

        return mesh;
    }

    public Mesh2D CreateUnitMesh2Elements()
    {
        var mesh = new Mesh2D(IdentityTransform<Vector2D>.Instance);
        //mesh.AddVertex(new(2, 2));
        //mesh.AddVertex(new(7, 2));
        //mesh.AddVertex(new(8, 5));
        //mesh.AddVertex(new(3, 5));
        //mesh.AddVertex(new(11, 2));
        //mesh.AddVertex(new(12, 5));

        //mesh.AddElement(FiniteElements.Quadrangle.LagrangeCubic, [0, 1, 2, 3], materialIndex: 0);
        //mesh.AddElement(FiniteElements.Quadrangle.LagrangeCubic, [1, 4, 5, 2], materialIndex: 0);


        //mesh.AddVertex(new(2, 2));
        //mesh.AddVertex(new(7, 2));
        //mesh.AddVertex(new(8, 5));
        //mesh.AddVertex(new(3, 5));
        //mesh.AddVertex(new(9, 2));
        //mesh.AddVertex(new(13, 2));
        //mesh.AddVertex(new(14, 5));
        //mesh.AddVertex(new(10, 5));

        //mesh.AddElement(FiniteElements.Quadrangle.LagrangeCubic, [0, 1, 2, 3], materialIndex: 0);
        //mesh.AddElement(FiniteElements.Quadrangle.LagrangeCubic, [4, 5, 6, 7], materialIndex: 0);


        mesh.AddVertex(new(2, 2));
        mesh.AddVertex(new(5, 2));
        mesh.AddVertex(new(5, 5));
        mesh.AddVertex(new(2, 5));
        mesh.AddVertex(new(8, 2));
        mesh.AddVertex(new(8, 5));

        mesh.AddElement(FiniteElements.Quadrangle.LagrangeCubic, [0, 1, 2, 3], materialIndex: 0);
        mesh.AddElement(FiniteElements.Quadrangle.LagrangeCubic, [1, 4, 5, 2], materialIndex: 0);

        return mesh;
    }


    [Fact]
    public void check_diagonalpositivity_of_stiffness_matrix_on_quadrangle()
    {
        var mesh = CreateUnitMesh();

        var integrator = NumericItegrator2D.Instance;
        var stiffness = integrator.CalculateLocalStiffness(mesh.FiniteElements.First(), _ => 1.0);

        for (int i = 0; i < stiffness.Size; i++)
        {
            Assert.True(stiffness[i, i] > 0.0, $"Диагональный элемент M[{i},{i}] = {stiffness[i, i]} не положительный");
        }
    }

    [Fact]
    public void check_symmetry_of_stiffness_matrix_on_quadrangle()
    {
        var mesh = CreateUnitMesh();
        const double eps = 1e-12;

        var integrator = NumericItegrator2D.Instance;
        var stiffness = integrator.CalculateLocalStiffness(mesh.FiniteElements.First(), _ => 1.0);

        for (int i = 0; i < stiffness.Size; i++)
        {
            for(int j = i; j < stiffness.Size; j++)
            {
                double elU = stiffness[i, j];
                double elB = stiffness[j, i];
                Assert.True(Math.Abs(elU - elB) < eps, $"Матрица жескости не симметрична: M[{i},{j}] = {elU}, M[{j},{i}] = {elB}");
            }
        }
    }

    [Fact]
    public void check_diagonalpositivity_of_mass_matrix_on_quadrangle()
    {
        var mesh = CreateUnitMesh();

        var integrator = NumericItegrator2D.Instance;
        var mass = integrator.CalculateLocalStiffness(mesh.FiniteElements.First(), _ => 1.0);

        for (int i = 0; i < mass.Size; i++)
        {
            Assert.True(mass[i, i] > 0.0, $"Диагональный элемент M[{i},{i}] = {mass[i, i]} не положительный");
        }
    }

    [Fact]
    public void check_symmetry_of_mass_matrix_on_quadrangle()
    {
        var mesh = CreateUnitMesh();
        const double eps = 1e-12;

        var integrator = NumericItegrator2D.Instance;
        var mass = integrator.CalculateLocalStiffness(mesh.FiniteElements.First(), _ => 1.0);

        for (int i = 0; i < mass.Size; i++)
        {
            for (int j = i; j < mass.Size; j++)
            {
                double elU = mass[i, j];
                double elB = mass[j, i];
                Assert.True(Math.Abs(elU - elB) < eps, $"Матрица масс не симметрична: M[{i},{j}] = {elU}, M[{j},{i}] = {elB}");
            }
        }
    }

    [Fact]
    public void check_TwoElements_TotalMassIsArea()
    {
        var mesh = CreateUnitMesh2Elements();
        double totalMass = 0.0;
        double gammaConst = 10.0;

        var integrator = NumericItegrator2D.Instance;
        var mass1 = integrator.CalculateLocalMass(mesh.FiniteElements.ElementAt(0), _ => gammaConst);
        var mass2 = integrator.CalculateLocalMass(mesh.FiniteElements.ElementAt(1), _ => gammaConst);
        

        for (int i = 0; i < mass1.Size; i++)
        {
            for (int j = 0; j < mass1.Size; j++)
            {
                totalMass += mass1[i, j];
                totalMass += mass2[i, j];
            }
        }

        //double expectedTotal = 27.0 * gammaConst;
        double expectedTotal = 18.0 * gammaConst;

        Assert.InRange(
            totalMass,
            expectedTotal * 0.99,
            expectedTotal * 1.01
        );
    }
}


