using System;
using System.Collections.Generic;
using System.IO;
using Xunit;
using Model.Core.CoordinateSystem;
using Model.Core.Matrix;
using Model.Core.Solver;
using Model.Core.Solver.Precondition;
using Model.Model.Elements;
using Model.Model.Mesh;
using Telma;

namespace UnitTests.Problem;

public class ElipticProblemQuadrangleHermiteTests
{
    private const double Eps = 1e-10;
    private const string квадратик_эталонный =
            """
    4
    0 0
    1 0
    1 1
    0 1
    1
    0 1 2 3 0
    4
    0 1 0
    1 2 0
    2 3 0
    3 0 0
    """;
    private const string сетка_на_6_элементов =
            """
    12
    0 0
    2 0
    4 0
    6 0
    0 1
    2 1
    4 1
    6 1
    0 2
    2 2
    4 2
    6 2
    6
    0 1 5 4 0
    1 2 6 5 0
    2 3 7 6 0
    4 5 9 8 0
    5 6 10 9 0
    6 7 11 10 0
    10
    0 1 0
    1 2 0
    2 3 0
    3 7 0
    7 11 0
    11 10 0
    10 9 0
    9 8 0
    8 4 0
    4 0 0
    """;
    private const string ромб_без_выреза =
            """
    8
    0 0
    1 0
    3 3
    3 4
    3 -4
    3 -3
    5 0
    6 0
    5
    0 1 2 3 0
    0 4 5 1 0
    2 6 7 3 0
    4 7 6 5 0
    1 5 6 2 0
    4
    0 4 0
    4 7 0
    7 3 0
    3 0 0
    
    """;
    private const string ромб_c_вырезом =
            """
    8
    0 0
    1 0
    3 3
    3 4
    3 -4
    3 -3
    5 0
    6 0
    4
    0 1 2 3 0
    0 4 5 1 0
    2 6 7 3 0
    4 7 6 5 0
    8
    0 4 0
    4 7 0
    7 3 0
    3 0 0
    1 5 0
    5 6 0
    6 2 0
    2 1 0
    """;
    private const string RefinedMesh =
            """
    24
    0 0 
    1 0
    3 3 
    3 4
    3 -4 
    3 -3
    5 0 
    6 0
    0.5 0
    2 1.5
    3 3.5
    1.5 2
    1.5 -2
    3 -3.5
    2 -1.5
    4 1.5
    5.5 0
    4.5 2
    4.5 -2
    4 -1.5
    1.75 1.75
    1.75 -1.75
    4.25 1.75
    4.25 -1.75
    16
    0 8 20 11 0
    8 1 9 20 0
    20 9 2 10 0
    11 20 10 3 0
    0 12 21 8 0
    12 4 13 21 0
    21 13 5 14 0
    8 21 14 1 0
    2 15 22 10 0
    15 6 16 22 0
    22 16 7 17 0
    10 22 17 3 0
    4 18 23 13 0
    18 7 16 23 0
    23 16 6 19 0
    13 23 19 5 0
    16
    0 12 0
    12 4 0
    4 18 0
    18 7 0
    7 17 0
    17 3 0
    3 11 0
    11 0 0
    1 14 0
    14 5 0
    5 19 0
    19 6 0
    6 15 0
    15 2 0
    2 9 0
    9 1 0
    """;
    private const string ТестИзПрошлого =
            """
    12
    0 0
    1 0
    1.5 0
    3.5 0
    0   0.1
    1   0.1
    1.5 0.1
    3.5 0.1
    0   1.1
    1   1.1
    1.5 1.1
    3.5 1.1
    6
    0 1 5 4 0
    1 2 6 5 0
    2 3 7 6 0
    4 5 9 8 0
    5 6 10 9 0
    6 7 11 10 0
    10
    0 1 0
    1 2 0
    2 3 0
    3 7 0
    7 11 0
    11 10 0
    10 9 0
    9 8 0
    8 4 0
    4 0 0
    """;
    private const string TestMesh7 =
            """
9
0 0
1 0
2 0
0 1
1 1
2 1
0 2
1 2
2 2
4
0 1 4 3 0
1 2 5 4 0
3 4 7 6 0
4 5 8 7 0
8
0 1 0
1 2 0
2 5 0
5 8 0
7 8 0
6 7 0
0 3 0
3 6 0
""";
    private const string одна_внутренняя_точка =
            """
        9
        0 0
        1 0
        2 0
        0 1
        1 1
        2 1
        0 2
        1 2
        2 2
        4
        0 1 4 3 0
        1 2 5 4 0
        3 4 7 6 0
        4 5 8 7 0
        8
        0 1 0
        1 2 0
        2 5 0
        5 8 0
        7 8 0
        6 7 0
        0 3 0
        3 6 0
        """;
    private const string две_внутренние_точки =
            """
        12
        0 0
        1 0
        2 0
        3 0
        0 1
        1 1
        2 1
        3 1
        0 2
        1 2
        2 2
        3 2
        6
        0 1 5 4 0
        1 2 6 5 0
        2 3 7 6 0
        4 5 9 8 0
        5 6 10 9 0
        6 7 11 10 0
        10
        0 1 0
        1 2 0
        2 3 0 
        3 7 0
        7 11 0
        11 10 0
        10 9 0
        9 8 0
        8 4 0
        4 0 0
        """;
    private const string четыре_внутренние_точки =
            """
        16
        0 0
        1 0
        2 0
        3 0
        0 1
        1 1
        2 1
        3 1
        0 2
        1 2
        2 2
        3 2
        0 3
        1 3
        2 3
        3 3
        9
        0 1 5 4 0
        1 2 6 5 0
        2 3 7 6 0
        4 5 9 8 0
        5 6 10 9 0
        6 7 11 10 0
        8 9 13 12 0
        9 10 14 13 0
        10 11 15 14 0
        12
        0 1 0
        1 2 0
        2 3 0 
        3 7 0
        7 11 0
        11 15 0
        15 14 0
        14 13 0
        13 12 0
        12 8 0
        8 4 0
        4 0 0
        """;
    private const string четыре_внутренние_точки_произвольный =
            """
        8
        0 0
        2 -1
        6 1
        1 1
        2 0.5
        3.1 1
        2 2.5
        2 6
        5
        0 1 4 3 0
        1 2 5 4 0
        2 7 6 5 0
        0 3 6 7 0
        3 4 5 6 0
        4
        0 1 0
        1 2 0
        2 7 0 
        7 0 0
        """;
    private const string большая_четырехугольная_сетка =
            """
        22
        0 0
        2 0.5
        5 0
        8 0.5
        0.5 1
        2.5 2
        5.5 1
        6.5 1
        0.5 4
        1 4.5
        5.5 3
        6.5 2
        6.7 2.5
        1.5 6
        2.5 7
        4 6
        4.4 5.5
        7.1 3.5
        4.5 6
        6 7
        6.5 5.5
        6.5 3
        15
        0 1 5 4 0
        1 2 6 5 0
        2 3 7 6 0
        3 12 11 7 0
        3 17 21 12 0
        17 21 10 20 0
        20 19 18 10 0
        19 14 15 18 0
        14 13 9 5 0
        4 5 9 8 0
        5 6 10 14 0
        6 7 11 10 0
        11 12 21 10 0
        14 10 16 15 0
        15 16 10 18 0
        12
        0 1 0
        1 2 0
        2 3 0
        3 17 0
        17 20 0
        20 19 0
        19 14 0
        14 13 0
        13 9 0
        9 8 0
        8 4 0
        4 0 0
        """;
    [Fact] public void Test_Квадратик_Эталонный() => RunTestForMesh(квадратик_эталонный);
    [Fact] public void Test_Сетка_На_6_Элементов() => RunTestForMesh(сетка_на_6_элементов);
    [Fact] public void Test_Ромб_Без_Выреза() => RunTestForMesh(ромб_без_выреза);
    [Fact] public void Test_Ромб_С_Вырезом() => RunTestForMesh(ромб_c_вырезом);
    [Fact] public void Test_Ромб_С_Дроблением_Сетки_Без_Выреза() => RunTestForMesh(RefinedMesh);
    [Fact] public void Test_Неравномерная_Квадратная_Сетка() => RunTestForMesh(ТестИзПрошлого);
    [Fact] public void Test_Mesh7() => RunTestForMesh(TestMesh7);
    [Fact] public void Test_Одна_Внутренняя_Точка() => RunTestForMesh(одна_внутренняя_точка);
    [Fact] public void Test_Две_Внутренние_Точки() => RunTestForMesh(две_внутренние_точки);
    [Fact] public void Test_Четыре_Внутренние_Точки() => RunTestForMesh(четыре_внутренние_точки);
    [Fact] public void Test_Четыре_Внутренние_Точки_Произвольный() => RunTestForMesh(четыре_внутренние_точки_произвольный);
    [Fact] public void Test_Большая_Четырехугольная_Сетка() => RunTestForMesh(большая_четырехугольная_сетка);

    private void RunTestForMesh(string meshText)
    {
        var mesh = new StringReader(meshText).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Quadrangle.Hermite,
            FiniteElements.Segment.Hermite
        );

        var problem = new EllipticProblem2D(
            Materials: [
                new(Lambda: _ => 1.0, Gamma: _ => 0, Source: _ => 0)
            ],
            BoundaryConditions: [
                new BoundaryCondition2D.Dirichlet((p, _) => p.X)
            ],
            mesh
        );

        var solver = new EllipticSolver2D(
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(problem, new ISolver.Params(1e-15, 10000));

        var vertexSolution = new double[mesh.VertexCount];
        foreach (var element in mesh.AllElements)
        {
            int dofPerVertex = element.DOF.NumberOfDofOnVertex;
            for (int i = 0; i < element.Geometry.VertexCount; i++)
            {
                int vertexId = element.Geometry.Vertices[i];
                int dofIndex = element.DOF.Dof[i * dofPerVertex];
                vertexSolution[vertexId] = solution.Coefficients[dofIndex];
            }
        }

        for (int i = 0; i < mesh.VertexCount; i++)
        {
            var p = mesh[i];
            double exact = p.X;
            double error = Math.Abs(vertexSolution[i] - exact);

            Assert.True(
                error < Eps,
                $"Error too high at Node {i} {p}: FEM={vertexSolution[i]}, Exact={exact}, Error={error}"
            );
        }
    }
}
