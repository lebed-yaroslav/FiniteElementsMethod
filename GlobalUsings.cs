// Matrices

global using IJacobyMatrix1X1 = Model.Core.CoordinateSystem.IJacobyMatrix<Telma.Vector1D, Telma.Vector1D>;
global using IJacobyMatrix2X2 = Model.Core.CoordinateSystem.IJacobyMatrix<Telma.Vector2D, Telma.Vector2D>;
global using IJacobyMatrix3X3 = Model.Core.CoordinateSystem.IJacobyMatrix<Telma.Vector3D, Telma.Vector3D>;
global using IJacobyMatrix1X2 = Model.Core.CoordinateSystem.IJacobyMatrix<Telma.Vector1D, Telma.Vector2D>;
global using IJacobyMatrix2X3 = Model.Core.CoordinateSystem.IJacobyMatrix<Telma.Vector2D, Telma.Vector3D>;

global using ConstantJacobyMatrix1X1 = Model.Core.CoordinateSystem.ConstantJacobyMatrix<
    Telma.Vector1D, Telma.Vector1D, Model.Core.CoordinateSystem.MatrixOperations.Ops1X1
>;
global using ConstantJacobyMatrix2X2 = Model.Core.CoordinateSystem.ConstantJacobyMatrix<
    Telma.Vector2D, Telma.Vector2D, Model.Core.CoordinateSystem.MatrixOperations.Ops2X2
>;
global using ConstantJacobyMatrix3X3 = Model.Core.CoordinateSystem.ConstantJacobyMatrix<
    Telma.Vector3D, Telma.Vector3D, Model.Core.CoordinateSystem.MatrixOperations.Ops3X3
>;

global using ConstantJacobyMatrix1X2 = Model.Core.CoordinateSystem.ConstantJacobyMatrix<
    Telma.Vector1D, Telma.Vector2D, Model.Core.CoordinateSystem.MatrixOperations.Ops1X2
>;
global using ConstantJacobyMatrix2X3 = Model.Core.CoordinateSystem.ConstantJacobyMatrix<
    Telma.Vector2D, Telma.Vector3D, Model.Core.CoordinateSystem.MatrixOperations.Ops2X3
>;

// Basis Set

global using IBasisSet1D = Model.Fem.Elements.IBasisSet<Telma.Vector1D>;
global using IBasisSet2D = Model.Fem.Elements.IBasisSet<Telma.Vector2D>;
global using IBasisSet3D = Model.Fem.Elements.IBasisSet<Telma.Vector3D>;

global using BasisSet1D = Model.Fem.Elements.BasisSet<Telma.Vector1D>;
global using BasisSet2D = Model.Fem.Elements.BasisSet<Telma.Vector2D>;
global using BasisSet3D = Model.Fem.Elements.BasisSet<Telma.Vector3D>;

// Finite Elements

global using IFiniteElement1D = Model.Fem.Elements.IFiniteElement<Telma.Vector1D>;
global using IFiniteElement2D = Model.Fem.Elements.IFiniteElement<Telma.Vector2D>;
global using IFiniteElement3D = Model.Fem.Elements.IFiniteElement<Telma.Vector3D>;
global using FiniteElement1D = Model.Fem.Elements.VolumeElement<Telma.Vector1D>;
global using FiniteElement2D = Model.Fem.Elements.VolumeElement<Telma.Vector2D>;
global using FiniteElement3D = Model.Fem.Elements.VolumeElement<Telma.Vector3D>;

global using IFiniteElementFactory1D = Model.Fem.Elements.IFiniteElementFactory<Telma.Vector1D>;
global using IFiniteElementFactory2D = Model.Fem.Elements.IFiniteElementFactory<Telma.Vector2D>;
global using IFiniteElementFactory3D = Model.Fem.Elements.IFiniteElementFactory<Telma.Vector3D>;

global using IBoundaryElement2D = Model.Fem.Elements.IBoundaryElement<Telma.Vector2D, Telma.Vector1D>;
global using IBoundaryElement3D = Model.Fem.Elements.IBoundaryElement<Telma.Vector3D, Telma.Vector2D>;
global using BoundaryElement2D = Model.Fem.Elements.FiniteElement<Telma.Vector2D, Telma.Vector1D>;
global using BoundaryElement3D = Model.Fem.Elements.FiniteElement<Telma.Vector3D, Telma.Vector2D>;

global using IBoundaryElementFactory2D = Model.Fem.Elements.IBoundaryElementFactory<Telma.Vector2D, Telma.Vector1D>;
global using IBoundaryElementFactory3D = Model.Fem.Elements.IBoundaryElementFactory<Telma.Vector3D, Telma.Vector2D>;

// Geometry

global using IElementGeometry1D = Model.Fem.Elements.IElementGeometry<Telma.Vector1D>;
global using IElementGeometry2D = Model.Fem.Elements.IElementGeometry<Telma.Vector2D>;
global using IElementGeometry3D = Model.Fem.Elements.IElementGeometry<Telma.Vector3D>;
global using VolumeElementGeometry1D = Model.Fem.Elements.ElementGeometry<Telma.Vector1D, Telma.Vector1D>;
global using VolumeElementGeometry2D = Model.Fem.Elements.ElementGeometry<Telma.Vector2D, Telma.Vector2D>;
global using VolumeElementGeometry3D = Model.Fem.Elements.ElementGeometry<Telma.Vector3D, Telma.Vector3D>;

global using BoundaryElementGeometry2D = Model.Fem.Elements.ElementGeometry<Telma.Vector2D, Telma.Vector1D>;
global using BoundaryElementGeometry3D = Model.Fem.Elements.ElementGeometry<Telma.Vector3D, Telma.Vector2D>;

// Mesh

global using IMesh1D = Model.Fem.Mesh.IMesh<Telma.Vector1D>;
global using IMesh2D = Model.Fem.Mesh.IMesh<Telma.Vector2D>;
global using IMesh3D = Model.Fem.Mesh.IMesh<Telma.Vector3D>;

global using Mesh2D = Model.Fem.Mesh.Mesh<Telma.Vector2D, Telma.Vector1D>;
global using Mesh3D = Model.Fem.Mesh.Mesh<Telma.Vector3D, Telma.Vector2D>;

// Integrator

global using IItegrator2D = Model.Fem.Integrator.IIntegrator<Telma.Vector2D, Telma.Vector1D, Model.Core.CoordinateSystem.MatrixOperations.Ops2X2>;
global using IItegrator3D = Model.Fem.Integrator.IIntegrator<Telma.Vector3D, Telma.Vector2D, Model.Core.CoordinateSystem.MatrixOperations.Ops3X3>;

global using NumericItegrator2D = Model.Fem.Integrator.NumericIntegrator<Telma.Vector2D, Telma.Vector1D, Model.Core.CoordinateSystem.MatrixOperations.Ops2X2>;
global using NumbericItegrator3D = Model.Fem.Integrator.NumericIntegrator<Telma.Vector3D, Telma.Vector2D, Model.Core.CoordinateSystem.MatrixOperations.Ops3X3>;

// Problem-Related

global using BoundaryCondition1D = Model.Fem.Problem.BoundaryCondition<Telma.Vector1D>;
global using BoundaryCondition2D = Model.Fem.Problem.BoundaryCondition<Telma.Vector2D>;
global using BoundaryCondition3D = Model.Fem.Problem.BoundaryCondition<Telma.Vector3D>;

global using EllipticMaterial1D = Model.Fem.Problem.HyperbolicMaterial<Telma.Vector1D>;
global using EllipticMaterial2D = Model.Fem.Problem.HyperbolicMaterial<Telma.Vector2D>;
global using EllipticMaterial3D = Model.Fem.Problem.HyperbolicMaterial<Telma.Vector3D>;

global using HyperbolicMaterial1D = Model.Fem.Problem.HyperbolicMaterial<Telma.Vector1D>;
global using HyperbolicMaterial2D = Model.Fem.Problem.HyperbolicMaterial<Telma.Vector2D>;
global using HyperbolicMaterial3D = Model.Fem.Problem.HyperbolicMaterial<Telma.Vector3D>;

global using HyperbolicProblem1D = Model.Fem.Problem.HyperbolicProblem<Telma.Vector1D>;
global using HyperbolicProblem2D = Model.Fem.Problem.HyperbolicProblem<Telma.Vector2D>;
global using HyperbolicProblem3D = Model.Fem.Problem.HyperbolicProblem<Telma.Vector3D>;

global using EllipticProblem1D = Model.Fem.Problem.EllipticProblem<Telma.Vector1D>;
global using EllipticProblem2D = Model.Fem.Problem.EllipticProblem<Telma.Vector2D>;
global using EllipticProblem3D = Model.Fem.Problem.EllipticProblem<Telma.Vector3D>;

global using EllipticSolver2D = Model.Fem.Problem.EllipticSolver<Telma.Vector2D, Telma.Vector1D, Model.Core.CoordinateSystem.MatrixOperations.Ops2X2>;
global using EllipticSolver3D = Model.Fem.Problem.EllipticSolver<Telma.Vector3D, Telma.Vector2D, Model.Core.CoordinateSystem.MatrixOperations.Ops3X3>;

global using ParabolicSolver2D = Model.Fem.Problem.ParabolicSolver<Telma.Vector2D, Telma.Vector1D, Model.Core.CoordinateSystem.MatrixOperations.Ops2X2>;
global using ParabolicSolver3D = Model.Fem.Problem.ParabolicSolver<Telma.Vector3D, Telma.Vector2D, Model.Core.CoordinateSystem.MatrixOperations.Ops3X3>;
// Assembler

global using Assembler2D = Model.Fem.Assembly.Assembler<Telma.Vector2D, Telma.Vector1D, Model.Core.CoordinateSystem.MatrixOperations.Ops2X2>;
global using Assembler3D = Model.Fem.Assembly.Assembler<Telma.Vector3D, Telma.Vector2D, Model.Core.CoordinateSystem.MatrixOperations.Ops3X3>;
