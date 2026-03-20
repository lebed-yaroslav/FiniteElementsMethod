global using HyperbolicProblem1D = Model.Model.Problem.HyperbolicProblem<Telma.Vector1D>;
global using HyperbolicProblem2D = Model.Model.Problem.HyperbolicProblem<Telma.Vector2D>;
global using HyperbolicProblem3D = Model.Model.Problem.HyperbolicProblem<Telma.Vector3D>;

using Model.Model.Mesh;
using Telma.Extensions;

namespace Model.Model.Problem;


/// <summary>
/// Задача вида: χ(∂2u/∂t2)+σ(∂u/∂t)-∇⋅(λ∇u)=f
/// </summary>
public sealed record HyperbolicProblem<TSpace>
(
    Material<TSpace>[] Materials,
    BoundaryCondition<TSpace>[] BoundaryConditions,
    IMesh<TSpace> Mesh
) where TSpace : IVectorBase<TSpace>;
