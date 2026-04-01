

using Model.Model.Mesh;
using Telma.Extensions;

namespace Model.Model.Problem;


// TODO: EllipticSolver/ParabolicSolver/HyperbolicSolver

/// <summary>
/// Задача вида: χ(∂2u/∂t2)+σ(∂u/∂t)-∇⋅(λ∇u)=f
/// </summary>
public sealed record HyperbolicProblem<TSpace>
(
    Material<TSpace>[] Materials,
    BoundaryCondition<TSpace>[] BoundaryConditions,
    IMesh<TSpace> Mesh
) where TSpace : IVectorBase<TSpace>;


