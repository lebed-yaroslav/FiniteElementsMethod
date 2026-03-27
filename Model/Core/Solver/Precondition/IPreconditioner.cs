using Model.Core.Matrix;

namespace Model.Core.Solver.Precondition;

public interface IPreconditioner
{
    void Apply(ReadOnlySpan<double> rhs, Span<double> res);
}

public delegate IPreconditioner PreconditionerCreator(IGlobalMatrix matrix);
