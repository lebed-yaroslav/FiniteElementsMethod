namespace Model.Core.Solver.Precondition;


public sealed class IdentityPreconditioner : IPreconditioner
{
    public static IdentityPreconditioner Instance { get; } = new();
    private IdentityPreconditioner() { }

    public void Apply(ReadOnlySpan<double> input, Span<double> output)
        => input.CopyTo(output);
}
