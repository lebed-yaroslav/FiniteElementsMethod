namespace Telma.Extensions;

public interface IDimensions
{
    abstract static int Count { get; }

    public sealed class D1 : IDimensions {
        public static int Count => 1;
    }

    public sealed class D2 : IDimensions {
        public static int Count => 2;
    }

    public sealed class D3 : IDimensions {
        public static int Count => 3;
    }
}
