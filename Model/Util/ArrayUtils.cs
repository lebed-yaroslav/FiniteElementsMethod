using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Model.Util;


public static class ArrayUtils
{
    extension <T>(Array self)
    {
        public Span<T> AsFlatSpan() => MemoryMarshal.CreateSpan(
            ref Unsafe.As<byte, T>(ref MemoryMarshal.GetArrayDataReference(self)),
            self.Length
        );
    }
}
