using Telma;
using IJacobyMatrix2D = Model.Core.CoordinateSystem.IJacobyMatrix<Telma.Vector2D, Telma.Vector2D>;

namespace Model.Core.CoordinateSystem;

public static class JacobyMatrix2D
{
    extension(IJacobyMatrix2D self)
    {
        public IJacobyMatrix2D MulAt(Vector2D lp, IJacobyMatrix2D rhs, Vector2D rp) 
            => new ConstantJacobyMatrix2D(new[,] {
                {
                    self[0, 0, lp] * rhs[0, 0, rp] + self[0, 1, lp] * rhs[1, 0, rp],
                    self[0, 0, lp] * rhs[0, 1, rp] + self[0, 1, lp] * rhs[1, 1, rp]
                },
                {
                    self[1, 0, lp] * rhs[0, 0, rp] + self[1, 1, lp] * rhs[1, 0, rp],
                    self[1, 0, lp] * rhs[0, 1, rp] + self[1, 1, lp] * rhs[1, 1, rp]
                }
            });
    }
}
