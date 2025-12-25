using Telma;

namespace Model.Model.Basis;

public interface IBasisFunction
{
    double Value(Vector2D point);
    Vector2D Derivatives(Vector2D point);
}
