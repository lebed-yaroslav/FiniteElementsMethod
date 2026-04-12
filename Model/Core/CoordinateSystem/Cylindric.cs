using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using Telma;
using Telma.Extensions;

namespace Model.Core.CoordinateSystem
{
    public sealed class CylindricCoordinateSystem : ICoordinateTransform<Vector2D, Vector2D>
    {
        public static readonly CylindricCoordinateSystem Instance = new();
        private CylindricCoordinateSystem() { }

        public static bool IsLinear => false;

        public Vector2D Transform(Vector2D sourcePoint)
            => new(sourcePoint.Norm, Math.Atan2(sourcePoint.Y, sourcePoint.X));

        public Vector2D InverseTransform(Vector2D targetPoint)
        {
            (var r, var theta) = targetPoint;
            return new(r * Math.Cos(theta), r * Math.Sin(theta));
        }

        public double Jacobian(Vector2D targetPoint) => targetPoint.X; // r

        public IJacobyMatrix<Vector2D, Vector2D> InverseJacoby()
            => InverseCylindricJacobyMatrix<Vector2D>.Instance;
    }

    public sealed class InverseCylindricJacobyMatrix<TSource> : IJacobyMatrix<TSource, TSource>
    where TSource : IVectorBase<TSource>
    {
        public static InverseCylindricJacobyMatrix<TSource> Instance { get; } = new();
        private InverseCylindricJacobyMatrix() { }

        public static bool IsConstant => true;
        public double this[int i, int j]
        {
            get
            {
                Debug.Assert(0 <= i && i < TSource.Dimensions);
                Debug.Assert(0 <= j && j < TSource.Dimensions);
                return (i == j) ? 1 : 0;
            }
        }

        public double this[int i, int j, TSource _] => this[i, j];

        public double Det(TSource _) => 1.0;
    }
}


