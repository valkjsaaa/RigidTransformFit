 using System;
 using MathNet.Numerics.LinearAlgebra;

namespace RigidTransformFit
{
    internal static class Program
    {
        private static void Main(string[] args)
        {
            var fromPoints = Matrix<double>.Build.DenseOfArray(
                new double[,] {{1, 1}, {1, 2}, {2, 2}, {2, 1}}
            );

            const double rotation = Math.PI / 6;
            var rotationMatrix = Matrix<double>.Build.DenseOfArray(
                new[,]
                {
                    {Math.Cos(rotation), -Math.Sin(rotation)},
                    {Math.Sin(rotation), Math.Cos(rotation)}
                }
            );
            var offset = Vector<double>.Build.DenseOfArray(
                new double[] {1, 1}
            );

            var toPoints = (fromPoints * rotationMatrix).MapIndexed((i, j, value) => value + offset[j]);

            var result = FitRigidTransform.Fit(fromPoints, toPoints);

            var r = result.Item1;
            var d = result.Item2;

            for (var i = 0; i < fromPoints.RowCount; i++)
            {
                Console.Out.WriteLine(r * fromPoints.Row(i) + d);
                Console.Out.WriteLine(toPoints.Row(i));
            }
        }
    }
}