using System;
using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace RigidTransformFit
{
    public static class FitRigidTransform
    {
        /// <summary>
        /// Fit a rigid transform between 2 set of points using SVD
        /// https://www.ltu.se/cms_fs/1.51590!/svd-fitting.pdf
        /// The goal is to minimize |r * A + d - B|2
        /// </summary>
        /// <param name="pointsA"></param>
        /// <param name="pointsB"></param>
        public static Tuple<Matrix<double>, Vector<double>> Fit(Matrix<double> pointsA, Matrix<double> pointsB)
        {
            if (pointsA.RowCount != pointsB.RowCount ||
                pointsA.ColumnCount != pointsB.ColumnCount)
            {
                throw new InvalidParameterException();
            }

            var dims = pointsA.ColumnCount;
            
            var averageA = pointsA.ColumnSums() / pointsA.RowCount;
            var averageB = pointsB.ColumnSums() / pointsB.RowCount;

            var normalizedA = pointsA.MapIndexed((i, j, value) => value - averageA[j], Zeros.Include);
            var normalizedB = pointsB.MapIndexed((i, j, value) => value - averageB[j], Zeros.Include);

            var c = normalizedA.Transpose() * normalizedB;

            var usv = c.Svd();

            var newS = DenseMatrix.CreateIdentity(dims);
            newS[dims - 1, dims - 1] = (usv.U * usv.VT).Determinant();
            var r = (usv.U * newS * usv.VT).Transpose();

            var d = averageB - r * averageA;

            return Tuple.Create(r, d);
        }
    }
}