using System;
using System.Collections.Generic;
using System.Text;
using MathTools;

namespace Pathfinding.VG
{
    public class AngleComparer : IComparer<int>
    {
        private double[] angles;
        private float[] squaredDists;
        private double tolerance;

        public AngleComparer(double tolerance = MathExt.AngleEpsilon)
        {
            this.tolerance = tolerance;
        }

        public void Reset(double[] angles, float[] squaredDists)
        {
            this.angles = angles;
            this.squaredDists = squaredDists;
        }

        // Primary comparison between angles, secondary between distances from p
        public int Compare(int id1, int id2)
        {
            if (id1 == id2)
                return 0;
            double angle1 = angles[id1];
            double angle2 = angles[id2];
            if (MathExt.Approximately(angle1, angle2, tolerance))
            {
                int compare = squaredDists[id1].CompareTo(squaredDists[id2]);
                // Sort non-poly points last on ties 
                // if rho_i intersects interior of poly then poly point will be invisible 
                // and so non-poly point should also be invisible (!prevVisible)
                if (compare == 0)
                    return id1 < id2 ? -1 : 1;
                return compare;
            }
            return angle1 < angle2 ? -1 : 1;
        }
    }
}
