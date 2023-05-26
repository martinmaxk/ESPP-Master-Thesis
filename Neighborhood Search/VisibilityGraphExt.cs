using MathTools;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace Pathfinding.VG
{
    public static class VisibilityGraphExt
    {
        public static bool HasFlag(this VisTriOrientation flags, VisTriOrientation flag)
        {
            return (flags & flag) == flag;
        }

        private static void GetTriAnglePoints(VisVertices visVertices, VisTriEdge triEdgeCw,
            VisTriEdge triEdgeCcw, out Vector2 anglePCw, out Vector2 anglePCcw)
        {
            int closestVIdCw = triEdgeCw.GetClosestVId();
            int closestVIdCcw = triEdgeCcw.GetClosestVId();
            anglePCw = visVertices.V(closestVIdCw);
            anglePCcw = visVertices.V(closestVIdCcw);
            bool edgeOnCwSide = triEdgeCcw.intersection == anglePCcw || triEdgeCcw.IsEdgeOnCwSide();
            bool edgeOnCcwSide = triEdgeCw.intersection == anglePCw || !triEdgeCw.IsEdgeOnCwSide();
            // Incident edges
            if (edgeOnCwSide && edgeOnCcwSide)
                return;
            if (edgeOnCwSide || !edgeOnCcwSide)
                anglePCw = triEdgeCw.intersection;
            if (edgeOnCcwSide || !edgeOnCwSide)
                anglePCcw = triEdgeCcw.intersection;
        }

        // Check if p is in cw triangle between triEdgeCcw and triEdgeCw
        // Does not handle center = p case
        // Assumes cw angle from triEdgeCw to triEdgeCcw is less than 180 deg (otherwise not a triangle)
        // In order to return correct cw/ccw result for binary search both edges and p must be on
        // the same side of the 180 deg splitting line from center
        public static VisTriOrientation CompareVisTriP(VisVertices visVertices, VisTriEdge triEdgeCw,
            VisTriEdge triEdgeCcw, Vector2 center, Vector2 p)
        {
            Vector2 anglePCcw, anglePCw;
            GetTriAnglePoints(visVertices, triEdgeCw, triEdgeCcw, out anglePCw, out anglePCcw);
            var status = (VisTriOrientation)0;
            double orientation1 = MathExt.OrientationD(center, anglePCcw, p);
            // Inside some triangle ahead
            if (MathExt.Collinear(orientation1))
                status |= VisTriOrientation.Ccw;
            else if (orientation1 > 0)
                return VisTriOrientation.Ccw;
            double orientation2 = MathExt.OrientationD(center, anglePCw, p);
            if (MathExt.Collinear(orientation2))
                status |= VisTriOrientation.Cw;
            else if (orientation2 < 0)
                return VisTriOrientation.Cw;
            // p angleP1 angleP2 are all collinear
            // Don't enter this if angle between center angleP1 and center angleP2 is 180 degrees
            if (status == (VisTriOrientation.Ccw | VisTriOrientation.Cw) &&
                MathExt.IsVectorAngle0(center, anglePCcw, anglePCw))
            {
                var cmpP = MathExt.Approximately(center, anglePCcw) ? anglePCw : anglePCcw;
                if (!MathExt.IsVectorAngle0(center, cmpP, p))
                    return status;
                status |= VisTriOrientation.InAngle;
                if (MathExt.OnSegmentIncl(center, p, anglePCcw) ||
                    MathExt.OnSegmentIncl(center, p, anglePCw))
                    status |= VisTriOrientation.InTri;
                return status;
            }
            status |= VisTriOrientation.InAngle;
            // Is convex or collinear
            double orientation3 = MathExt.OrientationD(anglePCcw, anglePCw, p);
            if (orientation3 < MathExt.CollEpsilonD)
                status |= VisTriOrientation.InTri;
            return status;
        }

        /*[MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool? IsPInVisTri(VisVertices visVertices, DynamicArray<VisTriEdge> visTriEdges,
            int triEdgeIndexCw, int triEdgeIndexCcw, Vector2 center, Vector2 p,
            out int edgeIndex, out VisTriOrientation edgeOrientation)
        {
            edgeOrientation = CompareVisTriP(visVertices, visTriEdges[triEdgeIndexCw],
                visTriEdges[triEdgeIndexCcw], center, p);
            edgeIndex = -1;
            if (edgeOrientation.HasFlag(VisTriOrientation.InAngle))
            {
                edgeIndex = triEdgeIndexCw;
                if (edgeOrientation.HasFlag(VisTriOrientation.InTri))
                    return true;
                if ((edgeOrientation & (VisTriOrientation.Cw | VisTriOrientation.Ccw)) == 0)
                    return false;
            }
            // Inconclusive
            return null;
        }*/

        public static bool IsPVisible(VisVertices visVertices,
            DynamicArray<VisTriEdge> visTriEdges, int centerId, int pId, bool hasBlockedTri = true)
        {
            return IsPVisible(visVertices, visTriEdges, centerId, visVertices.V(pId), hasBlockedTri);
        }

        public static bool IsPVisible(VisVertices visVertices,
            DynamicArray<VisTriEdge> visTriEdges, int centerId, Vector2 p, bool hasBlockedTri = true)
        {
            int edgeIndex;
            VisTriOrientation edgeOrientation;
            return IsPVisible(visVertices, visTriEdges, centerId, p, hasBlockedTri, 
                out edgeIndex, out edgeOrientation);
        }

        public static bool IsPVisible(VisVertices visVertices,
            DynamicArray<VisTriEdge> visTriEdges, int centerId, int pId, bool hasBlockedTri, out int edgeIndex,
            out VisTriOrientation edgeOrientation)
        {
            return IsPVisible(visVertices, visTriEdges, centerId, visVertices.V(pId), hasBlockedTri,
                out edgeIndex, out edgeOrientation);
        }

        public static bool IsPVisible(VisVertices visVertices,
            DynamicArray<VisTriEdge> visTriEdges, int centerId, Vector2 p, bool hasBlockedTri, out int edgeIndex,
            out VisTriOrientation edgeOrientation)
        {
            var center = visVertices.V(centerId);
            if (MathExt.Approximately(center, p))
            {
                edgeIndex = 0;
                edgeOrientation = VisTriOrientation.Ccw | VisTriOrientation.Cw |
                    VisTriOrientation.InAngle | VisTriOrientation.InTri;
                return true;
            }
            edgeOrientation = (VisTriOrientation)0;
            // center is outside level
            if (visTriEdges.Count == 0)
            {
                edgeIndex = -1;
                return false;
            }
            // Start by checking if p is inside blocked angle space
            if (hasBlockedTri)
            {
                if (MathExt.PointInCwAngle(center, visTriEdges[0].intersection,
                    visTriEdges[visTriEdges.Count - 2].intersection, p))
                {
                    edgeIndex = visTriEdges.Count - 2;
                    edgeOrientation = VisTriOrientation.InAngle | VisTriOrientation.InTri;
                    return false;
                }
            }
            else
            {
                edgeOrientation = CompareVisTriP(visVertices, visTriEdges[^2],
                    visTriEdges[0], center, p);
                if (edgeOrientation.HasFlag(VisTriOrientation.InAngle))
                {
                    edgeIndex = visTriEdges.Count - 2;
                    if (edgeOrientation.HasFlag(VisTriOrientation.InTri))
                        return true;
                    if ((edgeOrientation & (VisTriOrientation.Cw | VisTriOrientation.Ccw)) == 0)
                        return false;
                }
            }
            var splitEdge = visTriEdges[^1];
            int splitEdgeVId = splitEdge.closestVAux;
            // Edges whose angular space crosses splitEdge
            edgeOrientation = CompareVisTriP(visVertices, visTriEdges[splitEdgeVId],
                visTriEdges[splitEdgeVId + 1], center, p);
            if (edgeOrientation.HasFlag(VisTriOrientation.InAngle))
            {
                edgeIndex = splitEdgeVId;
                if (edgeOrientation.HasFlag(VisTriOrientation.InTri))
                    return true;
                if ((edgeOrientation & (VisTriOrientation.Cw | VisTriOrientation.Ccw)) == 0)
                    return false;
            }
            double initOrientation = MathExt.OrientationD(center, splitEdge.intersection, p);
            int low, high;
            if (initOrientation < -MathExt.CollEpsilonD ||
                (MathExt.Collinear(initOrientation) && !MathExt.IsVectorAngle0(center, splitEdge.intersection, p)))
            {
                low = 0;
                high = splitEdgeVId - 1;
            }
            else
            {
                low = splitEdgeVId + 1;
                high = visTriEdges.Count - 3;
            }
            edgeIndex = visTriEdges.Count - 2;
            float sqrdDist = Vector2.DistanceSquared(center, p) - MathExt.Epsilon;
            while (low <= high)
            {
                int mid = edgeIndex = (high + low) / 2;
                var triEdgeCw = visTriEdges[mid];
                var triEdgeCcw = visTriEdges[mid + 1];
                edgeOrientation = CompareVisTriP(visVertices, triEdgeCw, triEdgeCcw, center, p);
                if (edgeOrientation.HasFlag(VisTriOrientation.InTri))
                    return true;
                if ((edgeOrientation & (VisTriOrientation.Cw | VisTriOrientation.Ccw)) == 0)
                    return false;
                Vector2 anglePCw, anglePCcw;
                if (edgeOrientation.HasFlag(VisTriOrientation.Ccw))
                {
                    low = mid + 1;
                    if (mid != visTriEdges.Count - 3 && edgeOrientation.HasFlag(VisTriOrientation.InAngle))
                    {
                        edgeIndex = low;
                        // Inside next triangle, if p is on overlapping edge
                        GetTriAnglePoints(visVertices, triEdgeCcw, visTriEdges[mid + 2],
                            out anglePCw, out anglePCcw);
                        if (sqrdDist <= Vector2.DistanceSquared(center, anglePCw))
                            return true;
                        else
                            low = high + 1; // break after checking if collinear to Cw edge
                    }
                }
                if (edgeOrientation.HasFlag(VisTriOrientation.Cw))
                {
                    high = mid - 1;
                    if (mid != 0 && edgeOrientation.HasFlag(VisTriOrientation.InAngle))
                    {
                        edgeIndex = high;
                        GetTriAnglePoints(visVertices, visTriEdges[mid - 1], triEdgeCw,
                            out anglePCw, out anglePCcw);
                        return sqrdDist <= Vector2.DistanceSquared(center, anglePCcw);
                    }
                }
            }

            return false;
        }
    }

    [Flags]
    public enum VisTriOrientation
    {
        // On or to the counter-clockwise side of counter-clockwise side
        Ccw = 1 << 0,
        // On or to the clockwise side of clockwise side
        Cw = 1 << 1,
        InAngle = 1 << 2,
        InTri = 1 << 3
    }

    public struct VisTriEdge
    {
        public Vector2 intersection;
        public int closestVAux;

        public VisTriEdge(Vector2 intersection, int closestVAux)
        {
            this.intersection = intersection;
            this.closestVAux = closestVAux;
        }

        public int GetClosestVId()
        {
            return closestVAux < 0 ? -closestVAux : closestVAux;
        }

        public bool IsEdgeOnCwSide()
        {
            return closestVAux < 0;
        }
    }
}
