using System;
using System.Collections.Generic;
using MathTools;
using Advanced.Algorithms.DataStructures;
using PolyBoolOpMartinez;
using Debugging;

namespace Pathfinding.VG
{
    public class RotationalPlaneSweep
    {
        private VisVertices obstacles;
        private DynamicArray<int> sortedIds;
        // Counter-clockwise angle/distance between p and w_i
        private double[] angles;
        private float[] squaredDists;
        // Tree of edges, key is distance between p and edge
        private RedBlackTree<EdgeNode> tree;

        private AngleComparer angleComparer;
        private EdgeDistComparer edgeComparer;
        // Closest collinear vertices with an edge on ccw/cw side
        private int closestCcwId, closestCwId, closestCcwCwId;
        public bool Reduced { get; private set; }
        public bool StoreTriangles { get; private set; }

        public RotationalPlaneSweep(bool reduced, bool storeTriangles)
        {
            edgeComparer = new EdgeDistComparer(this);
            tree = new RedBlackTree<EdgeNode>(edgeComparer, new EdgeIdEqComparer());
            this.Reduced = reduced;
            this.StoreTriangles = storeTriangles;
        }

        private bool Visible(Vector2 p, int wId, bool prevVisible,
            float intersectSqrdDist)
        {
            var w = obstacles.V(wId);
            // This check handles the case where
            // pw_i is blocked by the interior of the polygon that w_i is on
            if (obstacles.IsPoly(wId) && MathExt.PointInCwAngle(w, obstacles.NextV(wId), obstacles.PrevV(wId), p))
                return false;
            // w_i does not have same angle as w_i-1 so w_i
            // is not on the same line segment as w_i-1 
            // add non-poly points even if this is a reduced vg
            if (prevVisible)
                return squaredDists[wId] <= intersectSqrdDist;
            else
                return false;
        }

        private int ModifyTreeAux(int pId, int wId, int incidentId, bool insert)
        {
            // Insert into queue the obstacle edges incident to w_i on ccw side of rho
            // Delete from queue the obstacle edges incident to w_i on cw side of rho

            // Priority is distance from p to edge, so order in which rho hits edge
            // use squared distance from p to intersection between rho and edge

            var node = new EdgeNode(incidentId);
            if (wId == incidentId)
                incidentId = obstacles.NextID(wId);
            var p = obstacles.V(pId);
            double orientation = MathExt.OrientationD(p, obstacles.V(wId), obstacles.V(incidentId));
            // Ignore edges collinear with rho
            if (MathExt.Collinear(orientation))
                return 0;
            if (orientation > 0)
            {
                if (insert)
                    tree.Insert(node);
                if (StoreTriangles && closestCcwId == -1)
                    closestCcwId = wId;
                return 1;
            }
            else if (orientation < 0)
            {
                if (!insert && !MathExt.ApproxAngle(angles[wId], 0))
                {
                    int delIndex = tree.Delete(node);
                    Debug.Assert(delIndex != -1, "Node was not found in tree");
                }
                if (StoreTriangles && closestCwId == -1)
                    closestCwId = wId;
                return -1;
            }
            else
                throw new Exception("Orientation = 0, yet was not detected by Approx0");
        }

        // Returns should loop continue?
        private bool ModifyTree(int pId, int startI, int i, bool insert)
        {
            int wId = sortedIds[i];
            bool nextCollinear = i != sortedIds.Count - 1 &&
                MathExt.ApproxAngle(angles[wId], angles[sortedIds[i + 1]]);
            if (!obstacles.IsPoly(wId))
                return nextCollinear;
            edgeComparer.SetW(obstacles.V(wId));
            int prevPolyPId = obstacles.PrevID(wId);
            // Edge going from prevPolyId to wId
            int orientation1 = ModifyTreeAux(pId, wId, prevPolyPId, insert);
            // Edge with id wId is going from wId to nextPolyId
            int orientation2 = ModifyTreeAux(pId, wId, wId, insert);
            if (StoreTriangles && closestCcwCwId == -1 && orientation1 + orientation2 == 0 &&
                orientation1 != 0)
                closestCcwCwId = wId;
            return nextCollinear;
        }

        public void VisibleVertices(int pId, VisVertices obstacles, ref DynamicArray<Edge> edges,
            ref DynamicArray<VisTriEdge> visTriEdges, out Bbox visBbox)
        {
            edges.Clear();
            visTriEdges.Clear();
            var p = obstacles.V(pId);
            obstacles.AutoSetLastPolyEndI();
            this.obstacles = obstacles;
            bool isPPoly = obstacles.IsPoly(pId);
            // If Reduced & StoreTriangles & p is concave, do not add any edges
            bool isPConvex = true;

            if (angles == null || angles.Length < obstacles.vs.Count)
                angles = new double[obstacles.vs.Count + 2];
            if (squaredDists == null || squaredDists.Length < obstacles.vs.Count)
                squaredDists = new float[obstacles.vs.Count + 2];
            angleComparer = angleComparer ?? new AngleComparer();
            angleComparer.Reset(angles, squaredDists);

            // If isPPoly or p is on an edge, sweep exterior angle between incident edges of p
            // otherwise sweep 360 degrees from angle 0
            double initAngle = 0, stopAngle = 2 * Math.PI;
            var initRho = new Vector2(MathExt.MaxValue, p.y);
            if (isPPoly)
            {
                isPConvex = MathExt.IsConvex(obstacles.PrevV(pId), p, obstacles.NextV(pId));
                initRho = obstacles.NextV(pId);
                initAngle = MathExt.CcwAngle(p, initRho);
                stopAngle = MathExt.CcwAngle(p, obstacles.PrevV(pId));
                stopAngle -= initAngle;
                if (stopAngle < 0)
                    stopAngle += 2 * Math.PI;
            }
            else
            {
                for (int i = 0; i < obstacles.vs.Count; i++)
                {
                    var vertex = obstacles.V(i);
                    if (i == pId || VisVertices.IsDescriptor(vertex) || !obstacles.IsPoly(i))
                        continue;
                    var nextV = obstacles.NextV(i);
                    if (MathExt.Collinear(MathExt.OrientationD(vertex, p, nextV)) &&
                        MathExt.OnSegmentExcl((Vector2D)vertex, (Vector2D)p, (Vector2D)nextV))
                    {
                        initRho = nextV;
                        initAngle = MathExt.CcwAngle(p, initRho);
                        stopAngle = MathExt.CcwAngle(p, vertex);
                        stopAngle -= initAngle;
                        if (stopAngle < 0)
                            stopAngle += 2 * Math.PI;
                    }
                }
            }
            stopAngle += MathExt.AngleEpsilon;
            //var initRho = new Vector2((float)Math.Cos(initAngle), -(float)Math.Sin(initAngle)) + p;
            /*initRho *= Math.Min((MathExt.MaxValue - Math.Abs(p.x)) / Math.Abs(initRho.x),
                (MathExt.MaxValue - Math.Abs(p.y)) / Math.Abs(initRho.y));
            initRho += p;*/
            edgeComparer.SetP(p);
            edgeComparer.SetW(initRho);
            if (sortedIds.arr == null || sortedIds.arr.Length < obstacles.vs.Count)
                sortedIds.arr = new int[obstacles.vs.Count + 2];
            for (int i = 0; i < obstacles.vs.Count; i++)
            {
                var vertex = obstacles.V(i);
                if (i == pId || VisVertices.IsDescriptor(vertex))
                    continue;
                double angle = MathExt.CcwAngle(p, vertex) - initAngle;
                if (angle < 0)
                    angle += 2 * Math.PI;
                angles[i] = angle;
                // Insert all obstacle edges intersected by initRho in tree
                if (obstacles.IsPoly(i) && MathExt.HalfLineToLine(p, initRho, vertex, obstacles.NextV(i), out Vector2 _))
                    tree.Insert(new EdgeNode(i));
                if (angle > stopAngle && vertex != p)
                    continue;
                sortedIds.Add(i);
                var diff = vertex - p;
                squaredDists[i] = diff.LengthSquared();
            }

            for (int i = 0; i < sortedIds.Count; i++)
            {
                int id = sortedIds[i];
                var vertex = obstacles.V(id);
                if (obstacles.IsPoly(id) && vertex == p)
                {
                    double angle1 = angles[obstacles.PrevID(id)];
                    double angle2 = angles[obstacles.NextID(id)];
                    double interiorAngle = (angle2 - angle1);
                    if (interiorAngle < 0)
                        interiorAngle += 2 * Math.PI;
                    double midAngle = angle1 + (interiorAngle * 0.5);
                    angles[id] = midAngle;
                }
            }

            Array.Sort<int>(sortedIds.arr, 0, sortedIds.Count, angleComparer);

            bool prevVisible = false;
            visBbox = new Bbox(p.x, p.y, p.x, p.y);
            var intersection = new Vector2(float.NaN, float.NaN);
            bool closestCcwVisible, closestCwVisible, closestCcwCwVisible;
            // Vis tri index of last rho_i with an angle < PI
            int lastUnder180TriIndex = -1;
            // Is p on or outside a polygon, is p outside the level?
            bool isPInFreespace = false, isPExterior = false;
            for (int i = 0; i < sortedIds.Count;)
            {
                closestCcwVisible = closestCwVisible = closestCcwCwVisible = false;
                closestCcwId = closestCwId = closestCcwCwId = -1;
                prevVisible = true;
                int startColI = i;
                while (ModifyTree(pId, startColI, i, insert: false) && i < sortedIds.Count - 1)
                    i++;
                int endColI = i;
                var edgeIter = tree.FindMin();
                float intersectSqrdDist = float.MaxValue;
                if (!edgeIter.IsNull() && obstacles.LineToEdge(p, obstacles.V(sortedIds[endColI]),
                    edgeIter.Value.id, out intersection))
                {
                    intersectSqrdDist = Vector2.DistanceSquared(p, intersection);
                }
                //bool nextCollinear = false;
                var firstW = obstacles.V(sortedIds[startColI]);
                for (i = startColI; i <= endColI;)
                {
                    if (closestCcwCwVisible) //&& intersection != w)
                        break;
                    int wId = sortedIds[i];
                    var w = obstacles.V(wId);
                    bool prevCollinear = w != firstW; //nextCollinear;
                    //nextCollinear = i != endI;
                    bool visible = true;
                    int startSameI = i;
                    do
                    {
                        visible &= Visible(p, wId, prevVisible, intersectSqrdDist);
                        i++;
                        if (i == sortedIds.Count)
                            break;
                        wId = sortedIds[i];
                    } while (w == obstacles.V(wId));
                    int endSameI = i - 1;
                    for (i = startSameI; i <= endSameI; i++)
                    {
                        wId = sortedIds[i];
                        bool isWPoly = obstacles.IsPoly(wId);
                        isPInFreespace |= visible & isWPoly;
                        // Add wId to visibility tree at p, cost is euclidean distance
                        if (visible & (!prevCollinear | StoreTriangles | !Reduced))
                        {
                            bool isConvex = !isWPoly ||
                                MathExt.IsConvex(obstacles.PrevV(wId), w, obstacles.NextV(wId));
                            if (!Reduced || (!prevCollinear && isConvex && isPConvex))
                            {
                                if (!Reduced || !isPPoly ||
                                    (!obstacles.IsInNonTautRegion(pId, w) && !obstacles.IsInNonTautRegion(wId, p)))
                                    edges.Add(new Edge(wId, (float)Math.Sqrt(squaredDists[wId])));
                            }
                            /*if (p == w && startColI != endColI)
                                firstW = obstacles.V(sortedIds[i + 1]);*/
                            if (isWPoly)
                            {
                                if (!isConvex)
                                    closestCcwCwId = wId;
                                if (p == w)
                                    closestCwId = closestCcwId = closestCcwCwId = wId;
                                // Vertex with edge on each side will always block rho
                                if (closestCcwCwId == wId)
                                {
                                    intersection = w;
                                    closestCcwCwVisible = true;
                                }
                            }
                            if (StoreTriangles)
                            {
                                if (closestCwId == wId)
                                    closestCwVisible = true;
                                if (closestCcwId == wId)
                                    closestCcwVisible = true;
                            }
                        }
                    }
                    prevVisible = visible;
                }
                isPExterior |= float.IsNaN(intersection.x);
                if (StoreTriangles && !isPExterior)
                {
                    // Don't add a duplicate vis tri edge, if closestCwId = closestCcwId
                    if (closestCwVisible && closestCwId != closestCcwId)
                        visTriEdges.Add(new VisTriEdge(intersection, -closestCwId));
                    if (closestCcwVisible)
                        visTriEdges.Add(new VisTriEdge(intersection, closestCcwId));
                    visBbox = visBbox.Enclose(intersection);
                    if (angles[sortedIds[startColI]] < Math.PI - MathExt.AngleEpsilon)
                        lastUnder180TriIndex = visTriEdges.Count - 1;
                }
                for (i = startColI; i <= endColI; i++)
                    ModifyTree(pId, startColI, i, insert: true);
            }
            if (isPExterior)
                visTriEdges.Clear();
            else if (StoreTriangles)
            {
                Debugging.Debug.Assert(lastUnder180TriIndex != -1,
                    "There must be at least 2 vis tri edges under 180 degrees for p " + p);
                // Translate initAngleP to origo, invert and translate back 
                // to get 180 deg angle p (from startAngle)
                visTriEdges.Add(new VisTriEdge((-(initRho - p)) + p, lastUnder180TriIndex));
            }
            tree.Clear();
            sortedIds.Clear();
            // No polygon points are visible, so p must be inside a polygon
            if (!isPInFreespace)
            {
                edges.Clear();
                visBbox = new Bbox(p.x, p.y, p.x, p.y);
            }
        }

        // Assumes polygons of even depth is in clockwise order
        // and polygons of odd depth (holes, inverse polygons) 
        // are kept in ccw order, so vectors for orientation are also swapped
        public void VisibilityGraph(List<List<Vector2>> polygons,
            ref VisVertices obstaclesRef,
            ref DynamicArray<DynamicArray<Edge>> allEdges,
            ref DynamicArray<DynamicArray<VisTriEdge>> allVisTriEdges, ref DynamicArray<Bbox> visBboxes)
        {
            this.obstacles = obstaclesRef;
            /*for (int i = 0; i < polygons.Count; i++)
            {
                var polygon = polygons[i];
                bool isClockwise = MathExt.IsPolygonClockwise(polygon);
                bool isInverse = inversePolygons[i] != -1;
                if (isInverse == isClockwise)
                    polygon.Reverse();
            }*/

            obstacles.Reset(polygons);

            if (allEdges.arr.Length < obstacles.vs.Count + 2)
                allEdges.arr = new DynamicArray<Edge>[obstacles.vs.Count + 2];
            allEdges.ResizeAndExtendTo(obstacles.vs.Count);
            if (StoreTriangles)
            {
                allVisTriEdges.ResizeAndExtendTo(obstacles.vs.Count);
                visBboxes.ResizeAndExtendTo(obstacles.vs.Count);
            }

            for (int i = 0; i < obstacles.vs.Count; i++)
            {
                if (obstacles.IsDescriptor(i))
                    continue;
                var edges = allEdges[i];
                if (edges.arr == null)
                    edges = new DynamicArray<Edge>(0);
                else
                    edges.Clear();
                // Only calculate visibility from convex vertices
                if ((Reduced & !StoreTriangles) &&
                    !MathExt.IsConvex(obstacles.PrevV(i), obstacles.V(i), obstacles.NextV(i)))
                    continue;
                DynamicArray<VisTriEdge> visTriEdges;
                if (StoreTriangles)
                {
                    visTriEdges = allVisTriEdges[i];
                    if (visTriEdges.arr == null)
                        visTriEdges = new DynamicArray<VisTriEdge>(2);
                    else
                        visTriEdges.Clear();
                }
                else
                    visTriEdges = default(DynamicArray<VisTriEdge>);
                VisibleVertices(i, obstacles, ref edges, ref visTriEdges, out Bbox visBbox);
                allEdges[i] = edges;
                if (StoreTriangles)
                    allVisTriEdges[i] = visTriEdges;
                visBboxes.Add(visBbox);
            }
            obstaclesRef = this.obstacles;
        }

        private struct EdgeNode
        {
            public int id;

            public EdgeNode(int id)
            {
                this.id = id;
            }

            public override string ToString()
            {
                return "EdgeNode ID: " + id;
            }
        }

        private class EdgeDistComparer : Comparer<EdgeNode>
        {
            private Vector2 p, w;
            private readonly RotationalPlaneSweep rps;

            public EdgeDistComparer(RotationalPlaneSweep rps)
            {
                this.rps = rps;
            }

            public void SetP(Vector2 p)
            {
                this.p = p;
            }

            public void SetW(Vector2 w)
            {
                this.w = w;
            }

            public override int Compare(EdgeNode edge1, EdgeNode edge2)
            {
                Vector2 intersection1, intersection2;
                float sqrdDist1 = GetSqrdDist(edge1, out intersection1);
                float sqrdDist2 = GetSqrdDist(edge2, out intersection2);
                // edge1 and edge2 are incident edges and intersect rho in their endpoints
                if (MathExt.Approximately(sqrdDist1, sqrdDist2))
                {
                    // oppA and oppB are opposite endpoints of where the two edges meet
                    Vector2 a1, a2, b1, b2, intersection, oppA, oppB;
                    rps.obstacles.E(edge1.id, out a1, out a2);
                    rps.obstacles.E(edge2.id, out b1, out b2);
                    oppA = MathExt.Approximately(a1, intersection1) ? a2 : a1;
                    oppB = MathExt.Approximately(b1, intersection2) ? b2 : b1;
                    if (MathExt.LineToLine(p, oppB, a1, a2, out intersection))
                    {
                        sqrdDist1 = Vector2.DistanceSquared(p, intersection);
                        sqrdDist2 = Vector2.DistanceSquared(p, oppB);
                    }
                    else
                    {
                        MathExt.LineToLine(p, oppA, b1, b2, out intersection);
                        sqrdDist1 = Vector2.DistanceSquared(p, oppA);
                        sqrdDist2 = Vector2.DistanceSquared(p, intersection);
                    }
                }
                return sqrdDist1.CompareTo(sqrdDist2);
            }

            private float GetSqrdDist(EdgeNode edge, out Vector2 intersection)
            {
                Vector2 a1, a2;
                rps.obstacles.E(edge.id, out a1, out a2);
                MathExt.LineToLine(p, w, a1, a2, out intersection);
                return Vector2.DistanceSquared(p, intersection);
            }
        }

        private class EdgeIdEqComparer : EqualityComparer<EdgeNode>
        {
            public override bool Equals(EdgeNode edge1, EdgeNode edge2)
            {
                return edge1.id == edge2.id;
            }

            public override int GetHashCode(EdgeNode edge)
            {
                return edge.id;
            }
        }
    }
}
