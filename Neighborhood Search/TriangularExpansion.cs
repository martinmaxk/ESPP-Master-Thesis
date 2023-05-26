using System;
using System.Collections.Generic;
using MathTools;
using Advanced.Algorithms.DataStructures;
using PolyBoolOpMartinez;
using Debugging;
using CGALDotNet.Polygons;
using CGALDotNet;
using CGALDotNetGeometry.Numerics;
using System.Linq;

namespace Pathfinding.VG
{
    public class TriangularExpansion
    {
        private VisVertices obstacles;
        private List<IntPtr> triExpPtrs;
        private List<PolygonWithHoles2<EIK>> traversablePolys;
        private Dictionary<Vector2, int> vertexToId;
        private List<Point2d> outputPoints = new List<Point2d>();
        private DynamicArray<int> sortedIndices;
        private int[] ids;
        // Counter-clockwise angle/distance between p and w_i
        private double[] angles;
        private float[] squaredDists;

        private AngleComparer angleComparer;
        private const double AngleEpsilon = 1e-8;
        // Closest collinear vertices with an edge on ccw/cw side
        private int closestCcwId, closestCwId, closestCcwCwId;
        public bool Reduced { get; private set; }
        public bool StoreTriangles { get; private set; }

        public System.Diagnostics.Stopwatch sw;

        public TriangularExpansion(bool reduced, bool storeTriangles)
        {
            this.Reduced = reduced;
            this.StoreTriangles = storeTriangles;
        }

        private int CheckClockwiseAux(int pId, int wId, int incidentId)
        {
            if (wId == incidentId)
                incidentId = obstacles.NextID(wId);
            var p = obstacles.V(pId);
            double orientation = MathExt.OrientationD(p, obstacles.V(wId), obstacles.V(incidentId));
            // Ignore edges collinear with rho
            if (MathExt.Collinear(orientation))
                return 0;
            if (orientation > 0)
            {
                if (closestCcwId == -1)
                    closestCcwId = wId;
                return 1;
            }
            else if (orientation < 0)
            {
                if (closestCwId == -1)
                    closestCwId = wId;
                return -1;
            }
            else
                throw new Exception("Orientation = 0, yet was not detected by Approx0");
        }

        // Returns should loop continue?
        private bool CheckClockwise(int pId, int startI, int i)
        {
            int wId = ids[sortedIndices[i]];
            bool nextCollinear = i != sortedIndices.Count - 1 &&
                MathExt.Approximately(angles[sortedIndices[i]], angles[sortedIndices[i + 1]], AngleEpsilon);
            if (wId == -1)
                return nextCollinear;
            int prevPolyPId = obstacles.PrevID(wId);
            // Edge going from prevPolyId to wId
            int orientation1 = CheckClockwiseAux(pId, wId, prevPolyPId);
            // Edge with id wId is going from wId to nextPolyId
            int orientation2 = CheckClockwiseAux(pId, wId, wId);
            if (closestCcwCwId == -1 && orientation1 + orientation2 == 0 &&
                orientation1 != 0)
                closestCcwCwId = wId;
            return nextCollinear;
        }

        public void VisibleVertices(int pId, VisVertices obstacles, 
            List<PolygonWithHoles2<EIK>> traversablePolys, List<IntPtr> triExpPtrs, 
            Dictionary<Vector2, int> vertexToId, 
            ref DynamicArray<Edge> edges, ref DynamicArray<VisTriEdge> visTriEdges, out Bbox visBbox)
        {
            edges.Clear();
            visTriEdges.Clear();
            this.traversablePolys = traversablePolys;
            this.triExpPtrs = triExpPtrs;
            var p = obstacles.V(pId);
            obstacles.AutoSetLastPolyEndI();
            this.obstacles = obstacles;
            bool isPPoly = obstacles.IsPoly(pId);
            // If Reduced & StoreTriangles & p is concave, do not add any edges
            bool isPConvex = true;

            angleComparer = angleComparer ?? new AngleComparer(AngleEpsilon);

            double initAngle = 0, stopAngle = 2 * Math.PI;
            var initRho = new Vector2(MathExt.MaxValue, p.y);
            var pMoved = (Vector2D)p;
            //Console.WriteLine(p / 32);
            if (isPPoly)
            {
                var prevV = obstacles.PrevV(pId);
                var nextV = obstacles.NextV(pId);
                isPConvex = MathExt.IsConvex(prevV, p, nextV);
                initRho = nextV;
                initAngle = MathExt.CcwAngle(p, initRho);
                stopAngle = MathExt.CcwAngle(p, prevV);
                stopAngle -= initAngle;
                if (stopAngle < 0)
                    stopAngle += 2 * Math.PI;
                var pMovedF = ((prevV + nextV) / 2) - p;
                pMovedF /= pMovedF.Length();
                if (isPConvex)
                    pMovedF = -pMovedF;
                pMoved = (Vector2D)pMovedF * 1e-8;
                pMoved += (Vector2D)p;
            }
            /*else
            {
                for (int i = 0; i < obstacles.vs.Count; i++)
                {
                    var vertex = obstacles.V(i);
                    if (i == pId || VisVertices.IsDescriptor(vertex) || !IsPoly(i))
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
            }*/
            sw?.Stop();
            var polyIndex = PolygonVisibility<EIK>.Instance.FindIndexContainsPoint(
                new Point2d(pMoved.x, pMoved.y), traversablePolys);
            sw?.Start();
            PolygonVisibility<EIK>.Instance.ComputeVisibilityTEVCached(
                new Point2d(pMoved.x, pMoved.y), triExpPtrs[polyIndex], outputPoints);
            //outputPoints.Clear();
            //PolygonVisibility<EEK>.Instance.ComputeVisibilityTEV(new Point2d(p.x, p.y), traversablePolys[0], out var poly);
            //Console.WriteLine(p / 32);
            //if (obstacles.vs.Count > 861)
                //Console.WriteLine("hey " + obstacles.V(861) / 32);
            //Console.WriteLine(string.Join(";", outputPoints.Select(p => new Vector2D(p.x, p.y) / 32)));
            if (angles == null || angles.Length < outputPoints.Count)
                angles = new double[outputPoints.Count];
            if (squaredDists == null || squaredDists.Length < outputPoints.Count)
                squaredDists = new float[outputPoints.Count];
            if (ids == null || ids.Length < outputPoints.Count)
                ids = new int[outputPoints.Count];
            if (sortedIndices.arr == null || sortedIndices.arr.Length < outputPoints.Count)
                sortedIndices.arr = new int[outputPoints.Count];
            angleComparer.Reset(angles, squaredDists);
            var pD = (Vector2D)p;
            for (int i = 0; i < outputPoints.Count; i++)
            {
                //outputPoints[i] = new Point2d(Math.Round(outputPoints[i].x, 3, MidpointRounding.AwayFromZero),
                    //Math.Round(outputPoints[i].y, 3, MidpointRounding.AwayFromZero));
                var vertexD = new Vector2D(outputPoints[i].x, outputPoints[i].y);
                var vertex = (Vector2)vertexD;
                if (vertex == p)
                    continue;
                double angle = MathExt.CcwAngle(pD, vertexD) - initAngle;
                if (angle < 0)
                    angle += 2 * Math.PI;
                angles[i] = angle;
                var diff = vertex - p;
                squaredDists[i] = diff.LengthSquared();
                sortedIndices.Add(i);
                if (vertexToId.TryGetValue(vertex, out int vId))
                    ids[i] = vId;
                else
                    ids[i] = -1;
            }

            /*for (int i = 0; i < sortedIds.Count; i++)
            {
                int id = sortedIds[i];
                var vertex = obstacles.V(id);
                if (IsPoly(id) && vertex == p)
                {
                    double angle1 = angles[obstacles.PrevID(id)];
                    double angle2 = angles[obstacles.NextID(id)];
                    double interiorAngle = (angle2 - angle1);
                    if (interiorAngle < 0)
                        interiorAngle += 2 * Math.PI;
                    double midAngle = angle1 + (interiorAngle * 0.5);
                    angles[id] = midAngle;
                }
            }*/

            Array.Sort<int>(sortedIndices.arr, 0, sortedIndices.Count, angleComparer);

            visBbox = new Bbox(p.x, p.y, p.x, p.y);
            // Vis tri index of last rho_i with an angle < PI
            int lastUnder180TriIndex = -1;
            for (int i = 0; i < sortedIndices.Count;)
            {
                closestCcwId = closestCwId = closestCcwCwId = -1;
                int startColI = i;
                while (CheckClockwise(pId, startColI, i) && i < sortedIndices.Count - 1)
                    i++;
                int endColI = i;
                //bool nextCollinear = false;
                /*Console.WriteLine((p / 32) + " " + outputPoints[sortedIndices[startColI]] / 32);
                Console.WriteLine(angles[sortedIndices[startColI]] + " " + angles[sortedIndices[startColI + 1]]);
                Console.WriteLine(startColI + " " + endColI);*/
                var firstW = obstacles.V(ids[sortedIndices[startColI]]);
                for (i = startColI; i <= endColI; i++)
                {
                    //Console.WriteLine(outputPoints[sortedIndices[i]] / 32);
                    int wIndex = sortedIndices[i];
                    int wId = ids[wIndex];
                    if (wId == -1)
                        continue;
                    var w = obstacles.V(wId);
                    bool prevCollinear = w != firstW;

                    // Add wId to visibility tree at p, cost is euclidean distance
                    if (!prevCollinear | StoreTriangles | !Reduced)
                    {
                        bool isConvex = MathExt.IsConvex(obstacles.PrevV(wId), w, obstacles.NextV(wId));
                        if (!Reduced || (!prevCollinear && isConvex && isPConvex))
                        {
                            if (!Reduced || !isPPoly ||
                                (!obstacles.IsInNonTautRegion(pId, w) && !obstacles.IsInNonTautRegion(wId, p)))
                            {
                                //Console.WriteLine(obstacles.V(wId) / 32);
                                edges.Add(new Edge(wId, (float)Math.Sqrt(squaredDists[wIndex])));
                            }
                        }
                        if (!isConvex)
                            closestCcwCwId = wId;
                        /*if (p == w)
                            closestCwId = closestCcwId = closestCcwCwId = wId;
                        // Vertex with edge on each side will always block rho
                        if (closestCcwCwId == wId)
                        {
                            intersection = w;
                        }*/
                    }
                }
                if (StoreTriangles)
                {
                    var intersectionP = outputPoints[sortedIndices[endColI]];
                    var intersection = new Vector2((float)intersectionP.x, (float)intersectionP.y);
                    // Don't add a duplicate vis tri edge, if closestCwId = closestCcwId
                    if (closestCwId != -1 && closestCwId != closestCcwId)
                    {
                        //Console.WriteLine("Cw " + obstacles.V(closestCwId) / 32 + " " + intersection / 32);
                        visTriEdges.Add(new VisTriEdge(intersection, -closestCwId));
                    }
                    if (closestCcwId != -1)
                    {
                        //Console.WriteLine("Ccw " + obstacles.V(closestCcwId) / 32 + " " + intersection / 32);
                        visTriEdges.Add(new VisTriEdge(intersection, closestCcwId));
                    }
                    Debug.Assert(closestCwId != -1 || closestCcwId != -1);
                    visBbox = visBbox.Enclose(intersection);
                    if (angles[sortedIndices[startColI]] < Math.PI - MathExt.AngleEpsilon)
                        lastUnder180TriIndex = visTriEdges.Count - 1;
                }
            }
            bool isPExterior = outputPoints.Count == 0;
            if (!isPExterior && StoreTriangles)
            {
                Debugging.Debug.Assert(lastUnder180TriIndex != -1,
                    "There must be at least 2 vis tri edges under 180 degrees for p " + p);
                // Translate initAngleP to origo, invert and translate back 
                // to get 180 deg angle p (from startAngle)
                visTriEdges.Add(new VisTriEdge((-(initRho - p)) + p, lastUnder180TriIndex));
            }
            sortedIndices.Clear();
            // No polygon points are visible, so p must be inside a polygon
            if (isPExterior)
                visBbox = new Bbox(p.x, p.y, p.x, p.y);
            /*var rps = new RotationalPlaneSweep(Reduced, StoreTriangles);
            var expectedEdges = new DynamicArray<Edge>(0);
            var expectedVisTriEdges = new DynamicArray<VisTriEdge>(2);
            rps.VisibleVertices(pId, obstacles, ref expectedEdges, ref expectedVisTriEdges, out var expectedVisBbox);

            Debug.Assert(MathExt.Approximately(edges.Count, expectedEdges.Count, 2));
            Debug.Assert(MathExt.Approximately(visTriEdges.Count, expectedVisTriEdges.Count, 2));
            Debug.Assert(edges.Count <= expectedEdges.Count);
            int goalIndex = -1;
            for (int i = 0; i < expectedEdges.Count; i++)
            {
                if (!obstacles.IsPoly(expectedEdges[i].vertexId))
                    goalIndex = i;
            }
            bool isGoalVisible = goalIndex != -1;
            int expectEdgesSub = isGoalVisible ? 1 : 0;
            int actualI = 0;
            for (int expectI = 0; expectI < expectedEdges.Count; expectI++)
            {
                if (expectI == goalIndex)
                    continue;
                int actualVId = edges[actualI].vertexId;
                int expectVId = expectedEdges[expectI].vertexId;
                if (goalIndex != -1 && MathExt.Approximately(MathExt.CcwAngle(p, obstacles.V(expectedEdges[goalIndex].vertexId)), 
                    MathExt.CcwAngle(p, obstacles.V(actualVId)), AngleEpsilon) &&
                    Vector2.DistanceSquared(p, obstacles.V(expectedEdges[goalIndex].vertexId)) < 
                    Vector2.DistanceSquared(p, obstacles.V(actualVId)))
                {
                    actualI++;
                    expectI--;
                    expectEdgesSub--;
                    continue;
                }
                //if (!MathExt.Collinear(MathExt.OrientationD(p, obstacles.V(actualVId), obstacles.V(expectVId))))
                //{
                if (actualVId != expectVId)
                {
                    Console.WriteLine(obstacles.V(actualVId) / 32 + " " + obstacles.V(expectVId) / 32);
                    Console.WriteLine(string.Join(";", outputPoints.Select(p => new Vector2D(p.x, p.y) / 32)));
                }
                Debug.Assert(actualVId == expectVId);
                Debug.Assert(MathExt.Approximately(edges[actualI].cost, expectedEdges[expectI].cost));
                //}
                actualI++;
            }
            Debug.Assert(edges.Count == expectedEdges.Count - expectEdgesSub);
            Debug.Assert(visTriEdges.Count == expectedVisTriEdges.Count);
            for (int i = 0; i < expectedVisTriEdges.Count; i++)
            {
                var visTriEdge = visTriEdges[i];
                var expectVisTriEdge = expectedVisTriEdges[i];
                Debug.Assert(visTriEdge.closestVAux == expectVisTriEdge.closestVAux);
                Console.WriteLine(obstacles.V(visTriEdge.GetClosestVId()) / 32 + " " + visTriEdge.intersection / 32 + " " + expectVisTriEdge.intersection / 32);
                Debug.Assert(visTriEdge.intersection == expectVisTriEdge.intersection);
            }*/
        }

        // Assumes polygons of even depth is in clockwise order
        // and polygons of odd depth (holes, inverse polygons) 
        // are kept in ccw order, so vectors for orientation are also swapped
        public void VisibilityGraph(Polygon union, List<List<Vector2>> polygons,
            ref VisVertices obstaclesRef,
            ref DynamicArray<DynamicArray<Edge>> allEdges,
            ref DynamicArray<DynamicArray<VisTriEdge>> allVisTriEdges, ref DynamicArray<Bbox> visBboxes)
        {
            PolygonUtils.GetTraversablePolygonsAndTEV(union, out traversablePolys, out triExpPtrs);
            this.obstacles = obstaclesRef;
            /*for (int i = 0; i < polygons.Count; i++)
            {
                var polygon = polygons[i];
                bool isClockwise = MathExt.IsPolygonClockwise(polygon);
                bool isInverse = inversePolygons[i] != -1;
                if (isInverse == isClockwise)
                    polygon.Reverse();
            }*/

            // Omit boundary polygon
            obstacles.Reset(polygons, 1, polygons.Count);
            obstacles.lastPolyEndI = obstacles.vs.Count;

            this.vertexToId = obstacles.GetVertexToIdMap();

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
                VisibleVertices(i, obstacles, traversablePolys, triExpPtrs, vertexToId, 
                    ref edges, ref visTriEdges, out Bbox visBbox);
                allEdges[i] = edges;
                if (StoreTriangles)
                    allVisTriEdges[i] = visTriEdges;
                visBboxes.Add(visBbox);
            }
            obstaclesRef = this.obstacles;
        }
    }
}
