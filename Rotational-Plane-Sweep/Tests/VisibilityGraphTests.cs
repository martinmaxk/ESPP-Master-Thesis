using System;
using System.Linq;
using System.Collections.Generic;
using Xunit;
using MathTools;
using Pathfinding.VG;
using Pathfinding;
using PolyBoolOpMartinez;
using TriO = Pathfinding.VG.VisTriOrientation;

namespace RpsTests
{
    public class VisibilityGraphTests
    {
        private List<List<Vector2>> polygons;
        private VisVertices obstacles;
        private DynamicArray<DynamicArray<Edge>> allEdges = 
            new DynamicArray<DynamicArray<Edge>>(4);
        private DynamicArray<DynamicArray<VisTriEdge>> allVisTriEdges = 
            new DynamicArray<DynamicArray<VisTriEdge>>(4);
        private DynamicArray<Bbox> visBboxes = new DynamicArray<Bbox>(4);
        private DynamicArray<Edge> edges = new DynamicArray<Edge>(0);
        private DynamicArray<VisTriEdge> visTriEdges = new DynamicArray<VisTriEdge>(2);
        private Bbox visBbox;

        private VisibilityGraph vg = new VisibilityGraph();
        private PlaneSweepConnector rpsConnector = new PlaneSweepConnector();
        private DfsVgConnector dfsConnector = new DfsVgConnector();
        private RotationalPlaneSweep rps;

        public VisibilityGraphTests()
        {
            polygons = new List<List<Vector2>>
            {
                new List<Vector2>() { new Vector2(50, 50), new Vector2(60, 50), new Vector2(60, 60), 
                    new Vector2(50, 60) },
                new List<Vector2>() { new Vector2(50, 40), new Vector2(60, 30), new Vector2(55, 40) },
                new List<Vector2>() { new Vector2(55, 65), new Vector2(60, 70), new Vector2(65, 65), 
                    new Vector2(65, 75), new Vector2(55, 75) },
                new List<Vector2>() { new Vector2(60, 10), new Vector2(70, 10), new Vector2(70, 20) },
                new List<Vector2>() { new Vector2(55, 25), new Vector2(60, 20), new Vector2(60, 25) },
                new List<Vector2>() { new Vector2(60, 16), new Vector2(64, 16), new Vector2(64, 18) },
                new List<Vector2>() { new Vector2(0, 0), new Vector2(0, 100), new Vector2(100, 100), 
                    new Vector2(100, 0) }
            };
            int count = 0;
            for (int i = 0; i < polygons.Count; i++)
                count += polygons[i].Count + 2;
            obstacles = new VisVertices(new DynamicArray<Vector2>(count));
            obstacles.vs.Extend(count);
            count = 0;
            for (int i = 0; i < polygons.Count; i++)
            {
                var polygon = polygons[i];
                int head = count++;
                for (int j = 0; j < polygon.Count; j++)
                    obstacles.vs[count++] = polygon[j];

                obstacles.SetDescriptor(head, count - 1);
                obstacles.SetDescriptor(count++, head + 1);
            }
        }

        private void VisibleVertices(bool reduced, bool storeTriangles, int pId, Vector2 p)
        {
            edges.Clear();
            visTriEdges.Clear();
            AssertApprox(obstacles.vs[pId], p);
            var rps = new RotationalPlaneSweep(reduced, storeTriangles);
            rps.VisibleVertices(pId, obstacles, ref edges, ref visTriEdges, out visBbox);
        }

        private void VisibleVertices(bool reduced, bool storeTriangles)
        {
            VisibleVertices(reduced, storeTriangles, 2, new Vector2(60, 50));
        }

        [Fact]
        public void VerifyEdges()
        {
            VisibleVertices(false, false);
            AssertEqEdge(0, new Vector2(60, 60));
            AssertEqEdge(1, new Vector2(60, 70));
            AssertEqEdge(2, new Vector2(65, 65));
            AssertEqEdge(3, new Vector2(100, 100));
            AssertEqEdge(4, new Vector2(100, 0));
            AssertEqEdge(5, new Vector2(70, 20));
            AssertEqEdge(6, new Vector2(64, 18));
            AssertEqEdge(7, new Vector2(60, 30));
            AssertEqEdge(8, new Vector2(60, 25));
            AssertEqEdge(9, new Vector2(60, 20));
            AssertEqEdge(10, new Vector2(60, 16));
            AssertEqEdge(11, new Vector2(60, 10));
            AssertEqEdge(12, new Vector2(55, 40));
            AssertEqEdge(13, new Vector2(50, 40));
            AssertEqEdge(14, new Vector2(0, 0));
            AssertEqEdge(15, new Vector2(50, 50));
            Assert.Equal(16, edges.Count);
        }

        [Fact]
        public void VerifyReducedEdges()
        {
            VisibleVertices(true, false);
            AssertEqEdge(0, new Vector2(60, 60));
            AssertEqEdge(1, new Vector2(65, 65));
            AssertEqEdge(2, new Vector2(60, 30));
            AssertEqEdge(3, new Vector2(50, 40));
            AssertEqEdge(4, new Vector2(50, 50));
            Assert.Equal(5, edges.Count);
        }

        [Theory]
        [InlineData(false)]
        [InlineData(true)]
        public void VerifyVisTriEdges(bool reduced)
        {
            VisibleVertices(reduced, true);
            AssertApprox(visBbox.xmin, 0);
            AssertApprox(visBbox.xmax, 100);
            AssertApprox(visBbox.ymin, 0);
            AssertApprox(visBbox.ymax, 100);
            AssertEqVisTriEdge(0, new Vector2(60, 60), new Vector2(60, 70));
            AssertEqVisTriEdge(1, new Vector2(60, 70), new Vector2(60, 70));
            AssertEqVisTriEdge(2, new Vector2(65, 65), new Vector2(76 + (2f / 3f), 100));
            AssertEqVisTriEdge(3, new Vector2(100, 100), new Vector2(100, 100));
            AssertEqVisTriEdge(4, new Vector2(100, 0), new Vector2(100, 0));
            AssertEqVisTriEdge(5, new Vector2(70, 20), new Vector2(76 + (2f / 3f), 0));
            AssertEqVisTriEdge(6, new Vector2(64, 18), new Vector2(64.444444f, 14.444444f));
            AssertEqVisTriEdge(7, new Vector2(60, 16), new Vector2(60, 0));
            AssertEqVisTriEdge(8, new Vector2(60, 30), new Vector2(60, 0));
            AssertEqVisTriEdge(9, new Vector2(55, 40), new Vector2(55, 40));
            AssertEqVisTriEdge(10, new Vector2(50, 40), new Vector2(10, 0));
            AssertEqVisTriEdge(11, new Vector2(0, 0), new Vector2(0, 0));
            AssertEqVisTriEdge(12, new Vector2(50, 50), new Vector2(0, 50));
            var splitEdge = visTriEdges[13];
            Assert.Equal(6, splitEdge.closestVAux);
            AssertApprox(splitEdge.intersection, new Vector2(60, 40));
            Assert.Equal(14, visTriEdges.Count);
        }

        public static IEnumerable<object[]> ConnectDataGen()
        {
            yield return new object[] 
            {   
                new Vector2(30, 28), new Vector2(70, 26), false, null,
                new List<Vector2> { new Vector2(60, 25), new Vector2(55, 25), new Vector2(60, 20),
                new Vector2(64, 18), new Vector2(60, 16), new Vector2(60, 10), new Vector2(0, 0), 
                new Vector2(0, 100), new Vector2(55, 75), new Vector2(50, 60), new Vector2(50, 50), 
                new Vector2(60, 50), new Vector2(50, 40), new Vector2(60, 30), new Vector2(70, 26) }, 
                new List<Vector2> { new Vector2(100, 0), new Vector2(70, 20), new Vector2(70, 10),
                new Vector2(64, 16), new Vector2(64, 18), new Vector2(60, 16), new Vector2(60, 20),
                new Vector2(60, 25), new Vector2(55, 25), new Vector2(60, 30), new Vector2(55, 40), 
                new Vector2(0, 100), new Vector2(50, 50), new Vector2(60, 50), new Vector2(60, 60), 
                new Vector2(60, 70), new Vector2(65, 65), new Vector2(65, 75), new Vector2(100, 100), 
                new Vector2(30, 28) },
            };
        }

        public static IEnumerable<object[]> ConnectDataGen2()
        {
            var startGoals = new Vector2[][]
            {
                new Vector2[] { new Vector2(30, 35), new Vector2(70, 80) },
                new Vector2[] { new Vector2(55, 55), new Vector2(58, 52) },
                new Vector2[] { new Vector2(29.4f, 83.9f), new Vector2(82.3f, 53f) },
                new Vector2[] { new Vector2(61.5f, 58.8f), new Vector2(90.8f, 36f) },
                new Vector2[] { new Vector2(98.5f, 3.6f), new Vector2(33.2f, 3.4f) },
                new Vector2[] { new Vector2(84.2f, 56.8f), new Vector2(64.5f, 51.6f) },
                new Vector2[] { new Vector2(0.5f, 36f), new Vector2(85f, 6.2f) },
                new Vector2[] { new Vector2(76f, 14f), new Vector2(2.3f, 95.5f) },
                new Vector2[] { new Vector2(89.7f, 58.1f), new Vector2(53f, 28.6f) }
            };
            // Precision of stored float intersection is not enough 
            // for some very precise collinear checks for vis tri edge,
            // but will still find correct path (consider removing AssertEqConnectEdges check)
            //new Vector2[] { new Vector2(34f, 5.6f), new Vector2(66.6f, 0.1f) }
            for (int i = 0; i < startGoals.Length; i++)
            {
                var startGoal = startGoals[i];
                yield return new object[] { startGoal[0], startGoal[1], false, null, null, null };
            }

            for (int i = 0; i < startGoals.Length; i++)
            {
                var startGoal = startGoals[i];
                yield return new object[] { startGoal[0], startGoal[1], true, null, null, null };
            }
        }

        public static IEnumerable<object[]> ConnectRandDataGen()
        {
            var random = new Random(0);
            var startGoals = new Vector2[2000];
            for (int i = 0; i < startGoals.Length; i++)
            {
                startGoals[i] = new Vector2(NextCoordinate(random), NextCoordinate(random));
            }
            for (int b = 0; b < 2; b++)
            {
                for (int i = 0; i < startGoals.Length; i += 2)
                {
                    var start = startGoals[i];
                    var goal = startGoals[i + 1];
                    yield return new object[] { start, goal, b % 2 != 0, null, null, null };
                }
            }
        }

        // Generates number between 0 and 100 rounded to 1 decimal
        private static float NextCoordinate(Random random)
        {
            double r = random.NextDouble() * 100;
            r = ((int)(r * 10)) / 10.0;
            return (float)r;
        }

        [Theory]
        [InlineData(false)]
        [InlineData(true)]
        public void VerifyConnectAllVertices(bool reduced)
        {
            var vertices = new List<Vector2>();
            for (int i = 0; i < obstacles.vs.Count; i++)
            {
                if (obstacles.IsDescriptor(i))
                    continue;
                for (int j = 0; j < obstacles.vs.Count; j++)
                {
                    if (obstacles.IsDescriptor(j))
                        continue;
                    /*if (reduced)
                        Console.WriteLine(obstacles.V(i) + " " + obstacles.V(j));*/
                    VerifyConnectEdges(obstacles.V(i), obstacles.V(j), reduced);
                }
            }
        }

        [Theory]
        [MemberData(nameof(ConnectDataGen))]
        [MemberData(nameof(ConnectDataGen2))]
        [MemberData(nameof(ConnectRandDataGen))]
        public void VerifyConnectEdges(Vector2 start, Vector2 goal, 
            bool reduced, List<Vector2> pathExpect = null, 
            List<Vector2> startEdgesExpect = null, List<Vector2> goalEdgesExpect = null)
        {
            VisibilityGraph(reduced, true);
            /*var vis = allVisTriEdges[35];
            for (int i = 0; i < vis.Count; i++)
            {
                Console.WriteLine(visVertices.V(vis[i].GetClosestVId()));
                Console.WriteLine(vis[i].intersection);
            }*/
            //Console.WriteLine(visVertices.V(vis[3].GetClosestVId()));
            var path1 = new List<Vector2>();
            rpsConnector.Initialize(new RotationalPlaneSweep(rps.Reduced, false));
            VgFindPath(start, goal, path1, rpsConnector, startEdgesExpect, goalEdgesExpect);
            if (pathExpect == null)
                pathExpect = path1;
            else
                AssertEqPaths(pathExpect, path1);
            if (startEdgesExpect == null && !reduced)
            {
                startEdgesExpect = GetEdgesV(allEdges.Count);
                goalEdgesExpect = GetEdgesV(allEdges.Count + 1);
            }
            if (!reduced || startEdgesExpect == null)
            {
                var path2 = new List<Vector2>();
                dfsConnector.Initialize(obstacles, allVisTriEdges, new Vector2(50, 50), 100, fullVg: !reduced);
                VgFindPath(start, goal, path2, dfsConnector, startEdgesExpect, goalEdgesExpect);
                AssertEqPaths(pathExpect, path2);
            }
        }

        private void VisibilityGraph(bool reduced, bool storeTriangles)
        {
            if (rps != null && reduced == rps.Reduced && storeTriangles == rps.StoreTriangles)
                return;
            allEdges.Clear();
            allVisTriEdges.Clear();
            visBboxes.Clear();
            rps = new RotationalPlaneSweep(reduced, storeTriangles);
            rps.VisibilityGraph(polygons, ref obstacles, ref allEdges,
                ref allVisTriEdges, ref visBboxes);
        }

        private bool VgFindPath(Vector2 start, Vector2 goal, List<Vector2> path,
            IVgConnector connector, List<Vector2> startEdgesExpect,
            List<Vector2> goalEdgesExpect)
        {
            vg.Initialize(obstacles, allEdges, connector);
            path.Clear();
            bool found = vg.FindPath(new AStarPathfinder(obstacles.vs.Count + 2), start, goal,
                path);
            if (startEdgesExpect != null)
            {
                AssertEqConnectVgEdges(allEdges.Count, startEdgesExpect);
                AssertEqConnectVgEdges(allEdges.Count + 1, goalEdgesExpect);
            }
            return found;
        }

        private List<Vector2> GetEdgesV(int fromId)
        {
            var edges = new List<Vector2>();
            vg.BeginIterEdges(fromId);
            while (vg.MoveNext())
                edges.Add(obstacles.V(vg.CurrentEdge.vertexId));
            return edges;
        }

        private void AssertEqConnectVgEdges(int fromId, List<Vector2> edgesExpect)
        {
            AssertEq(edgesExpect, GetEdgesV(fromId));
        }

        private void AssertEqPaths(List<Vector2> pathExpect, List<Vector2> path)
        {
            double costExpect = MathExt.PathCost(pathExpect);
            double costActual = MathExt.PathCost(path);
            /*if (!MathExt.Approximately(costExpect, costActual, MathExt.Epsilon))
            {
                for (int i = 0; i < pathExpect.Count; i++)
                {
                    Console.WriteLine(pathExpect[i]);
                }
            }*/
            AssertApprox(costExpect, costActual, MathExt.Epsilon);
        }

        private void AssertEq(List<Vector2> expected, List<Vector2> actual)
        {
            if (expected.Count != actual.Count)
            {
                for (int i = 0; i < actual.Count; i++)
                {
                    if (!expected.Contains(actual[i]))
                        Console.WriteLine(actual[i]);
                }

                for (int i = 0; i < expected.Count; i++)
                {
                    if (!actual.Contains(expected[i]))
                        Console.WriteLine(expected[i]);
                }
            }
            Assert.Equal(expected.Count, actual.Count);
            actual.Sort(Vector2Comparer.Instance);
            expected.Sort(Vector2Comparer.Instance);
            /*for (int i = 0; i < edgesExpect.Count; i++)
            {
                if (!edges.Contains(edgesExpect[i]))
                    Console.WriteLine(edgesExpect[i]);
            }*/
            /*Console.WriteLine("Log");
            for (int i = 0; i < numEdges; i++)
            {
                Console.WriteLine(edges[i]);
            }*/
            for (int i = 0; i < expected.Count; i++)
                AssertApprox(expected[i], actual[i]);
        }

        private class Vector2Comparer : Comparer<Vector2>
        {
            public static readonly Vector2Comparer Instance = new Vector2Comparer();

            public override int Compare(Vector2 value1, Vector2 value2)
            {
                if (value1.x == value2.x)
                    return value1.y.CompareTo(value2.y);
                return value1.x.CompareTo(value2.x);
            }
        }

        public static IEnumerable<object[]> VisibleDataGen()
        {
            yield return new object[] { 1, true, 0, TriO.Cw | TriO.InAngle | TriO.InTri, 60, 60 };
            yield return new object[] { 1, false, 1, TriO.Cw | TriO.InAngle, 60, 80 };
            yield return new object[] { 1, true, 1, TriO.InAngle | TriO.InTri, 62, 60 };
            yield return new object[] { 1, false, 1, TriO.InAngle, 65, 80 };
            yield return new object[] { 1, true, 2, TriO.Ccw | TriO.InAngle, 70, 80 };
            yield return new object[] { 1, true, 4, TriO.Ccw, 100, 0 };
            yield return new object[] { 1, true, 7, TriO.Ccw, 60, 0 };
            yield return new object[] { 1, true, 9, TriO.Cw, 55, 40 };
            yield return new object[] { 1, true, 10, TriO.Cw, 0, 0 };
            yield return new object[] { 1, false, 12, TriO.Cw, 40, 70 };
            yield return new object[] { 2, true, 2, TriO.InAngle | TriO.InTri, 70, 70 };
            yield return new object[] { 2, true, 3, TriO.Ccw, 80, 50 };
            yield return new object[] { 2, true, 2, TriO.Ccw | TriO.InAngle | TriO.InTri, 100, 100 };
            yield return new object[] { 2, false, 3, TriO.Ccw | TriO.InAngle, 140, 150 };
            yield return new object[] { 7, false, 6, TriO.Cw | TriO.Ccw | TriO.InAngle, 60, -10 };
            yield return new object[] { 7, true, 7, TriO.Cw | TriO.Ccw | TriO.InAngle | TriO.InTri, 60, 0 };
            yield return new object[] { 7, true, 0, TriO.Cw | TriO.Ccw, 60, 60 };
            yield return new object[] { 7, false, 8, TriO.Ccw, 50, 10 };
            yield return new object[] { 7, false, 5, TriO.Cw, 70, 10 };
        }

        [Theory]
        [MemberData(nameof(VisibleDataGen))]
        public void CompareVisTriP(int edgeIndex, bool visibleExpect, int edgeIndexExpect, 
            TriO expectedTriO, float px, float py)
        {
            VisibleVertices(false, true);
            var p = new Vector2(px, py);
            Assert.Equal(expectedTriO, VisibilityGraphExt.CompareVisTriP(obstacles, visTriEdges[edgeIndex], 
                visTriEdges[edgeIndex + 1],
                obstacles.V(2), p));
            int containingEdgeIndex;
            TriO orientation;
            Assert.Equal(visibleExpect, 
                VisibilityGraphExt.IsPVisible(obstacles, visTriEdges, 2, p, true, out containingEdgeIndex,
                out orientation));
            Assert.Equal(edgeIndexExpect, containingEdgeIndex);
        }

        private void AssertEqVisTriEdge(int visTriIndex, Vector2 closestV, Vector2 intersection)
        {
            var visTriEdge = visTriEdges[visTriIndex];
            AssertApprox(obstacles.vs[visTriEdge.GetClosestVId()], closestV);
            AssertApprox(visTriEdge.intersection, intersection);
        }

        private void AssertEqEdge(int edgeIndex, Vector2 endV)
        {
            var edge = edges[edgeIndex];
            AssertApprox(obstacles.vs[edge.vertexId], endV);
        }

        private void AssertApprox(float value1, float value2, float epsilon = MathExt.Epsilon)
        {
            Assert.True(MathExt.Approximately(value1, value2, epsilon),
                "!Approx(" + value1 + "," + value2 + ")");
        }

        private void AssertApprox(double value1, double value2, double epsilon = MathExt.EpsilonD)
        {
            Assert.True(MathExt.Approximately(value1, value2, epsilon),
                "!Approx(" + value1 + "," + value2 + ")");
        }

        private void AssertApprox(Vector2 value1, Vector2 value2, float epsilon = MathExt.Epsilon)
        {
            Assert.True(MathExt.Approximately(value1, value2, epsilon), 
                "!Approx(" + value1 + "," + value2 + ")");
        }
    }
}
