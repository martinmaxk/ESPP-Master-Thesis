using MathTools;
using System;
using System.Collections.Generic;

namespace Pathfinding.VG
{
    // Find edges for a vertex, by finding a visible vertex 
    // and doing a dfs of the full vg from that vertex as 
    // long as there is visibility from source 
    public class DfsVgConnector : BaseVgConnector, IPointFinder
    {
        private VisVertices visVertices;
        private DynamicArray<DynamicArray<VisTriEdge>> allVisTriEdges;

        private DynamicArray<int> visitedVertices;
        private int sessionId;
        private Stack<int> openStack;
        private QuadTreeVG quadTreeVg;
        private QuadTreeVG.Query vertexQuery;
        // Angle spaces for vis triangles of start, created from edges
        private DynamicArray<int> startVisTriEdges = new DynamicArray<int>(2);

        private bool fullVg;

        public void Initialize(VisVertices visVertices,
            DynamicArray<DynamicArray<VisTriEdge>> allVisTriEdges, Vector2 levelMid, float cellExtent,
            bool fullVg = false)
        {
            this.fullVg = fullVg;
            this.visVertices = visVertices;
            this.allVisTriEdges = allVisTriEdges;
            if (visitedVertices.arr == null)
            {
                visitedVertices = new DynamicArray<int>(visVertices.vs.Count);
                sessionId = 1;
            }
            else
                visitedVertices.ResizeAndExtendTo(visVertices.vs.Count);
            if (quadTreeVg == null)
            {
                quadTreeVg = new QuadTreeVG(levelMid);
                vertexQuery = new QuadTreeVG.Query() { finder = this };
                openStack = new Stack<int>();
            }
            else
            {
                quadTreeVg.Clear();
                quadTreeVg.Move(levelMid);
            }
            for (int i = 0; i < visVertices.vs.Count; i++)
            {
                if (visVertices.IsDescriptor(i))
                    continue;
                vertexQuery.startId = i;
                vertexQuery.start = visVertices.V(i);
                quadTreeVg.Add(vertexQuery, cellExtent);
            }
        }

        /// <summary>
        /// Find the first visible vertex, then keep expanding by adding vertices at end of edges that 
        /// are visible from source vertex, this means we do not explore vertices that are not visible from source
        /// </summary>
        /// <param name="sourceId">The Id of the vertex we want to find visibility edges of</param>
        /// <param name="goalId">The Id of the goal vertex</param>
        public override bool FindEdges(int sourceId, int goalId,
            VisVertices visVertices, DynamicArray<DynamicArray<Edge>> allEdges)
        {
            bool isGoal = sourceId == goalId;
            this.visVertices = visVertices;
            startVisTriEdges.Clear();
            // Find a visible point
            var source = visVertices.V(sourceId);
            var sourceEdges = allEdges[sourceId];
            sourceEdges.Clear();
            vertexQuery.start = source;
            vertexQuery.startId = sourceId;
            int visibleVertexId = quadTreeVg.FindVisiblePoint(vertexQuery);
            // Debug check, did we skip another visible vertex Id among the ones we checked?
            /*for (int i = 0; i < visitedVertices.Count; i++)
                if (visitedVertices[i] == sessionId && i != visibleVertexId &&
                    LineOfSight(i, sourceId))
                    throw new System.Exception();*/
            // No edges, cannot connect p to any point in graph
            if (visibleVertexId == -1)
            {
                //throw new Exception(source / 32 + " " + visVertices.V(goalId) / 32);
                NextSessionId();
                return false;
            }
            // TODO: Make this BFS instead by using a queue?
            // DFS where we add vertices that are visible to an open stack
            openStack.Push(visibleVertexId);
            int numEdges = 0;
            while (openStack.Count > 0)
            {
                int vertexId = openStack.Pop();
                var vertex = visVertices.V(vertexId);
                if (fullVg || MathExt.IsConvex(visVertices.PrevV(vertexId), vertex, visVertices.NextV(vertexId)))
                {
                    float dist = Vector2.Distance(source, vertex);
                    // Connect to goal vertex (if isGoal)
                    if (isGoal)
                    {
                        var endEdges = allEdges[vertexId];
                        endEdges.Add(new Edge(sourceId, dist));
                        allEdges[vertexId] = endEdges;
                    }
                    sourceEdges.Add(new Edge(vertexId, dist));
                }
                if (!isGoal)
                    startVisTriEdges.Add(vertexId);
                if (numEdges < visVertices.vs.Count)
                {
                    // We need to explore concave and non-taut edges on each vertex as well
                    ExploreVertex(sourceId, allVisTriEdges[vertexId]);
                    numEdges += allVisTriEdges[vertexId].Count;
                    // When inspected n edges, swap over to O(n log n) algorithm 
                    // (check line of sight with every vertex) to avoid O(n^2)
                    if (numEdges >= visVertices.vs.Count)
                    {
                        for (int i = 0; i < visVertices.vs.Count - 2; i++)
                        {
                            if (visVertices.IsDescriptor(i))
                                continue;
                            if (visitedVertices[i] == sessionId)
                                continue;
                            visitedVertices[i] = sessionId;
                            if (LineOfSight(i, sourceId))
                                openStack.Push(i);
                        }
                    }
                }
            }

            // Check direct line of sight between start and goal
            if (!isGoal)
            {
                // Edges are not sorted for angle, so you need to do linear search and find
                // angleP with lowest ccw angle and angleP with lowest cw angle
                int ccwId = -1, cwId = -1;
                var goal = visVertices.V(goalId);
                for (int i = 0; i < startVisTriEdges.Count; i++)
                {
                    int id = startVisTriEdges[i];
                    var visV = visVertices.V(id);
                    double orientation1 = MathExt.OrientationD(source, goal, visV);
                    // Angle space is a line and there is LOS from source to visV, 
                    // implies LOS from source to goal if LOS from visV to goal
                    if (MathExt.Collinear(orientation1))
                    {
                        ccwId = cwId = id;
                        break;
                    }
                    else if (orientation1 < 0)
                    {
                        if (IsVisEdgeCloser(source, visV, ccwId, isCwAngle: false))
                            ccwId = id;
                    }
                    else if (IsVisEdgeCloser(source, visV, cwId, isCwAngle: true))
                        cwId = id;
                }
                // Goal is between two vis tri edges, defined by two vertices.
                // If goal is visible from both of these, then it may be visible from start
                // Note in the case where start, goal and cw or ccw are collinear, it is possible
                // the other cw/ccw is not visible from goal, yet start is visible from goal
                // in this case both start and end will be connected to cw/ccw, so path will be same length
                if (cwId != -1 && ccwId != -1 && LineOfSight(cwId, goalId) && LineOfSight(ccwId, goalId) &&
                    !LineToIncidentEdges(source, goal, ccwId) &&
                    !LineToIncidentEdges(source, goal, cwId))
                {
                    // Add edge from start to goal
                    float dist = Vector2.Distance(source, goal);
                    sourceEdges.Add(new Edge(goalId, dist));

                    var goalEdges = allEdges[goalId];
                    goalEdges.Add(new Edge(sourceId, dist));
                    allEdges[goalId] = goalEdges;
                }
            }

            NextSessionId();
            allEdges[sourceId] = sourceEdges;

            return true;
        }

        private bool LineToIncidentEdges(Vector2 a, Vector2 b, int id)
        {
            Vector2 intersection;
            return visVertices.LineToEdge(a, b, visVertices.PrevID(id), out intersection) ||
                   visVertices.LineToEdge(a, b, id, out intersection);
        }

        /// <summary>
        /// Returns true if center has smaller cw/ccw angle to visV than to otherV
        /// Break ties by picking closest vertex
        /// </summary>
        /// <param name="isCwAngle">Are visV and otherV on cw side of center? </param>
        private bool IsVisEdgeCloser(Vector2 center, Vector2 visV, int otherId,
            bool isCwAngle)
        {
            if (otherId == -1)
                return true;
            var otherV = visVertices.V(otherId);
            double orientation = MathExt.OrientationD(center, otherV, visV);
            return (MathExt.Collinear(orientation) &&
                Vector2.DistanceSquared(center, visV) < Vector2.DistanceSquared(center, otherV)) ||
                (isCwAngle ? orientation < 0 : orientation > 0);
        }

        // Add vertices in endEdges (edges from current vertex) that we can see from source to openStack
        private void ExploreVertex(int sourceId, DynamicArray<VisTriEdge> endEdges)
        {
            for (int i = 0; i < endEdges.Count - 1; i++)
            {
                int endVertexId = endEdges[i].GetClosestVId();

                if (visitedVertices[endVertexId] == sessionId)
                    continue;
                visitedVertices[endVertexId] = sessionId;

                var endVertex = visVertices.V(endVertexId);
                if (!LineOfSight(endVertexId, sourceId))
                    continue;

                openStack.Push(endVertexId);
            }
        }

        private void NextSessionId()
        {
            // Make sure we are reset
            if (sessionId == int.MaxValue)
            {
                for (int i = 0; i < visitedVertices.Count; i++)
                    visitedVertices[i] = 0;
                sessionId = 0;
            }
            sessionId++;
        }

        public bool StopSearch(int startId, int foundId)
        {
            visitedVertices[foundId] = sessionId;
            return LineOfSight(foundId, startId);
        }

        public override bool LineOfSight(int fromId, int toId)
        {
            var visTriEdges = allVisTriEdges[fromId];
            return VisibilityGraphExt.IsPVisible(visVertices, visTriEdges, fromId, toId);
        }

        public Vector2 GetPoint(int id)
        {
            return visVertices.V(id);
        }
    }
}
