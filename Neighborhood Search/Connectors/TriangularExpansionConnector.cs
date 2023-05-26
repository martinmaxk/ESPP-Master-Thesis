using CGALDotNet;
using CGALDotNet.Polygons;
using MathTools;
using System;
using System.Collections.Generic;

namespace Pathfinding.VG
{
    public class TriangularExpansionConnector : BaseVgConnector
    {
        protected TriangularExpansion triangularExpansion;
        private List<IntPtr> triExpPtrs;
        private List<PolygonWithHoles2<EIK>> traversablePolys;
        private Dictionary<Vector2, int> vertexToId;

        private DynamicArray<VisTriEdge> visTriEdges = new DynamicArray<VisTriEdge>(2);

        public void Initialize(TriangularExpansion triangularExpansion, List<PolygonWithHoles2<EIK>> traversablePolys,
            List<IntPtr> triExpPtrs, Dictionary<Vector2, int> vertexToId)
        {
            this.triangularExpansion = triangularExpansion;
            this.traversablePolys = traversablePolys;
            this.triExpPtrs = triExpPtrs;
            this.vertexToId = vertexToId;
        }

        public override bool FindEdges(int sourceId, int goalId, VisVertices visVertices,
            DynamicArray<DynamicArray<Edge>> allEdges)
        {
            bool isGoal = sourceId == goalId;
            triangularExpansion.VisibleVertices(sourceId, visVertices, traversablePolys, triExpPtrs, vertexToId, 
                ref allEdges.arr[sourceId], ref visTriEdges, out PolyBoolOpMartinez.Bbox _);
            var sourceEdges = allEdges[sourceId];
            if (!isGoal)
            {
                if (VisibilityGraphExt.IsPVisible(visVertices, visTriEdges, sourceId, goalId, hasBlockedTri: false))
                {
                    // Add edge from start to goal
                    float dist = Vector2.Distance(visVertices.V(sourceId),
                        visVertices.V(goalId));
                    sourceEdges.Add(new Edge(goalId, dist));
                    allEdges[sourceId] = sourceEdges;

                    var goalEdges = allEdges[goalId];
                    goalEdges.Add(new Edge(sourceId, dist));
                    allEdges[goalId] = goalEdges;
                }
                return sourceEdges.Count > 0;
            }
            // Connect to goal vertex (if isGoal)
            for (int i = 0; i < sourceEdges.Count; i++)
            {
                var edge = sourceEdges[i];
                // Start has not been connected yet (avoid duplicate edge)
                if (edge.vertexId == visVertices.vs.Count - 2)
                    continue;
                var endEdges = allEdges[edge.vertexId];
                endEdges.Add(new Edge(sourceId, edge.cost));
                allEdges[edge.vertexId] = endEdges;
            }
            return sourceEdges.Count > 0;
        }
    }
}
