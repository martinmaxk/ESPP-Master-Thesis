using Debugging;

namespace Pathfinding.VG
{
    public class PlaneSweepConnector : BaseVgConnector
    {
        private RotationalPlaneSweep rotPlaneSweep;

        public void Initialize(RotationalPlaneSweep rotPlaneSweep)
        {
            this.rotPlaneSweep = rotPlaneSweep;
            Debug.Assert(!rotPlaneSweep.StoreTriangles);
        }

        public override bool FindEdges(int sourceId, int goalId, VisVertices visVertices,
            DynamicArray<DynamicArray<Edge>> allEdges)
        {
            bool isGoal = sourceId == goalId;
            var unused = default(DynamicArray<VisTriEdge>);
            rotPlaneSweep.VisibleVertices(sourceId, visVertices, ref allEdges.arr[sourceId],
                ref unused, out PolyBoolOpMartinez.Bbox _);
            var sourceEdges = allEdges[sourceId];
            bool anyEdges = sourceEdges.Count > 0;
            if (!isGoal)
                return anyEdges;
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
            return anyEdges;
        }
    }
}
