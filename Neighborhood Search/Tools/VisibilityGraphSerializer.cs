using MathTools;
using PolyBoolOpMartinez;
using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using static Pathfinding.VG.Program;

namespace Pathfinding.VG
{
    public static class VisibilityGraphSerializer
    {
        public static void Serialize(BinaryWriter writer, VisVertices visVertices, DynamicArray<DynamicArray<Edge>> allEdges,
            DynamicArray<DynamicArray<VisTriEdge>> allVisTriEdges, Vector2 levelMid, Bbox levelBbox)
        {
            writer.Write(visVertices.vs.Count);
            for (int i = 0; i < visVertices.vs.Count; i++)
            {
                writer.Write(visVertices.vs[i].x);
                writer.Write(visVertices.vs[i].y);
            }

            writer.Write(allEdges.Count);
            for (int i = 0; i < allEdges.Count; i++)
            {
                writer.Write(allEdges[i].Count);
                for (int j = 0; j < allEdges[i].Count; j++)
                {
                    var edge = allEdges[i][j];
                    writer.Write(edge.vertexId);
                    writer.Write(edge.cost);
                }
            }

            writer.Write(allVisTriEdges.Count);
            for (int i = 0; i < allVisTriEdges.Count; i++)
            {
                writer.Write(allVisTriEdges[i].Count);
                for (int j = 0; j < allVisTriEdges[i].Count; j++)
                {
                    var visTriEdge = allVisTriEdges[i][j];
                    writer.Write(visTriEdge.intersection.x);
                    writer.Write(visTriEdge.intersection.y);
                    writer.Write(visTriEdge.closestVAux);
                }
            }
            writer.Write(levelMid.x);
            writer.Write(levelMid.y);
            writer.Write(levelBbox.xmin);
            writer.Write(levelBbox.ymin);
            writer.Write(levelBbox.xmax);
            writer.Write(levelBbox.ymax);
        }

        public static void Deserialize(BinaryReader reader, out VisibilityGraph visibilityGraph, out Bbox levelBbox,
            out IVgConnector connector, out AStarPathfinder pathfinder)
        {
            var visVertices = new VisVertices();
            visVertices.vs = new DynamicArray<Vector2>(reader.ReadInt32());
            for (int i = 0; i < visVertices.vs.Capacity; i++)
                visVertices.vs.Add(new Vector2(reader.ReadSingle(), reader.ReadSingle()));

            var allEdges = new DynamicArray<DynamicArray<Edge>>(reader.ReadInt32());
            for (int i = 0; i < allEdges.Capacity; i++)
            {
                allEdges.Add(new DynamicArray<Edge>(reader.ReadInt32()));
                for (int j = 0; j < allEdges[i].Capacity; j++)
                    allEdges.arr[i].Add(new Edge(reader.ReadInt32(), reader.ReadSingle()));
            }

            var allVisTriEdges = new DynamicArray<DynamicArray<VisTriEdge>>(reader.ReadInt32());
            for (int i = 0; i < allVisTriEdges.Capacity; i++)
            {
                allVisTriEdges.Add(new DynamicArray<VisTriEdge>(reader.ReadInt32()));
                for (int j = 0; j < allVisTriEdges[i].Capacity; j++)
                    allVisTriEdges.arr[i].Add(new VisTriEdge(new Vector2(reader.ReadSingle(), reader.ReadSingle()), reader.ReadInt32()));
            }
            var levelMid = new Vector2(reader.ReadSingle(), reader.ReadSingle());
            levelBbox = new Bbox(reader.ReadSingle(), reader.ReadSingle(), 
                reader.ReadSingle(), reader.ReadSingle());

            float maxLevelDim = Math.Max(levelBbox.xmax - levelBbox.xmin, levelBbox.ymax - levelBbox.ymin);
            var dfsConnector = new TimeDfsVgConnector();
            dfsConnector.Initialize(visVertices, allVisTriEdges, levelMid,
                maxLevelDim);
            connector = dfsConnector;
            visibilityGraph = new VisibilityGraph();
            visibilityGraph.Initialize(visVertices, allEdges, connector);
            pathfinder = new AStarPathfinder(visVertices.vs.Count + 2);
        }
    }
}
