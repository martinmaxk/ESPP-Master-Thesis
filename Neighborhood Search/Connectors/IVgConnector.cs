using System;
using System.Collections.Generic;

namespace Pathfinding.VG
{
    public interface IVgConnector
    {
        bool FindEdges(int sourceId, int goalId,
            VisVertices visVertices, DynamicArray<DynamicArray<Edge>> allEdges);

        bool LineOfSight(int startId, int endId);
    }

    public abstract class BaseVgConnector : IVgConnector
    {
        public abstract bool FindEdges(int sourceId, int goalId,
            VisVertices visVertices, DynamicArray<DynamicArray<Edge>> allEdges);

        public virtual bool LineOfSight(int fromId, int toId)
        {
            return false;
        }
    }
}
