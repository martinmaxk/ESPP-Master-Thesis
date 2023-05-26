using System.Collections.Generic;
using System.Linq;
using MathTools;

namespace Pathfinding.VG
{
    /// <summary>
    /// QuadTree that stores points to quickly find a visible point from a start point
    /// </summary>
    public class QuadTreeVG
    {
        private Vector2 mid;
        // id's of points stored within this cell 
        // only bottom-layer (children == null) cells contains points
        private DynamicArray<int> ids;
        private QuadTreeVG[] children;
        private const int Capacity = 4;

        public QuadTreeVG(Vector2 mid)
        {
            this.mid = mid;
            ids = new DynamicArray<int>(0);
        }

        private QuadTreeVG(Vector2 mid, DynamicArray<int> reuseArr)
        {
            this.mid = mid;
            this.ids = reuseArr;
        }

        // cellExtent is half the width the cell spans
        public void Add(Query query, float cellExtent)
        {
            if (children == null)
            {
                // Split content to 4 child cells when reaching capacity
                // but stop when cell extent becomes too small to matter
                if (ids.Count == Capacity && cellExtent > 32)
                    Subdivide(query, cellExtent);
                else
                {
                    ids.Add(query.startId);
                    return;
                }
            }
            int childIndex = GetChildIndexAt(query.start);
            children[childIndex].Add(query, cellExtent / 2);
        }

        public void Clear()
        {
            ids.Clear();
            if (children == null)
                return;
            for (int i = 0; i < children.Length; i++)
                children[i].Clear();
        }

        public void Move(Vector2 mid)
        {
            this.mid = mid;
            if (children == null)
                return;
            for (int i = 0; i < children.Length; i++)
                children[i].Move(mid / 2);
        }

        public int FindVisiblePoint(Query query)
        {
            int id;
            if (children == null)
            {
                for (int i = 0; i < ids.Count; i++)
                {
                    id = ids[i];
                    if (query.finder.StopSearch(query.startId, id))
                        return id;
                }
                return -1;
            }

            int childIndex = GetChildIndexAt(query.start);
            id = children[childIndex].FindVisiblePoint(query);

            // Check adjacent child cells if no luck (3 other cells)
            int curChildIndex;
            for (curChildIndex = 0; id == -1 && curChildIndex < childIndex; curChildIndex++)
                id = children[curChildIndex].FindVisiblePoint(query);
            for (curChildIndex++; id == -1 && curChildIndex < children.Length; curChildIndex++)
                id = children[curChildIndex].FindVisiblePoint(query);

            return id;
        }

        /// <summary>
        /// Get index of the child cell the point lies within
        /// </summary>
        private int GetChildIndexAt(Vector2 point)
        {
            if (point.y < mid.y)
                return point.x < mid.x ? 0 : 1;
            return point.x < mid.x ? 2 : 3;
        }

        private void Subdivide(Query query, float cellExtent)
        {
            float childExtent = cellExtent / 2;
            // Create TL, TR, BL, BR child cells
            children = new QuadTreeVG[4];
            children[1] = new QuadTreeVG(new Vector2(mid.x + childExtent, mid.y - childExtent));
            children[2] = new QuadTreeVG(new Vector2(mid.x - childExtent, mid.y + childExtent));
            children[3] = new QuadTreeVG(new Vector2(mid.x + childExtent, mid.y + childExtent));
            int tempId = query.startId;
            for (int i = 0; i < ids.Count; i++)
            {
                query.startId = ids[i];
                query.start = query.finder.GetPoint(ids[i]);
                int childIndex = GetChildIndexAt(query.start);
                if (childIndex != 0)
                {
                    ids.SwapRemoveAt(i--);
                    children[childIndex].Add(query, childExtent);
                }
            }
            children[0] = new QuadTreeVG(new Vector2(mid.x - childExtent, mid.y - childExtent), ids);
            //if (ids.Count == Capacity && childExtent > 32)
                //children[0].Subdivide(query, childExtent);
            query.startId = tempId;
            query.start = query.finder.GetPoint(query.startId);
            ids = default(DynamicArray<int>);
        }

        public class Query
        {
            public Vector2 start;
            public int startId;
            public IPointFinder finder;
        }
    }

    public interface IPointFinder
    {
        bool StopSearch(int startId, int otherId);
        Vector2 GetPoint(int id);
    }
}
