using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Runtime.CompilerServices;
using MathTools;

namespace Pathfinding.VG
{
    // Type punning (interpret same bytes as either float or integer)
    [StructLayout(LayoutKind.Explicit)]
    public struct FloatInt
    {
        [FieldOffset(0)]
        public float f;
        [FieldOffset(0)]
        public int i;
    }

    // Vertices of polygons in clockwise order, merged into 1 array
    public struct VisVertices
    {
        public DynamicArray<Vector2> vs;
        // Place points that are not polygon vertices at the end
        public int lastPolyEndI;
        private const float DescVal = float.NegativeInfinity;

        public VisVertices(DynamicArray<Vector2> vs)
        {
            this.vs = vs;
            this.lastPolyEndI = vs.Count;
        }

        public void Reset(List<List<Vector2>> polygons)
        {
            Reset(polygons, 0, polygons.Count);
        }

        public void Reset(List<List<Vector2>> polygons, int startIndex, int polyCount)
        {
            this.lastPolyEndI = vs.Count;
            int count = 0;
            for (int i = startIndex; i < polyCount; i++)
            {
                var polygon = polygons[i];
                count += polygon.Count + 2;
            }
            if (vs.arr.Length < count + 2)
                vs.arr = new Vector2[count + 2];
            vs.ResizeAndExtendTo(count);
            count = 0;
            for (int i = startIndex; i < polyCount; i++)
            {
                var polygon = polygons[i];
                int head = count++;
                for (int j = 0; j < polygon.Count; j++)
                    vs[count++] = polygon[j];

                SetDescriptor(head, count - 1);
                SetDescriptor(count++, head + 1);
            }
        }

        public void AutoSetLastPolyEndI()
        {
            lastPolyEndI = vs.Count - 1;
            while (!IsDescriptor(lastPolyEndI))
                lastPolyEndI--;
        }

        // Is point with id a vertex on a polygon?
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool IsPoly(int id)
        {
            return id <= lastPolyEndI;
        }

        // Is vertex a polygon header/footer where index of footer/header is in v.y?
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsDescriptor(Vector2 v)
        {
            return v.x == DescVal;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool IsDescriptor(int vId)
        {
            return IsDescriptor(vs[vId]);
        }

        // Each polygon has a header and footer where you can obtain the index to 
        // first/last point. These entries are marked by x = DescVal
        // We do this to avoid casting because float cannot represent all integers
        public static Vector2 Descriptor(int otherEndI)
        {
            return new Vector2(DescVal, new FloatInt() { i = otherEndI }.f);
        }

        public void SetDescriptor(int vId, int otherEndI)
        {
            vs[vId] = Descriptor(otherEndI);
        }

        // Get prev/next point in polygon
        // If id is at a descriptor, then return its index to opposite end
        public int PrevNextID(int prevNextId)
        {
            var descriptor = vs[prevNextId];
            if (IsDescriptor(descriptor))
                return new FloatInt() { f = descriptor.y }.i;
            return prevNextId;
        }

        public int PrevID(int polyPId)
        {
            return PrevNextID(polyPId - 1);
        }

        public int NextID(int polyPId)
        {
            return PrevNextID(polyPId + 1);
        }

        public Vector2 PrevV(int polyPId)
        {
            return vs[PrevID(polyPId)];
        }

        public Vector2 NextV(int polyPId)
        {
            return vs[NextID(polyPId)];
        }

        public Vector2 V(int polyPId)
        {
            return vs[polyPId];
        }

        public void E(int edgeId, out Vector2 v1, out Vector2 v2)
        {
            v1 = V(edgeId);
            v2 = NextV(edgeId);
        }

        public bool LineToEdge(Vector2 a1, Vector2 a2, int edgeId, out Vector2 intersection)
        {
            return MathExt.LineToLine(a1, a2, V(edgeId), NextV(edgeId), out intersection);
        }

        // Is p in non-taut region of polyPId?
        public bool IsInNonTautRegion(int polyPId, Vector2 p)
        {
            // Translate all points by -center, so center is origo (0, 0)
            var center = V(polyPId);
            // Mirror the angle points in center point, i.e. origo, so negate
            var angleP1 = -(NextV(polyPId) - center);
            var angleP2 = -(PrevV(polyPId) - center);
            p -= center;
            // is p - polyP in clockwise angle between 
            // -(polyPId nextPolyId) and -(polyId prevPolyId)
            return MathExt.PointInCwAngle(Vector2.Zero, angleP1, angleP2, p);
        }

        public Dictionary<Vector2, int> GetVertexToIdMap()
        {
            return GetVertexToIdMap(vs.Count);
        }

        public Dictionary<Vector2, int> GetVertexToIdMap(int count)
        {
            var vertexToId = new Dictionary<Vector2, int>();
            for (int i = 0; i < count; i++)
            {
                if (IsDescriptor(i))
                    continue;
                var vertex = vs[i];
                vertexToId.Add(vertex, i);
            }
            return vertexToId;
        }
    }
}
