using CGALDotNet;
using CGALDotNet.Polygons;
using CGALDotNetGeometry.Numerics;
using PolyBoolOpMartinez;
using System;
using System.Collections.Generic;

namespace Pathfinding.VG
{
    public static class PolygonUtils
    {
        public static void GetTraversablePolygonsAndTEV(Polygon union, out List<PolygonWithHoles2<EIK>> traversablePolys,
            out List<IntPtr> triExpPtrs)
        {
            triExpPtrs = new List<IntPtr>();
            traversablePolys = new List<PolygonWithHoles2<EIK>>();
            var holes = new HashSet<int>();
            for (int i = 0; i < union.Ncontours; i++)
            {
                var traversablePoly = union.Contour(i);
                // Not traversable, continue
                if (traversablePoly.Clockwise())
                    continue;
                var points = new Point2d[traversablePoly.Nvertices];
                for (int j = 0; j < traversablePoly.Nvertices; j++)
                {
                    var vertex = traversablePoly.Vertex(j);
                    points[j] = new Point2d(vertex.x, vertex.y);
                }
                var unionCppPoly = new PolygonWithHoles2<EIK>(points);
                for (int j = 0; j < traversablePoly.Nholes; j++)
                {
                    var obstacle = union.Contour(traversablePoly.Hole(j));
                    points = new Point2d[obstacle.Nvertices];
                    for (int k = 0; k < obstacle.Nvertices; k++)
                    {
                        var vertex = obstacle.Vertex(k);
                        points[k] = new Point2d(vertex.x, vertex.y);
                    }
                    //Array.Reverse(points);
                    unionCppPoly.AddHole(new Polygon2<EIK>(points));
                    holes.Add(traversablePoly.Hole(j));
                }
                traversablePolys.Add(unionCppPoly);
            }

            for (int i = 1; i < union.Ncontours; i++)
            {
                var obstacle = union.Contour(i);
                if (!holes.Contains(i) && obstacle.Clockwise())
                {
                    var v = obstacle.Vertex(0);
                    int polyIndex = PolygonVisibility<EIK>.Instance.FindIndexContainsPoint(new Point2d(v.x, v.y), traversablePolys);
                    var points = new Point2d[obstacle.Nvertices];
                    for (int k = 0; k < obstacle.Nvertices; k++)
                    {
                        var vertex = obstacle.Vertex(k);
                        points[k] = new Point2d(vertex.x, vertex.y);
                    }
                    traversablePolys[polyIndex].AddHole(new Polygon2<EIK>(points));
                }
            }

            for (int i = 0; i < traversablePolys.Count; i++)
            {
                triExpPtrs.Add(PolygonVisibility<EIK>.Instance.GetLocatorAndTEV(traversablePolys[i]));
            }
        }
    }
}
