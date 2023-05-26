using pgraphBase;
using System;

namespace org.jgrapht.traverse
{
    /**
     * Created with IntelliJ IDEA.
     * User: dindar.oz
     * Date: 3/29/13
     * Time: 9:20 AM
     * To change this template use File | Settings | File Templates.
     */
    public class OctileDistanceHeuristic : Heuristic<BaseVertex> {
    public const double ROOT_TWO = 1.414213562f;
    private BaseVertex target;

    public OctileDistanceHeuristic(BaseVertex t)
    {
        target = t;
    }

    public double getValue(BaseVertex s)
    {
        if (target == null)
            return 0;
        return getValue(s, target);
    }

    public double getValue(BaseVertex s, BaseVertex t)
    {
        if (s == null)
            return 0;

        return getValue(
                (int)s.pos.x, (int)s.pos.y,
                (int)t.pos.x, (int)t.pos.y);
    }

    public double getValue(int x1, int y1, int x2, int y2)
    {
        int dx = Math.Abs(x1 - x2);
        int dy = Math.Abs(y1 - y2);
        int min = (dx < dy) ? dx : dy;
        double octileDistance = ((int)Math.Abs(dx - dy)) + min * ROOT_TWO;
        return octileDistance;
    }
}
}
