using MathTools;
using System;
using System.Collections.Generic;

namespace pgraphBase
{
    /**
 * Created with IntelliJ IDEA.
 * User: dindaro
 * Date: 13.01.2013
 * Time: 20:01
 * To change this template use File | Settings | File Templates.
 */
    public class BaseVertex : IComparable<BaseVertex>
    {
        public long id;
        public Vector2D pos;
        public int hash;

        /**
         * Outgoing Edges
         */
        List<BaseEdge> outgoings = new List<BaseEdge>();

        /**
         * Incoming Edges
         */
        List<BaseEdge> incomings = new List<BaseEdge>();

        /**
         * Union of outgoings and incomings
         */
        List<BaseEdge> touchings = new List<BaseEdge>();


        /**
         * Constructor
         * @param id
         * @param pos
         */
        public BaseVertex(long id, Vector2D pos)
        {
            this.id = id;
            this.pos = pos;
            this.hash = base.GetHashCode();
        }

        /**
         * Getter for outgoings
         * @return
         */
        public List<BaseEdge> getOutgoings()
        {
            return outgoings;
        }


        /**
         * Getter for outgoings
         * @return
         */
        public List<BaseVertex> getOutgoingNeighbors()
        {
            List<BaseVertex> on = new List<BaseVertex>();

            foreach (var e in outgoings)
            {
                on.Add(e.end);
            }
            return on;
        }

        /**
         * Getter for incomings
         * @return
         */
        public List<BaseEdge> getIncomings()
        {
            return incomings;
        }

        /**
         * Tests if this vertex is equal to another
         * @param o
         * @return
         */
        public bool equals(BaseVertex o)
        {
            return id == o.id;
        }

        /**
         * Returns the edge outgoing to the given target
         * @param target
         * @return
         */
        public BaseEdge getOutgoingTo(BaseVertex target)
        {
            if (target == null)
                return null;
            foreach (var e in outgoings)
            {
                if (e.end.equals(target))
                    return e;
            }
            return null;
        }

        /**
         * Returns the edge incoming from the given start
         * @param start
         * @return
         */
        public BaseEdge getIncomingFrom(BaseVertex start)
        {
            foreach (var e in incomings)
            {
                if (e.start.equals(start))
                    return e;
            }
            return null;
        }

        /**
         * Adds new outgoing edge
         * @param e
         */
        void addOutgoing(BaseEdge e)
        {
            outgoings.Add(e);
            touchings.Add(e);
        }

        /**
         * Adds new incoming edge
         * @param e
         */
        void addIncoming(BaseEdge e)
        {
            incomings.Add(e);
            touchings.Add(e);
        }

        /**
         * removes the given incoming edge
         * @param e
         */
        void removeIncoming(BaseEdge e)
        {
            incomings.Remove(e);
            touchings.Remove(e);
        }

        /**
         * Removes the given outgoing edge
         * @param e
         */
        void removeOutgoing(BaseEdge e)
        {
            outgoings.Remove(e);
            touchings.Remove(e);
        }


        /**
         * Compare this edge to another
         * @param o
         * @return
         */
        public int CompareTo(BaseVertex o)
        {
            return this.id.CompareTo(o.id);  //To change body of implemented methods use File | Settings | File Templates.
        }

        public override int GetHashCode()
        {
            return this.hash;
        }

        public override string ToString()
        {
            return "BaseVertex " + id + "; " + pos.ToString();

        }
    }
}
