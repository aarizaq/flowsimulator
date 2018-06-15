//
// Copyright (C) 2016 Alfonso Ariza, Malaga University
// Copyright (C) 2017 Alfonso Ariza
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//

#ifndef __DIJKSTRA_K_SHORTEST_FUZZY__H__
#define __DIJKSTRA_K_SHORTEST_FUZZY__H__

#include <vector>
#include <map>
#include <set>
#include <algorithm>

#include <omnetpp.h>

using namespace omnetpp;

typedef int NodeId;

enum Metricts
{
    aditiveMin, concaveMin, aditiveMax, concaveMax
};
enum StateLabel
{
    perm, tent
};

#define UndefinedAddr -1

class DijkstraKshortestFuzzy
{
protected:
    static double alpha;

    struct FuzzyCost
    {
        double cost1 = 0;
        double cost2 = 0;
        double cost3 = 0;
        FuzzyCost() {}
        FuzzyCost(const double &c1,const double &c2,const double &c3) {cost1 = c1;cost2 = c2;cost3 = c3;}
        FuzzyCost& operator+=(const FuzzyCost& rhs)
        {
            this->cost1 += rhs.cost1;
            this->cost2 += rhs.cost2;
            this->cost3 += rhs.cost3;
            return *this; // return the result by reference
        }

        // friends defined inside class body are inline and are hidden from non-ADL lookup
        friend FuzzyCost operator+(FuzzyCost lhs, const FuzzyCost& rhs) // otherwise, both parameters may be const references
        {
            lhs += rhs;
            return lhs;
        }
        const double exp() const {
            return ((1 - DijkstraKshortestFuzzy::alpha) * cost1 + cost2 + DijkstraKshortestFuzzy::alpha * cost3);
        }
    };
public:
    static DijkstraKshortestFuzzy::FuzzyCost minimumCost;
    static DijkstraKshortestFuzzy::FuzzyCost maximumCost;
    typedef std::vector<NodeId> Route;
    typedef std::vector<Route> Kroutes;
protected:
    typedef std::map<NodeId, Kroutes> MapRoutes;
    MapRoutes kRoutesMap;

    class SetElem
    {
    public:
        NodeId iD;
        int idx;
        DijkstraKshortestFuzzy::FuzzyCost cost;
        SetElem()
        {
            iD = UndefinedAddr;
            idx = -1;
        }
        SetElem& operator=(const SetElem& val)
        {
            this->iD = val.iD;
            this->idx = val.idx;
            this->cost = val.cost;
            return *this;
        }
    };
    void addCost(FuzzyCost &, const FuzzyCost & a, const FuzzyCost & b);

    friend bool operator <(const DijkstraKshortestFuzzy::FuzzyCost&, const DijkstraKshortestFuzzy::FuzzyCost&);
    friend bool operator ==(const DijkstraKshortestFuzzy::FuzzyCost&, const DijkstraKshortestFuzzy::FuzzyCost&);
    friend bool operator !=(const DijkstraKshortestFuzzy::FuzzyCost&, const DijkstraKshortestFuzzy::FuzzyCost&);
    friend bool operator >(const DijkstraKshortestFuzzy::FuzzyCost&, const DijkstraKshortestFuzzy::FuzzyCost&);
    friend bool operator <(const DijkstraKshortestFuzzy::SetElem& x, const DijkstraKshortestFuzzy::SetElem& y);
    friend bool operator ==(const DijkstraKshortestFuzzy::SetElem& x, const DijkstraKshortestFuzzy::SetElem& y);
    friend bool operator >(const DijkstraKshortestFuzzy::SetElem& x, const DijkstraKshortestFuzzy::SetElem& y);

    class State
    {
    public:
        DijkstraKshortestFuzzy::FuzzyCost cost;
        NodeId idPrev;
        int idPrevIdx;
        StateLabel label;
        State();
        State(const FuzzyCost &cost);
        ~State();
    };

    struct Edge
    {
        NodeId last_node_; // last node to reach node X
        FuzzyCost cost;
        Edge()
        {
            cost = maximumCost;
        }
        virtual ~Edge()
        {
        }
        inline NodeId& last_node()
        {
            return last_node_;
        }
        virtual FuzzyCost& Cost()
        {
            return cost;
        }
    };


    typedef std::vector<DijkstraKshortestFuzzy::State> StateVector;
    typedef std::map<NodeId, DijkstraKshortestFuzzy::StateVector> RouteMap;
    typedef std::map<NodeId, std::vector<DijkstraKshortestFuzzy::Edge*> > LinkArray;
    LinkArray linkArray;
    RouteMap routeMap;
    NodeId rootNode;
    int K_LIMITE;
public:
    DijkstraKshortestFuzzy(int);
    virtual ~DijkstraKshortestFuzzy();
    virtual void setFromTopo(const cTopology *);

    virtual void setKLimit(int val)
    {
        if (val > 0)
            K_LIMITE = val;
    }
    virtual void initMinAndMax();
    virtual void cleanLinkArray();
    virtual void addEdge(const NodeId & dest_node, const NodeId & last_node, double cost1, double cost2, double cost3);
    virtual void deleteEdge(const NodeId &, const NodeId &);
    virtual Edge * removeEdge(const NodeId &, const NodeId &);
    virtual void setRoot(const NodeId & dest_node);
    virtual void run();
    virtual void runUntil(const NodeId &);
    virtual int getNumRoutes(const NodeId &nodeId);
    virtual bool getRoute(const NodeId &nodeId, std::vector<NodeId> &pathNode, int k = 0);
    virtual void setRouteMapK();
    virtual void getRouteMapK(const NodeId &nodeId, Kroutes &routes);
    virtual unsigned int commonLinks(const Route &S, const Route &Sp);
};

inline bool operator <(const DijkstraKshortestFuzzy::SetElem& x, const DijkstraKshortestFuzzy::SetElem& y)
{
    if (x.iD == y.iD)
        return false;
    if (x.cost == y.cost)
        return x.iD < y.iD;
    return x.cost < y.cost;
}

inline bool operator ==(const DijkstraKshortestFuzzy::SetElem& x, const DijkstraKshortestFuzzy::SetElem& y)
{
    if (x.iD == y.iD)
        return true;
    return x.cost == y.cost;
}

inline bool operator >(const DijkstraKshortestFuzzy::SetElem& x, const DijkstraKshortestFuzzy::SetElem& y)
{
    if (x.iD == y.iD)
        return (false);
    return (x.cost > y.cost);
}

inline bool operator <(const DijkstraKshortestFuzzy::FuzzyCost& x, const DijkstraKshortestFuzzy::FuzzyCost& y)
{
    if (x.exp() < y.exp()) {
        return (true);
    }
    return (false);
}

inline bool operator ==(const DijkstraKshortestFuzzy::FuzzyCost& x, const DijkstraKshortestFuzzy::FuzzyCost& y)
{
    if (x.exp() == y.exp()) {
        return true;
    }
    return false;
}

inline bool operator !=(const DijkstraKshortestFuzzy::FuzzyCost& x, const DijkstraKshortestFuzzy::FuzzyCost& y)
{
    if (x==y)
        return false;
    return true;
}

inline bool operator >(const DijkstraKshortestFuzzy::FuzzyCost& x, const DijkstraKshortestFuzzy::FuzzyCost& y)
{
    if (x.exp() > y.exp()) {
        return true;
    }
    return false;
}


#endif
