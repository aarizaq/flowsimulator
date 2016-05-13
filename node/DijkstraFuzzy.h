//
// Copyright (C) 2016 Alfonso Ariza, Malaga University
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
#include <deque>
#include <algorithm>
#include <omnetpp.h>

using namespace omnetpp;

typedef int NodeId;
#define InvalidId -1

#define UndefinedAddr -1

class DijkstraFuzzy
{
public:
    struct FuzzyCost
    {
        double cost1;
        double cost2;
        double cost3;
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

    };
    class SetElem
    {
    public:
        int iD;
        DijkstraFuzzy::FuzzyCost cost;
        SetElem()
        {
            iD = InvalidId;
        }
        SetElem& operator=(const SetElem& val)
        {
            this->iD = val.iD;
            this->cost = val.cost;
            return *this;
        }
    };

    class State
    {
    public:
        FuzzyCost cost;
        NodeId idPrev;
        State();
        State(const FuzzyCost &cost);
        ~State();
        void setFuzzyCost(const FuzzyCost &cost);
    };
    struct Edge
    {
        NodeId last_node_; // last node to reach node X
        FuzzyCost cost;
        Edge()
        {
            cost = maximumCost;
        }
        inline NodeId& last_node()
        {
            return last_node_;
        }
    };

    typedef std::vector<NodeId> Route;
    typedef std::vector<Route> Kroutes;
    typedef std::map<NodeId, DijkstraFuzzy::State> RouteMap;
    typedef std::map<NodeId, std::vector<DijkstraFuzzy::Edge*> > LinkArray;
protected:

    void addCost(FuzzyCost &, const FuzzyCost & a, const FuzzyCost & b);
    static FuzzyCost minimumCost;
    static FuzzyCost maximumCost;
    friend bool operator <(const DijkstraFuzzy::FuzzyCost&, const DijkstraFuzzy::FuzzyCost&);
    friend bool operator ==(const DijkstraFuzzy::FuzzyCost&, const DijkstraFuzzy::FuzzyCost&);
    friend bool operator >(const DijkstraFuzzy::FuzzyCost&, const DijkstraFuzzy::FuzzyCost&);
    friend bool operator <(const DijkstraFuzzy::SetElem& x, const DijkstraFuzzy::SetElem& y);
    friend bool operator ==(const DijkstraFuzzy::SetElem& x, const DijkstraFuzzy::SetElem& y);
    friend bool operator >(const DijkstraFuzzy::SetElem& x, const DijkstraFuzzy::SetElem& y);

    typedef std::map<NodeId, Kroutes> MapRoutes;
    MapRoutes kRoutesMap;

    static double alpha;

    LinkArray linkArray;
    LinkArray uniqueLink;
    RouteMap routeMap;
    NodeId rootNode;
    int K_LIMITE = 1;
    FuzzyCost limitsData;

    typedef std::vector<NodeId> BreaksVect;
    void breaks(const Route &S, const Route &Sp, BreaksVect &Vect_breaks);
    void Pair_Paths(const Route &, const Route &, BreaksVect &, const LinkArray &, const NodeId&, const NodeId &,
            Route &, Route &);
    void buildGamma1(Route&, Route&, const Route&, const Route&, const BreaksVect &, const LinkArray &, const NodeId &,
            const NodeId & t, Route &);
    void buildGamma2(Route&, Route&, const Route&, const Route&, const BreaksVect &, const LinkArray &, const NodeId &,
            const NodeId & t, const Route &, Route &);
public:
    DijkstraFuzzy();
    virtual ~DijkstraFuzzy();
    virtual void setFromTopo(const cTopology *);
    virtual void setKLimit(int val)
    {
        if (val > 0)
            K_LIMITE = val;
    }
    virtual void initMinAndMax();
    virtual void cleanLinkArray();
    virtual void clearAll();
    virtual void addEdge(const NodeId & dest_node, const NodeId & last_node, double, double, double);
    virtual void deleteEdge(const NodeId & originNode, const NodeId & last_node);
    virtual void deleteEdgeWithoutDeletePtr(const NodeId & originNode, const NodeId & last_node);
    virtual void setRoot(const NodeId & dest_node);
    virtual void run();
    virtual void run(const LinkArray &linkArray, RouteMap&, MapRoutes&);
    virtual void runUntil(const NodeId &, const LinkArray &linkArray, RouteMap&, MapRoutes&);
    virtual void runDisjoint(const NodeId &target);
    virtual int getNumRoutes(const NodeId &nodeId);
    virtual bool getRoute(const NodeId &nodeId, std::vector<NodeId> &, FuzzyCost &);
    virtual bool getRoute(const NodeId &, std::vector<NodeId> &, const RouteMap &, FuzzyCost &);
};

inline bool operator <(const DijkstraFuzzy::SetElem& x, const DijkstraFuzzy::SetElem& y)
{
    if (x.iD == y.iD)
        return false;
    if (x.cost == y.cost)
        return x.iD < y.iD;
    return x.cost < y.cost;
}

inline bool operator ==(const DijkstraFuzzy::SetElem& x, const DijkstraFuzzy::SetElem& y)
{
    if (x.iD == y.iD)
        return true;
    return x.cost == y.cost;
}

inline bool operator >(const DijkstraFuzzy::SetElem& x, const DijkstraFuzzy::SetElem& y)
{
    if (x.iD == y.iD)
        return false;
    return x.cost > y.cost;
}

inline bool operator <(const DijkstraFuzzy::FuzzyCost& x, const DijkstraFuzzy::FuzzyCost& y)
{
    double exp_x = (1 - DijkstraFuzzy::alpha) * x.cost1 + x.cost2 + DijkstraFuzzy::alpha * x.cost3;
    double exp_y = (1 - DijkstraFuzzy::alpha) * y.cost1 + y.cost2 + DijkstraFuzzy::alpha * y.cost3;

    if (exp_x < exp_y) {
        return true;
    }
    return false;
}

inline bool operator ==(const DijkstraFuzzy::FuzzyCost& x, const DijkstraFuzzy::FuzzyCost& y)
{
    double exp_x = (1 - DijkstraFuzzy::alpha) * x.cost1 + x.cost2 + DijkstraFuzzy::alpha * x.cost3;
    double exp_y = (1 - DijkstraFuzzy::alpha) * y.cost1 + y.cost2 + DijkstraFuzzy::alpha * y.cost3;

    if (exp_x == exp_y) {
        return true;
    }
    return false;
}

inline bool operator >(const DijkstraFuzzy::FuzzyCost& x, const DijkstraFuzzy::FuzzyCost& y)
{
    double exp_x = (1 - DijkstraFuzzy::alpha) * x.cost1 + x.cost2 + DijkstraFuzzy::alpha * x.cost3;
    double exp_y = (1 - DijkstraFuzzy::alpha) * y.cost1 + y.cost2 + DijkstraFuzzy::alpha * y.cost3;

    if (exp_x > exp_y) {
        return true;
    }
    return false;
}

class Dijkstra
{
protected:
    enum StateLabel
    {
        perm, tent
    };

public:
    typedef std::vector<NodeId> Route;
    typedef std::map<NodeId, Route> MapRoutes;

    class SetElem
    {
    public:
        NodeId iD;
        double cost;
        SetElem()
        {
            iD = UndefinedAddr;
            cost = 1e30;
        }
        SetElem& operator=(const SetElem& val)
        {
            this->iD = val.iD;
            this->cost = val.cost;
            return *this;
        }

    };
    friend bool operator <(const Dijkstra::SetElem& x, const Dijkstra::SetElem& y);
    friend bool operator >(const Dijkstra::SetElem& x, const Dijkstra::SetElem& y);
    class State
    {
    public:
        double cost;
        NodeId idPrev;
        StateLabel label;
        State();
        State(const double &cost);
        ~State();
        void setCostVector(const double &cost);
    };

    struct Edge
    {
        NodeId last_node_; // last node to reach node X
        double cost;
        Edge()
        {
            cost = 1e30;
        }
        virtual ~Edge()
        {

        }
        inline NodeId& last_node()
        {
            return last_node_;
        }
        virtual double& Cost()
        {
            return cost;
        }
    };

    typedef std::map<NodeId, Dijkstra::State> RouteMap;
    typedef std::map<NodeId, std::vector<Dijkstra::Edge*> > LinkArray;
protected:
    LinkArray linkArray;
    RouteMap routeMap;
    NodeId rootNode;
public:

    Dijkstra();
    virtual ~Dijkstra();
    virtual void discoverPartitionedLinks(std::vector<NodeId> &pathNode, const LinkArray &, LinkArray &);
    virtual void discoverAllPartitionedLinks(const LinkArray & topo, LinkArray &links);

    virtual void discoverPartitionedLinks(std::vector<NodeId> &pathNode, LinkArray &array) {
        discoverPartitionedLinks(pathNode, linkArray, array);
    }

    virtual void discoverAllPartitionedLinks(LinkArray &links) {
        discoverAllPartitionedLinks(linkArray, links);
    }

    virtual void setFromTopo(const cTopology *);
    virtual void setFromDijkstraFuzzy(const DijkstraFuzzy::LinkArray &);
    virtual void setFromDijkstraFuzzy(const DijkstraFuzzy::LinkArray &, LinkArray &);

    virtual void cleanLinkArray(LinkArray &);
    virtual void addEdge(const NodeId & dest_node, const NodeId & last_node, double cost, LinkArray &);
    virtual void addEdge(const NodeId & dest_node, Edge*, LinkArray &);
    virtual void deleteEdge(const NodeId &, const NodeId &, LinkArray &);
    virtual Edge* deleteEdgeWithoutDeletePtr(const NodeId & originNode, const NodeId & last_node, LinkArray & linkArray);

    virtual void cleanLinkArray();
    virtual void addEdge(const NodeId & dest_node, const NodeId & last_node, double cost);
    virtual void addEdge(const NodeId & dest_node, Edge *);
    virtual void deleteEdge(const NodeId &, const NodeId &);
    virtual void setRoot(const NodeId & dest_node);
    virtual void run(const int &, const LinkArray &, RouteMap &);
    virtual void runUntil(const int &, const int &, const LinkArray &, RouteMap &);
    virtual void run();
    virtual void runUntil(const NodeId &);
    virtual bool getRoute(const NodeId &nodeId, std::vector<NodeId> &pathNode);
    virtual bool getRoute(const NodeId &, std::vector<NodeId> &, const RouteMap &);
};

inline bool operator <(const Dijkstra::SetElem& x, const Dijkstra::SetElem& y)
{
    return x.cost < y.cost;
}

inline bool operator >(const Dijkstra::SetElem& x, const Dijkstra::SetElem& y)
{
    return x.cost > y.cost;
}

#endif
