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

typedef std::vector<std::pair<NodeId, NodeId> > NodePairs;

class Dijkstra
{
protected:
    enum StateLabel
    {
        perm, tent
    };

public:
    enum Method
    {
        basic, widestshortest, shortestwidest
    };

protected :
    Method method;

public:

    typedef std::vector<NodeId> Route;
    typedef std::map<NodeId, Route> MapRoutes;

    class SetElem
    {
    public:
        NodeId iD;
        Method m;
        double cost;
        double cost2 = -1;
        SetElem()
        {
            iD = UndefinedAddr;
            cost = 1e30;
            cost2 = 0;
            m = basic;
        }
        SetElem& operator=(const SetElem& val)
        {
            this->iD = val.iD;
            this->cost2 = val.cost2;
            this->cost = val.cost;
            return *this;
        }
    };

    friend bool operator <(const Dijkstra::SetElem& x, const Dijkstra::SetElem& y);
    friend bool operator >(const Dijkstra::SetElem& x, const Dijkstra::SetElem& y);
    class State
    {
    public:
        double cost = 1e30;
        double cost2 = 0;
        NodeId idPrev;
        StateLabel label;
        State();
        State(const double &cost, const double &);
        virtual ~State();
    };

    struct Edge
    {
        NodeId last_node_; // last node to reach node X
        double cost;
        double cost2;
        Edge()
        {
            cost = 1e30;
            cost2 = 0;
            last_node_ = -1;
        }

        Edge(const Edge &other)
        {
            last_node_ = other.last_node_;
            cost = other.cost;
            cost2 = other.cost2;
        }

        virtual Edge *dup() const {return new Edge(*this);}

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

        virtual double& Cost2()
        {
            return cost2;
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
    virtual void discoverPartitionedLinks(std::vector<NodeId> &pathNode, const LinkArray &, NodePairs &);
    virtual void discoverAllPartitionedLinks(const LinkArray & topo, NodePairs &links);

    virtual void discoverPartitionedLinks(std::vector<NodeId> &pathNode, NodePairs &array) {
        discoverPartitionedLinks(pathNode, linkArray, array);
    }

    virtual void discoverAllPartitionedLinks(NodePairs &links) {
        discoverAllPartitionedLinks(linkArray, links);
    }

    virtual void setFromTopo(const cTopology *);
/*    virtual void setFromDijkstraFuzzy(const DijkstraFuzzy::LinkArray &);
    virtual void setFromDijkstraFuzzy(const DijkstraFuzzy::LinkArray &, LinkArray &);*/

    virtual void cleanLinkArray(LinkArray &);
    virtual void addEdge(const NodeId & dest_node, const NodeId & last_node, double cost, double cost2, LinkArray &);
    virtual void addEdge(const NodeId & dest_node, Edge*, LinkArray &);
    virtual void deleteEdge(const NodeId &, const NodeId &, LinkArray &);
    virtual Edge* removeEdge(const NodeId & originNode, const NodeId & last_node, LinkArray & linkArray);

    virtual void cleanLinkArray();
    virtual void addEdge(const NodeId & dest_node, const NodeId & last_node, double cost, double cost2);
    virtual void addEdge(const NodeId & dest_node, Edge *);
    virtual void deleteEdge(const NodeId &, const NodeId &);
    virtual void setRoot(const NodeId & dest_node);
    virtual void run(const int &, const LinkArray &, RouteMap &);
    virtual void runUntil(const int &, const int &, const LinkArray &, RouteMap &);
    virtual void run();
    virtual void runUntil(const NodeId &);
    virtual bool getRoute(const NodeId &nodeId, std::vector<NodeId> &pathNode);
    virtual bool getRoute(const NodeId &, std::vector<NodeId> &, const RouteMap &);
    virtual void setMethod(Method p) {method = p;}
};

inline bool operator <(const Dijkstra::SetElem& x, const Dijkstra::SetElem& y)
{
    if (x.m == Dijkstra::widestshortest) {
        if (x.cost != y.cost)
            return x.cost < y.cost;
        return x.cost2 > y.cost2;
    }
    if (x.m == Dijkstra::shortestwidest)
        if (x.cost2 != y.cost2)
            return x.cost2 > y.cost2;
        return x.cost < y.cost;
    return x.cost < y.cost;
}

inline bool operator >(const Dijkstra::SetElem& x, const Dijkstra::SetElem& y)
{
    if (x.m == Dijkstra::widestshortest) {
        if (x.cost != y.cost)
            return x.cost > y.cost;
        return x.cost2 < y.cost2;
    }
    if (x.m == Dijkstra::shortestwidest)
        if (x.cost2 != y.cost2)
            return x.cost2 < y.cost2;
        return x.cost > y.cost;
    return x.cost > y.cost;
}

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
        const double exp() const {return ((1 - DijkstraFuzzy::alpha) * cost1 + cost2 + DijkstraFuzzy::alpha * cost3);}
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
    struct Edge {
        NodeId last_node_; // last node to reach node X
        FuzzyCost cost;
        Edge() {
            cost = maximumCost;
        }
        Edge(const Edge &other) {
            last_node_ = other.last_node_;
            cost = other.cost;
        }
        virtual ~Edge() {

        }

        virtual Edge *dup() const {
            return new Edge(*this);
        }
        inline NodeId& last_node() {
            return last_node_;
        }
    };

    typedef std::vector<NodeId> Route;
    typedef std::vector<Route> Kroutes;
    typedef std::map<NodeId, DijkstraFuzzy::State> RouteMap;
    typedef std::map<NodeId, std::vector<DijkstraFuzzy::Edge*> > LinkArray;
protected:
    bool hasFindDisjoint = false;
    NodePairs partitionLinks;

    void addCost(FuzzyCost &, const FuzzyCost & a, const FuzzyCost & b);
    static FuzzyCost minimumCost;
    static FuzzyCost maximumCost;
    friend bool operator <(const DijkstraFuzzy::FuzzyCost&, const DijkstraFuzzy::FuzzyCost&);
    friend bool operator ==(const DijkstraFuzzy::FuzzyCost&, const DijkstraFuzzy::FuzzyCost&);
    friend bool operator !=(const DijkstraFuzzy::FuzzyCost&, const DijkstraFuzzy::FuzzyCost&);
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
    DijkstraFuzzy(const DijkstraFuzzy& other);
    virtual ~DijkstraFuzzy();
    virtual void setFromTopo(const cTopology *);
    virtual bool getHasFindDisjoint() const {return (hasFindDisjoint);}
    virtual void setHasFindDisjoint(const bool &p) {hasFindDisjoint = p; if (!hasFindDisjoint) partitionLinks.clear();}
    virtual void initMinAndMax();
    virtual void cleanLinkArray();
    virtual void clearAll();
    virtual void addEdge(const NodeId & dest_node, const NodeId & last_node, double, double, double);
    virtual void addEdge(const NodeId & originNode, Edge *edge);
    virtual void deleteEdge(const NodeId & originNode, const NodeId & last_node);
    virtual Edge * removeEdge(const NodeId & originNode, const NodeId & last_node);
    virtual void setRoot(const NodeId & dest_node);
    virtual void run();
    virtual void run(const LinkArray &linkArray, RouteMap&);
    virtual void runUntil(const NodeId &, const LinkArray &linkArray, RouteMap&);
    virtual void runDisjoint(const NodeId & rootNode, const NodeId &target, NodePairs &partitionLinks, RouteMap &routeMap,const LinkArray &linkArray, MapRoutes &kRoutesMap);
    virtual void runDisjoint(const NodeId &target);
    virtual int getNumRoutes(const NodeId &nodeId);
    virtual bool getRoute(const NodeId &nodeId, std::vector<NodeId> &, FuzzyCost &);
    virtual bool getRoute(const NodeId &, std::vector<NodeId> &, const RouteMap &, FuzzyCost &);
    virtual bool checkDisjoint(const NodeId &nodeId, Route & r1, Route &r2);
    virtual bool getCostPath(const Route &, const LinkArray &linkArray, FuzzyCost &);
    virtual bool getCostPath(const Route &r, FuzzyCost &c) {return getCostPath(r, linkArray, c);}

    virtual void discoverAllPartitionedLinks(const LinkArray & topo, NodePairs &links);
    virtual void discoverAllPartitionedLinks(NodePairs &links) {
        discoverAllPartitionedLinks(linkArray, links);
    }
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
        return (false);
    return (x.cost > y.cost);
}

inline bool operator <(const DijkstraFuzzy::FuzzyCost& x, const DijkstraFuzzy::FuzzyCost& y)
{
    if (x.exp() < y.exp()) {
        return (true);
    }
    return (false);
}

inline bool operator ==(const DijkstraFuzzy::FuzzyCost& x, const DijkstraFuzzy::FuzzyCost& y)
{
    if (x.exp() == y.exp()) {
        return true;
    }
    return false;
}

inline bool operator !=(const DijkstraFuzzy::FuzzyCost& x, const DijkstraFuzzy::FuzzyCost& y)
{
    if (x==y)
        return false;
    return true;
}

inline bool operator >(const DijkstraFuzzy::FuzzyCost& x, const DijkstraFuzzy::FuzzyCost& y)
{
    if (x.exp() > y.exp()) {
        return true;
    }
    return false;
}

#endif
