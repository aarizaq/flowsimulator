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

#ifndef __DIJKSTRA_SHORTEST__H__
#define __DIJKSTRA_SHORTEST__H__

#include <vector>
#include <map>
#include <set>
#include <deque>
#include <algorithm>
#include <omnetpp.h>


//#define OtherCost

using namespace omnetpp;

#ifndef NodeId
typedef int NodeId;
#endif

#ifndef InvalidId
#define InvalidId -1
#endif

#ifndef UndefinedAddr
#define UndefinedAddr -1
#endif

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
        basic, widestshortest, shortestwidest,otherCost
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
    virtual void addEdge(const NodeId & dest_node, const NodeId & last_node, const double &cost, const double &cost2, LinkArray &);
    virtual void addEdge(const NodeId & dest_node, Edge*, LinkArray &);
    virtual void deleteEdge(const NodeId &, const NodeId &, LinkArray &);
    virtual Edge* removeEdge(const NodeId & originNode, const NodeId & last_node, LinkArray & linkArray);

    virtual void cleanLinkArray();
    virtual void clearAll();
    virtual void cleanRoutes() {routeMap.clear();}
    virtual void addEdge(const NodeId & dest_node, const NodeId & last_node, const double &cost, const double &cost2);
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

#endif
