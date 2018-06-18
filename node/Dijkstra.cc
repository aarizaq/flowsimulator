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

#include "Dijkstra.h"

inline bool operator <(const Dijkstra::SetElem& x, const Dijkstra::SetElem& y)
{
    if (x.m == Dijkstra::widestshortest) {
        if (x.cost != y.cost)
            return (x.cost < y.cost);
        return (x.cost2 > y.cost2);
    }
    if (x.m == Dijkstra::shortestwidest) {
        if (x.cost2 != y.cost2)
            return (x.cost2 > y.cost2);
        return (x.cost < y.cost);
    }
    return (x.cost < y.cost);
}

inline bool operator >(const Dijkstra::SetElem& x, const Dijkstra::SetElem& y)
{
    if (x.m == Dijkstra::widestshortest) {
        if (x.cost != y.cost)
            return (x.cost > y.cost);
        return (x.cost2 < y.cost2);
    }
    if (x.m == Dijkstra::shortestwidest) {
        if (x.cost2 != y.cost2)
            return (x.cost2 < y.cost2);
        return (x.cost > y.cost);
    }
    return (x.cost > y.cost);
}


Dijkstra::State::State(): idPrev(UndefinedAddr), label(tent)
{
}

Dijkstra::State::State(const double &costData, const double &c ) :Dijkstra::State::State()
{
    cost = costData;
    cost2 = c;
}

Dijkstra::State::~State()
{

}

Dijkstra::Dijkstra()
{
    rootNode = -1;
}

void Dijkstra::cleanLinkArray()
{
    cleanLinkArray(linkArray);
}

void Dijkstra::clearAll()
{
    cleanLinkArray();
    routeMap.clear();
}

void Dijkstra::cleanLinkArray(LinkArray &linkArray)
{
    for (auto it = linkArray.begin(); it != linkArray.end(); it++)
        while (!it->second.empty()) {
            delete it->second.back();
            it->second.pop_back();
        }
    linkArray.clear();
}

Dijkstra::~Dijkstra()
{
    cleanLinkArray();
}

void Dijkstra::addEdge(const NodeId & originNode, const NodeId & last_node, const double &cost, const double &cost2, LinkArray & linkArray)
{
    auto it = linkArray.find(originNode);
    if (it != linkArray.end()) {
        for (unsigned int i = 0; i < it->second.size(); i++) {
            if (last_node == it->second[i]->last_node_) {
                // check changes in the cost
                if (it->second[i]->Cost() != cost || it->second[i]->Cost2() != cost2) {
                    it->second[i]->Cost() = cost;
                    it->second[i]->Cost2() = cost2;
                    // delete routing stored information
                    routeMap.clear();
                }
                return;
            }
        }
    }
    Edge *link = new Edge;
    // The last hop is the interface in which we have this neighbor...
    link->last_node() = last_node;
    // Also record the link delay and quality..
    link->Cost() = cost;
    link->Cost2() = cost2;
    linkArray[originNode].push_back(link);
    routeMap.clear();
}

void Dijkstra::addEdge(const NodeId & originNode, Edge * edge, LinkArray & linkArray)
{
    auto it = linkArray.find(originNode);
    routeMap.clear();
    if (it != linkArray.end()) {
        for (unsigned int i = 0; i < it->second.size(); i++) {
            if (edge->last_node() == it->second[i]->last_node_) {
                it->second[i]->Cost() = edge->Cost();
                delete edge;
                return;
            }
        }
    }
    linkArray[originNode].push_back(edge);
}

void Dijkstra::deleteEdge(const NodeId & originNode, const NodeId & last_node, LinkArray & linkArray)
{
    Dijkstra::Edge * e = removeEdge(originNode, last_node,linkArray);
    if (e != nullptr)
        delete e;
}

Dijkstra::Edge * Dijkstra::removeEdge(const NodeId & originNode, const NodeId & last_node, LinkArray & linkArray)
{
    auto it = linkArray.find(originNode);
    if (it != linkArray.end()) {
        for (auto itAux = it->second.begin(); itAux != it->second.end(); ++itAux) {
            Edge *edge = *itAux;
            if (last_node == edge->last_node_) {
                it->second.erase(itAux);
                routeMap.clear();
                return (edge);
            }
        }
    }
    return (nullptr);
}

void Dijkstra::addEdge(const NodeId & originNode, const NodeId & last_node, const double &cost,  const double &cost2)
{
    addEdge(originNode, last_node, cost, cost2, linkArray);
}

void Dijkstra::addEdge(const NodeId & originNode, Edge * edge)
{
    addEdge(originNode, edge,linkArray);
}

void Dijkstra::deleteEdge(const NodeId & originNode, const NodeId & last_node)
{
    deleteEdge(originNode, last_node, linkArray);
}

void Dijkstra::setRoot(const NodeId & dest_node)
{
    rootNode = dest_node;
}

void Dijkstra::run()
{
    run(rootNode, linkArray, routeMap);
}

void Dijkstra::runUntil(const NodeId &target)
{
    runUntil(target, rootNode, linkArray, routeMap);
}

void Dijkstra::run(const int &rootNode, const LinkArray &linkArray, RouteMap &routeMap)
{
    runUntil(-1, rootNode, linkArray, routeMap);
}

void Dijkstra::runUntil(const NodeId &target, const int &rootNode, const LinkArray &linkArray, RouteMap &routeMap)
{
    //std::multiset<SetElem> heap;
    std::deque<SetElem> heap;
    routeMap.clear();

    auto it = linkArray.find(rootNode);
    if (it == linkArray.end())
        throw cRuntimeError("Node not found");

    State state(0, std::numeric_limits<double>::max());
    state.label = perm;
    routeMap[rootNode] = state;

    SetElem elem;
    elem.iD = rootNode;
    elem.cost = 0;
    //heap.insert(elem);
    heap.push_back(elem);
    while (!heap.empty()) {
        auto itHeap = heap.begin();
        double cost1It = itHeap->cost;
        double cost2It = itHeap->cost2;
        // search min
        for (auto it = heap.begin(); it!=heap.end(); ++it)
        {
            if (method == Method::widestshortest) {
                if (it->cost < cost1It || (it->cost == cost1It && it->cost2 > cost2It)) {
                    itHeap = it;
                    cost1It = itHeap->cost;
                    cost2It = itHeap->cost2;
                }
            }
            else if (method == Method::shortestwidest) {
                if (it->cost2 > cost2It || (it->cost2 == cost2It && it->cost < cost1It)) {
                    itHeap = it;
                    cost1It = itHeap->cost;
                    cost2It = itHeap->cost2;
                }
            }
            else {
                if (it->cost < cost1It) {
                    itHeap = it;
                    cost1It = itHeap->cost;
                    cost2It = itHeap->cost2;
                }
            }
        }

        // SetElem elem = *heap.begin();
        //heap.erase(heap.begin());
        SetElem elem = *itHeap;
        heap.erase(itHeap);
        auto it = routeMap.find(elem.iD);
        if (it == routeMap.end())
            throw cRuntimeError("node not found in routeMap");

        if (elem.iD != rootNode) {
            if (it->second.label == perm)
                continue; // nothing to do with this element
        }
        it->second.label = perm;

        if (target == elem.iD)
            return;
        auto linkIt = linkArray.find(elem.iD);
        if (linkIt == linkArray.end())
            throw cRuntimeError("Error link not found in linkArray");
        for (unsigned int i = 0; i < linkIt->second.size(); i++) {

            Edge* current_edge = (linkIt->second)[i];
            double cost;
            double cost2;
            auto itNext = routeMap.find(current_edge->last_node());
            cost = current_edge->cost + it->second.cost;
            cost2 = std::min(current_edge->cost2, it->second.cost2);

           if (itNext == routeMap.end()) {
                State state;
                state.idPrev = elem.iD;
                state.cost = cost;
                state.cost2 = cost2;
                state.label = tent;
                routeMap[current_edge->last_node()] = state;

                SetElem newElem;
                newElem.iD = current_edge->last_node();
                newElem.cost = cost;
                newElem.cost2 = cost2;
                heap.push_back(newElem);
                //heap.insert(newElem);
            }
            else {
                if (itNext->second.label == perm)
                    continue;

                double maxCost = itNext->second.cost;
                double maxCost2 = itNext->second.cost2;
                bool actualize = false;

                if (method == Method::widestshortest) {
                    if (cost < maxCost || (cost == maxCost && cost2 > maxCost2))
                        actualize = true;
                }
                else if (method == Method::shortestwidest) {
                    if (cost2 > maxCost2 || (cost2 == maxCost2 && cost < maxCost))
                        actualize = true;
                }
                else {
                    if (cost < maxCost)
                        actualize = true;
                }

                if (actualize) {
                    itNext->second.cost = cost;
                    itNext->second.idPrev = elem.iD;
                    SetElem newElem;
                    newElem.iD = current_edge->last_node();
                    newElem.cost = cost;
                    //for (auto it = heap.begin(); it != heap.end(); ++it) {
                    //    if (it->iD == newElem.iD && it->cost > newElem.cost) {
                    //        heap.erase(it);
                    //        break;
                    //    }
                    //}
                    heap.push_back(newElem);
                    //heap.insert(newElem);
                }
            }
        }
    }
}

bool Dijkstra::getRoute(const NodeId &nodeId, std::vector<NodeId> &pathNode, const RouteMap &routeMap)
{
    auto it = routeMap.find(nodeId);
    if (it == routeMap.end())
        return (false);
    std::vector<NodeId> path;
    NodeId currentNode = nodeId;

    while (currentNode != rootNode) {
        path.push_back(currentNode);
        currentNode = it->second.idPrev;
        it = routeMap.find(currentNode);
        if (it == routeMap.end())
            throw cRuntimeError("error in data");
    }
    path.push_back(rootNode);
    pathNode.clear();
    while (!path.empty()) {
        pathNode.push_back(path.back());
        path.pop_back();
    }
    return (true);

}

bool Dijkstra::getRoute(const NodeId &nodeId, std::vector<NodeId> &pathNode)
{
    return (getRoute(nodeId, pathNode, routeMap));
}

void Dijkstra::setFromTopo(const cTopology *topo)
{
    for (int i = 0; i < topo->getNumNodes(); i++) {
        cTopology::Node *node = const_cast<cTopology*>(topo)->getNode(i);
        NodeId id = node->getModule()->par("address");
        for (int j = 0; j < node->getNumOutLinks(); j++) {

            NodeId idNex = node->getLinkOut(j)->getRemoteNode()->getModule()->par("address");
            cChannel * channel = node->getModule()->gate("port$o", i)->getTransmissionChannel();
            double cost = 1 / channel->getNominalDatarate();
            addEdge(id, idNex, cost,channel->getNominalDatarate());
        }
    }
}

/*void Dijkstra::setFromDijkstraFuzzy(const DijkstraFuzzy::LinkArray & topo)
{
    cleanLinkArray();
    for (auto elem : topo) {
        for (auto elem2 : elem.second)
            addEdge(elem.first, elem2->last_node(), 1);

    }
}

void Dijkstra::setFromDijkstraFuzzy(const DijkstraFuzzy::LinkArray & topo, LinkArray &linkArray)
{
    cleanLinkArray(linkArray);
    for (auto elem : topo) {
        for (auto elem2 : elem.second)
            addEdge(elem.first, elem2->last_node(), 1, linkArray);

    }
}*/

void Dijkstra::discoverPartitionedLinks(std::vector<NodeId> &pathNode, const LinkArray & topo, NodePairs &links)
{

    LinkArray topoAux = topo;
    for (unsigned int i = 0; i < pathNode.size() - 1; i++) {
        auto it1 = topoAux.find(pathNode[i]);
        if (it1 == topoAux.end())
            throw cRuntimeError("Node not found %i", pathNode[i]);
        auto it = topoAux.find(pathNode[i]);
        Edge *tempEdge = nullptr;
        int origin = it1->first;
        int nodeId = pathNode[i + 1];
        for (auto itAux = it->second.begin(); itAux != it->second.end(); ++it) {
            if ((*itAux)->last_node() == pathNode[i + 1]) {
                tempEdge = *itAux;
                it->second.erase(itAux);
                break;
            }
        }
        if (tempEdge == nullptr)
            throw cRuntimeError("Link not found %i - %i", pathNode[i], pathNode[i + 1]);
        RouteMap routeMap;

        runUntil(nodeId, origin, topoAux, routeMap);
        setRoot(origin);
        std::vector<NodeId> pathNode;
        bool has = getRoute(nodeId, pathNode, routeMap);
        // include the link other time
        it->second.push_back(tempEdge);
        if (!has) {
            links.push_back(std::make_pair(origin,nodeId));
        }
    }
}

void Dijkstra::discoverAllPartitionedLinks(const LinkArray & topo, NodePairs &links)
{
    LinkArray topoAux = topo;
    NodePairs tested;
    for (auto elem : topo)
    {
        NodeId node = elem.first;
        setRoot(node);
        for (unsigned int i = 0; i < elem.second.size();i++)
        {
            auto it1 = std::find(tested.begin(),tested.end(),std::make_pair(node, elem.second[i]->last_node()));
            if (it1 != tested.end())
                continue;
            Edge * edge = removeEdge(node, elem.second[i]->last_node(), topoAux);
            RouteMap routeMap;
            runUntil(elem.second[i]->last_node(), node, topoAux, routeMap);
            std::vector<NodeId> pathNode;
            bool has = getRoute(elem.second[i]->last_node(), pathNode, routeMap);
            if (!has) {
                links.push_back(std::make_pair(node,elem.second[i]->last_node()));
                links.push_back(std::make_pair(elem.second[i]->last_node(),node));
            }
            addEdge(node, edge, topoAux);
            tested.push_back(std::make_pair(node, elem.second[i]->last_node()));
            tested.push_back(std::make_pair(elem.second[i]->last_node(),node));
        }
    }
}

