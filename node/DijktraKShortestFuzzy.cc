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

#include <omnetpp.h>
#include "DijktraKShortestFuzzy.h"


DijkstraKshortestFuzzy::FuzzyCost DijkstraKshortestFuzzy::minimumCost = {0, 0, 0};
DijkstraKshortestFuzzy::FuzzyCost DijkstraKshortestFuzzy::maximumCost = {std::numeric_limits<double>::max(),  std::numeric_limits<double>::max(),  std::numeric_limits<double>::max()};
double DijkstraKshortestFuzzy::alpha = 0.55;

DijkstraKshortestFuzzy::State::State()
{
    idPrev = UndefinedAddr;
    idPrevIdx = -1;
    label = tent;
}

DijkstraKshortestFuzzy::State::State(const FuzzyCost &costData)
{
    idPrev = UndefinedAddr;
    idPrevIdx = -1;
    label = tent;
    cost = costData;
}

DijkstraKshortestFuzzy::State::~State()
{
}


void DijkstraKshortestFuzzy::addCost(FuzzyCost &val, const FuzzyCost & a, const FuzzyCost & b)
{
    val = a + b;
}

void DijkstraKshortestFuzzy::initMinAndMax()
{
    minimumCost.cost1 = 0;
    minimumCost.cost2 = 0;
    minimumCost.cost3 = 0;
    maximumCost.cost1 = std::numeric_limits<double>::max();
    maximumCost.cost2 = std::numeric_limits<double>::max();
    maximumCost.cost3 = std::numeric_limits<double>::max();
}

DijkstraKshortestFuzzy::DijkstraKshortestFuzzy() : K_LIMITE(1)
{
    // initMinAndMax();
}

DijkstraKshortestFuzzy::DijkstraKshortestFuzzy(int limit) : K_LIMITE(limit)
{
    initMinAndMax();
}

void DijkstraKshortestFuzzy::cleanLinkArray()
{
    for (auto it = linkArray.begin(); it != linkArray.end(); it++)
        while (!it->second.empty()) {
            delete it->second.back();
            it->second.pop_back();
        }
    linkArray.clear();
}

DijkstraKshortestFuzzy::~DijkstraKshortestFuzzy()
{
    cleanLinkArray();
    kRoutesMap.clear();
}

void DijkstraKshortestFuzzy::addEdge(const NodeId & originNode, const NodeId & last_node, double cost1, double cost2,
        double cost3)
{
    auto it = linkArray.find(originNode);
    if (it != linkArray.end()) {
        for (unsigned int i = 0; i < it->second.size(); i++) {
            if (last_node == it->second[i]->last_node_) {
                it->second[i]->Cost() =  FuzzyCost(cost1, cost2, cost3);
                return;
            }
        }
    }
    Edge *link = new Edge;
    // The last hop is the interface in which we have this neighbor...
    link->last_node() = last_node;
    // Also record the link delay and quality..
    link->Cost() =  FuzzyCost(cost1, cost2, cost3);;

    linkArray[originNode].push_back(link);
}

void DijkstraKshortestFuzzy::addLink(const NodeId & node1, const NodeId & node2, double cost, double cost2,
        double cost3)
{
    addEdge(node1, node2, cost, cost2, cost3);
    addEdge(node2, node1, cost, cost2, cost3);
}


void DijkstraKshortestFuzzy::deleteEdge(const NodeId & originNode, const NodeId & last_node)
{
    auto it = linkArray.find(originNode);
    if (it != linkArray.end()) {
        for (auto itAux = it->second.begin(); itAux != it->second.end(); ++itAux) {
            Edge *edge = *itAux;
            if (last_node == edge->last_node_) {

                it->second.erase(itAux);
                delete edge;
                return;
            }
        }
    }
}

DijkstraKshortestFuzzy::Edge * DijkstraKshortestFuzzy::removeEdge(const NodeId & originNode, const NodeId & last_node)
{
    auto it = linkArray.find(originNode);
    if (it != linkArray.end()) {
        for (auto itAux = it->second.begin(); itAux != it->second.end(); ++itAux) {
            Edge *edge = *itAux;
            if (last_node == edge->last_node_) {

                it->second.erase(itAux);
                return edge;

            }
        }
    }
    return nullptr;
}

void DijkstraKshortestFuzzy::setRoot(const NodeId & dest_node)
{
    rootNode = dest_node;

}

void DijkstraKshortestFuzzy::run()
{
    std::multiset<SetElem> heap;
    routeMap.clear();

    // include routes in the map
    kRoutesMap.clear();

    auto it = linkArray.find(rootNode);
    if (it == linkArray.end())
        throw cRuntimeError("Node not found");
    for (int i = 0; i < K_LIMITE; i++) {
        State state(minimumCost);
        state.label = perm;
        routeMap[rootNode].push_back(state);
    }
    SetElem elem;
    elem.iD = rootNode;
    elem.idx = 0;
    elem.cost = minimumCost;
    heap.insert(elem);
    while (!heap.empty()) {
        SetElem elem = *heap.begin();
        heap.erase(heap.begin());
        auto it = routeMap.find(elem.iD);
        if (it == routeMap.end())
            throw cRuntimeError("node not found in routeMap");

        if (elem.iD != rootNode) {
            if ((int)it->second.size() > elem.idx && it->second[elem.idx].label == perm)
                continue; // set
            if ((int) it->second.size() == K_LIMITE) {
                bool continueLoop = true;
                for (int i = 0; i < K_LIMITE; i++) {
                    if (it->second[i].label != perm) {
                        continueLoop = false;
                        break;
                    }
                }
                if (continueLoop)
                    continue; // nothing to do with this element
            }
        }

        if ((int) it->second.size() <= elem.idx) {
            for (int i = elem.idx - ((int) it->second.size() - 1); i >= 0; i--) {
                State state(maximumCost);
                state.label = tent;
                it->second.push_back(state);
            }
        }

        /// Record the route in the map
        auto itAux = it;
        Route pathActive;
        Route pathNode;
        int prevIdx = elem.idx;
        NodeId currentNode = elem.iD;
        while (currentNode != rootNode) {
            pathActive.push_back(currentNode);
            currentNode = itAux->second[prevIdx].idPrev;
            prevIdx = itAux->second[prevIdx].idPrevIdx;
            itAux = routeMap.find(currentNode);
            if (itAux == routeMap.end())
                throw cRuntimeError("error in data");
            if (prevIdx >= (int) itAux->second.size())
                throw cRuntimeError("error in data");
        }

        bool routeExist = false;
        if (!pathActive.empty()) // valid path, record in the map
        {
            while (!pathActive.empty()) {
                pathNode.push_back(pathActive.back());
                pathActive.pop_back();
            }
            auto itKroutes = kRoutesMap.find(elem.iD);
            if (itKroutes == kRoutesMap.end()) {
                Kroutes kroutes;
                kroutes.push_back(pathNode);
                kRoutesMap.insert(std::make_pair(elem.iD, kroutes));
            }
            else {
                for (unsigned int j = 0; j < itKroutes->second.size(); j++) {
                    if (pathNode == itKroutes->second[j]) {
                        routeExist = true;
                        break;
                    }
                }
                if (!routeExist) {
                    if ((int) itKroutes->second.size() < K_LIMITE)
                        itKroutes->second.push_back(pathNode);
                }
            }
        }

        if (routeExist)
            continue; // next
        it->second[elem.idx].label = perm;

        // next hop
        auto linkIt = linkArray.find(elem.iD);
        if (linkIt == linkArray.end())
            throw cRuntimeError("Error link not found in linkArray");

        for (unsigned int i = 0; i < linkIt->second.size(); i++) {
            Edge *current_edge = (linkIt->second)[i];
            FuzzyCost cost;
            FuzzyCost maxCost = maximumCost;
            int nextIdx;

            // check if the node is in the path
            if (std::find(pathNode.begin(), pathNode.end(), current_edge->last_node()) != pathNode.end())
                continue;

            auto itNext = routeMap.find(current_edge->last_node());


            addCost(cost, current_edge->cost, (it->second)[elem.idx].cost);


            if (itNext == routeMap.end() || (itNext != routeMap.end() && (int) itNext->second.size() < K_LIMITE)) {
                State state;
                state.idPrev = elem.iD;
                state.idPrevIdx = elem.idx;
                state.cost = cost;
                state.label = tent;
                routeMap[current_edge->last_node()].push_back(state);
                nextIdx = routeMap[current_edge->last_node()].size() - 1;
                SetElem newElem;
                newElem.iD = current_edge->last_node();
                newElem.idx = nextIdx;
                newElem.cost = cost;
                heap.insert(newElem);
            }
            else {
                bool permanent = true;
                for (unsigned i = 0; i < itNext->second.size(); i++) {
                    if ((maxCost < itNext->second[i].cost) && (itNext->second[i].label == tent)) {
                        maxCost = itNext->second[i].cost;
                        nextIdx = i;
                        permanent = false;
                    }
                }
                if (cost < maxCost && !permanent) {
                    itNext->second[nextIdx].cost = cost;
                    itNext->second[nextIdx].idPrev = elem.iD;
                    itNext->second[nextIdx].idPrevIdx = elem.idx;
                    SetElem newElem;
                    newElem.iD = current_edge->last_node();
                    newElem.idx = nextIdx;
                    newElem.cost = cost;
                    for (auto it = heap.begin(); it != heap.end(); ++it) {
                        if (it->iD == newElem.iD && it->idx == newElem.idx && it->cost > newElem.cost) {
                            heap.erase(it);
                            break;
                        }
                    }
                    heap.insert(newElem);
                }
            }
        }
    }
}

void DijkstraKshortestFuzzy::runUntil(const NodeId &target)
{
    std::multiset<SetElem> heap;
    routeMap.clear();

    auto it = linkArray.find(rootNode);
    if (it == linkArray.end())
        throw cRuntimeError("Node not found");
    for (int i = 0; i < K_LIMITE; i++) {
        State state(minimumCost);
        state.label = perm;
        routeMap[rootNode].push_back(state);
    }
    SetElem elem;
    elem.iD = rootNode;
    elem.idx = 0;
    elem.cost = minimumCost;
    heap.insert(elem);
    while (!heap.empty()) {
        SetElem elem = *heap.begin();
        heap.erase(heap.begin());
        auto it = routeMap.find(elem.iD);
        if (it == routeMap.end())
            throw cRuntimeError("node not found in routeMap");

        if (elem.iD != rootNode) {
            if ((int)it->second.size() > elem.idx && it->second[elem.idx].label == perm)
                continue; // set
            if ((int) it->second.size() == K_LIMITE) {
                bool continueLoop = true;
                for (int i = 0; i < K_LIMITE; i++) {
                    if (it->second[i].label != perm) {
                        continueLoop = false;
                        break;
                    }
                }
                if (continueLoop)
                    continue;
            }
        }

        if ((int) it->second.size() <= elem.idx) {
            for (int i = elem.idx - ((int) it->second.size() - 1); i >= 0; i--) {
                State state(maximumCost);
                state.label = tent;
                it->second.push_back(state);
            }
        }
        (it->second)[elem.idx].label = perm;
        if ((int) it->second.size() == K_LIMITE && target == elem.iD)
            return;
        auto linkIt = linkArray.find(elem.iD);
        if (linkIt == linkArray.end())
            throw cRuntimeError("Error link not found in linkArray");
        for (unsigned int i = 0; i < linkIt->second.size(); i++) {
            Edge* current_edge = (linkIt->second)[i];
            FuzzyCost cost;
            FuzzyCost maxCost = maximumCost;
            int nextIdx;
            auto itNext = routeMap.find(current_edge->last_node());
            addCost(cost, current_edge->cost, (it->second)[elem.idx].cost);

            if ((itNext == routeMap.end()) || (itNext != routeMap.end() && (int) itNext->second.size() < K_LIMITE)) {
                State state;
                state.idPrev = elem.iD;
                state.idPrevIdx = elem.idx;
                state.cost = cost;
                state.label = tent;
                routeMap[current_edge->last_node()].push_back(state);
                nextIdx = routeMap[current_edge->last_node()].size() - 1;
                SetElem newElem;
                newElem.iD = current_edge->last_node();
                newElem.idx = nextIdx;
                newElem.cost = cost;
                heap.insert(newElem);
            }
            else {
                bool permanent = true;
                for (unsigned i = 0; i < itNext->second.size(); i++) {
                    if ((maxCost < itNext->second[i].cost) && (itNext->second[i].label == tent)) {
                        maxCost = itNext->second[i].cost;
                        nextIdx = i;
                        permanent = false;
                    }
                }
                if (cost < maxCost && !permanent) {
                    itNext->second[nextIdx].cost = cost;
                    itNext->second[nextIdx].idPrev = elem.iD;
                    itNext->second[nextIdx].idPrevIdx = elem.idx;
                    SetElem newElem;
                    newElem.iD = current_edge->last_node();
                    newElem.idx = nextIdx;
                    newElem.cost = cost;
                    // erase
                    for (auto it = heap.begin(); it != heap.end(); ++it) {
                        if (it->iD == newElem.iD && it->idx == newElem.idx && it->cost > newElem.cost) {
                            heap.erase(it);
                            break;
                        }
                    }
                    heap.insert(newElem);
                }
            }
        }
    }
}

int DijkstraKshortestFuzzy::getNumRoutes(const NodeId &nodeId)
{
    auto it = routeMap.find(nodeId);
    if (it == routeMap.end())
        return -1;
    return (int) it->second.size();
}

bool DijkstraKshortestFuzzy::getRoute(const NodeId &nodeId, std::vector<NodeId> &pathNode, int k)
{
    auto it = routeMap.find(nodeId);
    if (it == routeMap.end())
        return false;
    if (nodeId != it->first)
        throw cRuntimeError("error in map");
    if (k >= (int) it->second.size())
        return false;
    std::vector<NodeId> path;
    NodeId currentNode = nodeId;
    int idx = k;
    auto itAux = it;
    auto itAux2 = it;
    int prevCurrentNode = -1;
    int prevIdx = -1;
    while (currentNode != rootNode) {
        path.push_back(currentNode);
        prevCurrentNode = currentNode;
        currentNode = itAux->second[idx].idPrev;
        if (currentNode < 0)
            throw cRuntimeError("error in data");
        prevIdx = idx;
        idx = itAux->second[idx].idPrevIdx;
        if (idx < 0)
            throw cRuntimeError("error in data");
        itAux2 = itAux;
        itAux = routeMap.find(currentNode);
        if (itAux == routeMap.end())
            throw cRuntimeError("error in data");
        if (idx >= (int) itAux->second.size())
            throw cRuntimeError("error in data");
    }
    path.push_back(rootNode);
    pathNode.clear();
    while (!path.empty()) {
        pathNode.push_back(path.back());
        path.pop_back();
    }
    return true;
}


DijkstraKshortestFuzzy::FuzzyCost DijkstraKshortestFuzzy::getRouteCost(const NodeId &nodeId, std::vector<NodeId> &pathNode, int k)
{
    auto it = routeMap.find(nodeId);
    if (it == routeMap.end())
        return maximumCost;
    if (nodeId != it->first)
        throw cRuntimeError("error in map");
    if (k >= (int) it->second.size())
        return maximumCost;
    std::vector<NodeId> path;
    NodeId currentNode = nodeId;
    int idx = k;
    auto itAux = it;
    auto itAux2 = it;
    int prevCurrentNode = -1;
    int prevIdx = -1;

    while (currentNode != rootNode) {
        path.push_back(currentNode);
        prevCurrentNode = currentNode;
        currentNode = itAux->second[idx].idPrev;
        if (currentNode < 0)
            throw cRuntimeError("error in data");
        prevIdx = idx;
        idx = itAux->second[idx].idPrevIdx;
        if (idx < 0)
            throw cRuntimeError("error in data");
        itAux2 = itAux;
        itAux = routeMap.find(currentNode);
        if (itAux == routeMap.end())
            throw cRuntimeError("error in data");
        if (idx >= (int) itAux->second.size())
            throw cRuntimeError("error in data");
    }
    path.push_back(rootNode);
    pathNode.clear();
    while (!path.empty()) {
        pathNode.push_back(path.back());
        path.pop_back();
    }
    return it->second[k].cost;
}


void DijkstraKshortestFuzzy::setFromTopo(const cTopology *topo)
{
    for (int i = 0; i < topo->getNumNodes(); i++) {
        cTopology::Node *node = const_cast<cTopology*>(topo)->getNode(i);
        NodeId id = node->getModule()->par("address");
        for (int j = 0; j < node->getNumOutLinks(); j++) {
            NodeId idNex = node->getLinkOut(j)->getRemoteNode()->getModule()->par("address");
            cChannel * channel = node->getModule()->gate("port$o", i)->getTransmissionChannel();
            double cost = 1 / channel->getNominalDatarate();

            uint64_t val = channel->getNominalDatarate();
            uint64_t valaux = channel->getNominalDatarate() * 0.1;
            double cost2 = 1 / (val - valaux);
            addEdge(id, idNex, cost, cost, cost2);
        }
    }
}

void DijkstraKshortestFuzzy::setRouteMapK()
{
    kRoutesMap.clear();
    std::vector<NodeId> pathNode;
    for (auto it = routeMap.begin(); it != routeMap.begin(); ++it) {
        for (int i = 0; i < (int) it->second.size(); i++) {
            std::vector<NodeId> path;
            NodeId currentNode = it->first;
            int idx = it->second[i].idPrevIdx;
            while (currentNode != rootNode) {
                path.push_back(currentNode);
                currentNode = it->second[idx].idPrev;
                idx = it->second[idx].idPrevIdx;
                it = routeMap.find(currentNode);
                if (it == routeMap.end())
                    throw cRuntimeError("error in data");
                if (idx >= (int) it->second.size())
                    throw cRuntimeError("error in data");
            }
            pathNode.clear();
            while (!path.empty()) {
                pathNode.push_back(path.back());
                path.pop_back();
            }
            kRoutesMap[it->first].push_back(pathNode);
        }
    }
}

void DijkstraKshortestFuzzy::getRouteMapK(const NodeId &nodeId, Kroutes &routes)
{
    routes.clear();
    auto it = kRoutesMap.find(nodeId);
    if (it == kRoutesMap.end())
        return;
    routes = it->second;
}


unsigned int DijkstraKshortestFuzzy::commonLinks(const Route &S, const Route &Sp)
{
    int common = 0;
    for (unsigned int i = 1 ; i < S.size(); i++) {
        auto it = std::find(Sp.begin(), Sp.end(), S[i]);
        if (it != Sp.end()) { // common node, check link
            unsigned int pos = std::distance(Sp.begin(), it);
            if (pos == 0)
                continue;
            // check previous node
            if (S[i-1] == Sp[pos-1])
                common++;
        }
    }
    return (common);
}

