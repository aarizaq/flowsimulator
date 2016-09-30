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

#include "DijkstraFuzzy.h"

DijkstraFuzzy::FuzzyCost DijkstraFuzzy::minimumCost;
DijkstraFuzzy::FuzzyCost DijkstraFuzzy::maximumCost;
double DijkstraFuzzy::alpha = 0.5;

DijkstraFuzzy::State::State()
{
    idPrev = InvalidId;
}

DijkstraFuzzy::State::State(const FuzzyCost &costData)
{
    idPrev = InvalidId;
    cost = costData;
}

DijkstraFuzzy::State::~State()
{

}

void DijkstraFuzzy::State::setFuzzyCost(const FuzzyCost &costData)
{
    cost = costData;
}

void DijkstraFuzzy::initMinAndMax()
{
    minimumCost.cost1 = 0;
    minimumCost.cost2 = 0;
    minimumCost.cost3 = 0;
    maximumCost.cost1 = 1e30;
    maximumCost.cost2 = 1e30;
    maximumCost.cost3 = 1e30;
}

DijkstraFuzzy::DijkstraFuzzy()
{
    initMinAndMax();
}

DijkstraFuzzy::DijkstraFuzzy(const DijkstraFuzzy& other)
{
    kRoutesMap = other.kRoutesMap;

    while (!linkArray.empty()) {
        while (!linkArray.begin()->second.empty()) {
            delete linkArray.begin()->second.back();
            linkArray.begin()->second.pop_back();
        }
        linkArray.erase(linkArray.begin());
    }

    while (!uniqueLink.empty()) {
        while (!uniqueLink.begin()->second.empty()) {
            delete uniqueLink.begin()->second.back();
            uniqueLink.begin()->second.pop_back();
        }
        uniqueLink.erase(uniqueLink.begin());
    }

    linkArray = other.linkArray;
    for (auto elem : linkArray) {
        for (unsigned int i = 0; i < elem.second.size(); i++)
            elem.second[i] = elem.second[i]->dup();
    }

    uniqueLink = other.uniqueLink;
    for (auto elem : uniqueLink) {
        for (unsigned int i = 0; i < elem.second.size(); i++)
            elem.second[i] = elem.second[i]->dup();
    }

    routeMap = other.routeMap;
    rootNode = other.rootNode;
    K_LIMITE = other.K_LIMITE;
    limitsData = other.limitsData;
}

void DijkstraFuzzy::cleanLinkArray()
{
    for (LinkArray::iterator it = linkArray.begin(); it != linkArray.end(); it++)
        while (!it->second.empty()) {
            delete it->second.back();
            it->second.pop_back();
        }
    linkArray.clear();
}

void DijkstraFuzzy::clearAll()
{
    cleanLinkArray();
    kRoutesMap.clear();
    routeMap.clear();
}

DijkstraFuzzy::~DijkstraFuzzy()
{
    cleanLinkArray();
    kRoutesMap.clear();
}

void DijkstraFuzzy::addEdge(const NodeId & originNode, const NodeId & last_node, double cost, double cost2,
        double cost3)
{
    LinkArray::iterator it;
    it = linkArray.find(originNode);
    if (it != linkArray.end()) {
        for (unsigned int i = 0; i < it->second.size(); i++) {
            if (last_node == it->second[i]->last_node_) {
                if (it->second[i]->cost.cost1 != cost || it->second[i]->cost.cost2 != cost2 || it->second[i]->cost.cost3 != cost3) {
                    it->second[i]->cost.cost1 = cost;
                    it->second[i]->cost.cost2 = cost2;
                    it->second[i]->cost.cost3 = cost3;
                    kRoutesMap.clear();
                    routeMap.clear();
                }
                return;
            }
        }
    }
    Edge *link = new Edge;
    // The last hop is the interface in which we have this neighbor...
    link->last_node() = last_node;
    link->cost.cost1 = cost;
    link->cost.cost2 = cost2;
    link->cost.cost3 = cost3;
    kRoutesMap.clear();
    routeMap.clear();
    linkArray[originNode].push_back(link);
}

void DijkstraFuzzy::addEdge(const NodeId & originNode, Edge *edge)
{
    LinkArray::iterator it;
    it = linkArray.find(originNode);
    if (it != linkArray.end()) {
        for (unsigned int i = 0; i < it->second.size(); i++) {
            if (edge->last_node() == it->second[i]->last_node_) {
                delete it->second[i];
                it->second[i] = edge;
                kRoutesMap.clear();
                routeMap.clear();
                return;
            }
        }
    }
    linkArray[originNode].push_back(edge);
}

void DijkstraFuzzy::deleteEdge(const NodeId & originNode, const NodeId & last_node)
{
    DijkstraFuzzy::Edge *  e = removeEdge(originNode, last_node);
    if (e)
        delete e;
}


DijkstraFuzzy::Edge * DijkstraFuzzy::removeEdge(const NodeId & originNode, const NodeId & last_node)
{
    LinkArray::iterator it;
    it = linkArray.find(originNode);
    if (it != linkArray.end()) {
        for (auto itAux = it->second.begin(); itAux != it->second.end(); ++itAux) {
            Edge *edge = *itAux;
            if (last_node == edge->last_node_) {
                it->second.erase(itAux);
                kRoutesMap.clear();
                routeMap.clear();
                return edge;
            }
        }
    }
    return nullptr;
}

void DijkstraFuzzy::setRoot(const NodeId & dest_node)
{
    rootNode = dest_node;
}

void DijkstraFuzzy::run()
{
    run(linkArray, routeMap);
}

void DijkstraFuzzy::run(const LinkArray &linkArray, RouteMap & routeMap)
{
    std::deque<SetElem> heap;
    routeMap.clear();

    auto it = linkArray.find(rootNode);
    if (it == linkArray.end())
        throw cRuntimeError("Node not found");

    State state(minimumCost);
    routeMap[rootNode] = State(minimumCost);

    // include the neighbors of the root node

    auto linkIt = linkArray.find(rootNode);
    if (linkIt == linkArray.end())
        throw cRuntimeError("Error link not found in linkArray");

    for (unsigned int i = 0; i < it->second.size(); i++) {
        Edge* current_edge = (it->second)[i];
        State state;
        state.idPrev = rootNode;
        state.cost = current_edge->cost;
        routeMap[current_edge->last_node()] = state;
        SetElem newElem;
        newElem.iD = current_edge->last_node();
        newElem.cost = current_edge->cost;
        heap.push_back(newElem);
    }

    while (!heap.empty()) {
        // search min element
        auto minIt = heap.begin();
        for (auto itAux = heap.begin(); itAux != heap.end(); ++itAux) {
            if (itAux->cost < minIt->cost || (itAux->cost == minIt->cost && itAux->iD < minIt->iD))
                minIt = itAux;
        }
        SetElem elem = *minIt;
        heap.erase(minIt);
        auto it = routeMap.find(elem.iD);
        if (it == routeMap.end())
            throw cRuntimeError("node not found in routeMap");
        if (it->second.cost < elem.cost)
            continue;

        /// Record the route in the map
        RouteMap::iterator itAux = it;
        Route pathActive;
        Route pathNode;
        NodeId currentNode = elem.iD;
        while (currentNode != rootNode) {
            pathActive.push_back(currentNode);
            currentNode = itAux->second.idPrev;
            itAux = routeMap.find(currentNode);
            if (itAux == routeMap.end())
                throw cRuntimeError("error in data");
        }

        // next hop
        auto linkIt = linkArray.find(elem.iD);
        if (linkIt == linkArray.end())
            throw cRuntimeError("Error link not found in linkArray");

        for (unsigned int i = 0; i < linkIt->second.size(); i++) {
            Edge* current_edge = (linkIt->second)[i];
            FuzzyCost cost;

            // check if the node is in the path
            if (std::find(pathActive.begin(), pathActive.end(), current_edge->last_node()) != pathActive.end())
                continue;

            auto itNext = routeMap.find(current_edge->last_node());

            cost = current_edge->cost + it->second.cost;

            if (itNext == routeMap.end()) {
                State state;
                state.idPrev = elem.iD;
                state.cost = cost;
                routeMap[current_edge->last_node()] = state;
                SetElem newElem;
                newElem.iD = current_edge->last_node();
                newElem.cost = cost;
                heap.push_back(newElem);
            }
            else {
                if (cost < itNext->second.cost) {
                    itNext->second.cost = cost;
                    itNext->second.idPrev = elem.iD;
                    SetElem newElem;
                    newElem.iD = current_edge->last_node();
                    newElem.cost = cost;
                    heap.push_back(newElem);
                }
                else if (cost == itNext->second.cost && itNext->second.idPrev > elem.iD) {
                    itNext->second.cost = cost;
                    itNext->second.idPrev = elem.iD;
                    SetElem newElem;
                    newElem.iD = current_edge->last_node();
                    newElem.cost = cost;
                    heap.push_back(newElem);
                }
            }
        }
    }
}

void DijkstraFuzzy::runUntil(const NodeId &target, const LinkArray &linkArray, RouteMap & routeMap)
{
    std::deque<SetElem> heap;
    routeMap.clear();

    auto it = linkArray.find(rootNode);
    if (it == linkArray.end())
        throw cRuntimeError("Node not found");

    routeMap[rootNode] = State(minimumCost);

    for (unsigned int i = 0; i < it->second.size(); i++) {
        Edge* current_edge = (it->second)[i];
        State state;
        state.idPrev = rootNode;
        state.cost = current_edge->cost;
        routeMap[current_edge->last_node()] = state;
        SetElem newElem;
        newElem.iD = current_edge->last_node();
        newElem.cost = current_edge->cost;
        heap.push_back(newElem);
    }

    while (!heap.empty()) {
        auto minIt = heap.begin();
        for (auto itAux = heap.begin(); itAux != heap.end(); ++itAux) {
            if (itAux->cost < minIt->cost || (itAux->cost == minIt->cost && itAux->iD < minIt->iD))
                minIt = itAux;
        }
        SetElem elem = *minIt;
        heap.erase(minIt);
        auto it = routeMap.find(elem.iD);
        if (it == routeMap.end())
            throw cRuntimeError("node not found in routeMap");
        if (it->second.cost < elem.cost)
            continue;
        /// Record the route in the map
        RouteMap::iterator itAux = it;
        Route pathActive;
        Route pathNode;
        NodeId currentNode = elem.iD;
        while (currentNode != rootNode) {
            // check if the node is in the path
            if (std::find(pathActive.begin(),pathActive.end(),currentNode) != pathActive.end())
                throw cRuntimeError("error in data");;
            pathActive.push_back(currentNode);

            currentNode = itAux->second.idPrev;
            itAux = routeMap.find(currentNode);
            if (itAux == routeMap.end())
                throw cRuntimeError("error in data");
        }

        if (target == elem.iD)
            return;

        // next hop
        auto linkIt = linkArray.find(elem.iD);
        if (linkIt == linkArray.end())
            throw cRuntimeError("Error link not found in linkArray");

        for (unsigned int i = 0; i < linkIt->second.size(); i++) {
            Edge* current_edge = (linkIt->second)[i];
            FuzzyCost cost;

            // check if the node is in the path
            if (std::find(pathActive.begin(),pathActive.end(),current_edge->last_node()) != pathActive.end())
                continue;

            RouteMap::iterator itNext = routeMap.find(current_edge->last_node());

            cost = current_edge->cost + it->second.cost;

            if (itNext == routeMap.end()) {
                State state;
                state.idPrev = elem.iD;
                state.cost = cost;
                routeMap[current_edge->last_node()] = state;
                SetElem newElem;
                newElem.iD = current_edge->last_node();
                newElem.cost = cost;
                heap.push_back(newElem);
            }
            else {
                if (cost < itNext->second.cost) {
                    itNext->second.cost = cost;
                    itNext->second.idPrev = elem.iD;
                    SetElem newElem;
                    newElem.iD = current_edge->last_node();
                    newElem.cost = cost;
                    heap.push_back(newElem);
                }
                else if (cost == itNext->second.cost && itNext->second.idPrev > elem.iD) {
                    itNext->second.cost = cost;
                    itNext->second.idPrev = elem.iD;
                    SetElem newElem;
                    newElem.iD = current_edge->last_node();
                    newElem.cost = cost;
                    heap.push_back(newElem);
                }
            }
        }
    }
}

void DijkstraFuzzy::runDisjoint(const NodeId &rootNode, const NodeId &target, NodePairs &partitionLinks, RouteMap &routeMap,const LinkArray &linkArray, MapRoutes &kRoutesMap)
{
    std::multiset<SetElem> heap;
    RouteMap routeMapdisj;

    Route minPath;
    std::vector<DijkstraFuzzy::Edge*> tempEdges;

    FuzzyCost cost;
    if (!getRoute(target, minPath, routeMap, cost)) {
        run();
    }

    if (!getRoute(target, minPath, routeMap, cost)) {
        return;
        // grafo particionado, no hay camino.
        throw cRuntimeError("Imposible encontrar ruta");
    }
    // test
    if (!getHasFindDisjoint()) {
        discoverAllPartitionedLinks(partitionLinks);
        setHasFindDisjoint(true);
    }
    // check if partition links
    NodePairs pairs;
    for (unsigned int i = 0; i < minPath.size()-1; i++) {
        auto itAux = std::find(partitionLinks.begin(),partitionLinks.end(),std::make_pair(minPath[i],minPath[i+1]));
        if (itAux !=  partitionLinks.end()) {
            pairs.push_back(*itAux);
        }
    }
    if (!pairs.empty()) { // critical link it is necessary search the route in other form
        // busco camino parciales.
        // descompongo las rutas.
        NodeId target1 = pairs.front().first;
        NodeId root1 = pairs.front().second;
        RouteMap routeMapAux;
        MapRoutes kRoutesMapAux;
        Route auxPath1;
        Route auxPath2;
        if (rootNode != target1 && root1 != target) {
            runDisjoint(rootNode, target1, partitionLinks, routeMap, linkArray, kRoutesMap);
            runDisjoint(root1, target, partitionLinks, routeMapAux, linkArray, kRoutesMapAux);
            auto it = kRoutesMap.find(target1);
            auto itAux = kRoutesMapAux.find(target);
            // extract routes and include it in the map
            for (int i = 0; i < 2; i++) {
                for (auto p = it->second[0].begin(); p != it->second[0].end(); ++p)
                    auxPath1.push_back(*p);
                for (auto p = it->second[1].begin(); p != it->second[1].end(); ++p)
                    auxPath2.push_back(*p);
            }
            for (int i = 0; i < 2; i++) {
                for (auto p = itAux->second[0].begin(); p != itAux->second[0].end(); ++p)
                    auxPath1.push_back(*p);
                for (auto p = itAux->second[1].begin(); p != itAux->second[1].end(); ++p)
                    auxPath2.push_back(*p);
            }
        }
        else if (rootNode == target1) {
            runDisjoint(root1, target, partitionLinks, routeMapAux, linkArray, kRoutesMapAux);
            auto itAux = kRoutesMapAux.find(target);
            // extract routes and include it in the map
            auxPath1.clear();
            auxPath2.clear();
            auxPath1.push_back(rootNode);
            auxPath2.push_back(rootNode);
            for (auto elem : itAux->second[0])
                auxPath1.push_back(elem);
            for (auto elem : itAux->second[1])
                auxPath2.push_back(elem);
        }
        else if (root1 == target) {
            runDisjoint(rootNode, target1, partitionLinks, routeMap, linkArray, kRoutesMap);
            auto it = kRoutesMap.find(target1);
            // extract routes and include it in the map
            auxPath1.clear();
            auxPath2.clear();
            for (auto elem : it->second[0])
                auxPath1.push_back(elem);
            for (auto elem : it->second[1])
                auxPath2.push_back(elem);
            auxPath1.push_back(target);
            auxPath2.push_back(target);
        }
        auto it = kRoutesMap.find(target);
        if (it == kRoutesMap.end()) {
            Kroutes rout;
            rout.push_back(auxPath1);
            rout.push_back(auxPath2);
            kRoutesMap[target] = rout;
        }
        else {
            it->second.clear();
            it->second.push_back(auxPath1);
            it->second.push_back(auxPath2);
        }
    }
/*
    int test[12];
    int test2[12];
    for (int i = 0; i < 12; i++)
        test[i] = test2[i] = -1;
    if (minPath.size() > 12)
        throw cRuntimeError("quitar test");
    for (unsigned int i = 0; i < minPath.size(); i++)
        test[i] = minPath[i];*/
    // comprobar si la ruta usa enlaces que particionan la red.

    LinkArray linkArrayMod = linkArray;
    // creamos el nuevo mapa
    for (unsigned int i = 0; i < minPath.size() - 1; i++) {
        int nodoInicial = minPath[i];
        int nodoFinal = minPath[i + 1];
        auto it = linkArrayMod.find(nodoInicial);
        if (it == linkArrayMod.end())
            throw cRuntimeError("Nodo no encontrado en la matriz de conexión");
        for (auto itAuxVec = it->second.begin(); itAuxVec != it->second.end(); ++itAuxVec) {
            // cancelo en enlace directo y pongo al arco en direccion contraria el coste inverso del arco directo

            Edge* edge = *itAuxVec;
            if (edge->last_node() == nodoFinal) {
                Edge* newedge = new Edge();
                newedge->last_node() = nodoInicial;
                newedge->cost.cost1 = -edge->cost.cost3;
                newedge->cost.cost2 = -edge->cost.cost2;
                newedge->cost.cost3 = -edge->cost.cost1;
                tempEdges.push_back(newedge);

                auto itAux = linkArrayMod.find(nodoFinal);
                if (itAux == linkArrayMod.end())
                    throw cRuntimeError("Nodo no encontrado en la matriz de conexión");
                for (auto &elem : itAux->second) {
                    if (elem->last_node() == nodoInicial) {
                        elem = newedge; // arco inverso con el coste invertido
                        break;
                    }
                }
                it->second.erase(itAuxVec); // elimino el arco directo
                break;
            }
        }
    }

    runUntil(target, linkArrayMod, routeMapdisj);

    Route minPathD;

    FuzzyCost cost2;
    if (!getRoute(target, minPathD, routeMapdisj, cost2)) {
        // grafo particionado, no se pueden quitar todos los enlaces.
        throw cRuntimeError("Imposible encontrar ruta");
    }
/*
    if (minPathD.size() > 12)
        throw cRuntimeError("quitar test");
    for (unsigned int i = 0; i < minPathD.size(); i++)
        test2[i] = minPathD[i];
*/

    BreaksVect Vect_breaks;
    breaks(minPath, minPathD, Vect_breaks);

    while (!tempEdges.empty()) {
        delete tempEdges.back();
        tempEdges.pop_back();
    }

    Route gamma1;
    Route gamma2;

    if (!Vect_breaks.empty()) {
        Pair_Paths(minPath, minPathD, Vect_breaks, linkArray, rootNode, target, gamma1, gamma2);
    }
    else {
        gamma1 = minPath;
        gamma2 = minPathD;
    }

    FuzzyCost costG1,costG2;
    // get the cost of the routes

    getCostPath(gamma1, linkArray, costG1);
    getCostPath(gamma2, linkArray, costG2);

    // store the routes
    auto it = kRoutesMap.find(target);

    if (it == kRoutesMap.end()) {
        Kroutes rout;
        rout.push_back(gamma1);
        rout.push_back(gamma2);
        kRoutesMap[target] = rout;
    }
    else {
        it->second.clear();
        it->second.push_back(gamma1);
        it->second.push_back(gamma2);
    }
}

void DijkstraFuzzy::runDisjoint(const NodeId &target)
{
    runDisjoint(rootNode, target, partitionLinks, routeMap,linkArray, kRoutesMap);
}

bool DijkstraFuzzy::checkDisjoint(const NodeId &nodeId, Route & r1, Route &r2) {
    auto it = kRoutesMap.find(nodeId);
    if (it == kRoutesMap.end())
        return false;

    r1 = it->second[0];
    r2 = it->second[1];

    return true;
}

int DijkstraFuzzy::getNumRoutes(const NodeId &nodeId)
{
    RouteMap::iterator it;
    it = routeMap.find(nodeId);
    if (it == routeMap.end())
        return -1;
    return 1;
}

bool DijkstraFuzzy::getRoute(const NodeId &nodeId, std::vector<NodeId> &pathNode, FuzzyCost &cost)
{
    return getRoute(nodeId, pathNode, routeMap, cost);
}

bool DijkstraFuzzy::getRoute(const NodeId &nodeId, std::vector<NodeId> &pathNode, const RouteMap &routeMap,
        FuzzyCost &cost)
{
    auto it = routeMap.find(nodeId);
    if (it == routeMap.end())
        return false;

    std::vector<NodeId> path;
    NodeId currentNode = nodeId;
    cost = it->second.cost;

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
    return true;
}

void DijkstraFuzzy::setFromTopo(const cTopology *topo)
{
    for (int i = 0; i < topo->getNumNodes(); i++) {
        cTopology::Node *node = const_cast<cTopology*>(topo)->getNode(i);
        NodeId id = node->getModuleId();
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

void DijkstraFuzzy::breaks(const Route &S, const Route &Sp, BreaksVect &Vect_breaks)
{
    std::vector<int> Com_verts;
    for (auto& el1 : S) {
        for (auto& el2 : Sp) {
            if (el1 == el2) {
                // comprobar duplicados primero
                auto it = std::find(Com_verts.begin(), Com_verts.end(), el2);
                if (it != Com_verts.end())
                    throw cRuntimeError("Camino con bucles");
                Com_verts.push_back(el2);
            }
        }
    }

    for (unsigned int i = 0; i < Com_verts.size() - 1; i++) {
        std::vector<int> pos1;
        auto itS = std::find(S.begin(), S.end(), Com_verts[i]);
        auto itSp = std::find(Sp.begin(), Sp.end(), Com_verts[i]);
        pos1.push_back(std::distance(S.begin(), itS));
        pos1.push_back(std::distance(Sp.begin(), itSp));

        for (unsigned int j = i + 1; j < Com_verts.size(); j++) {

            auto itSNext = std::find(S.begin(), S.end(), Com_verts[j]);
            auto itSpNext = std::find(Sp.begin(), Sp.end(), Com_verts[j]);

            std::vector<int> next;
            next.push_back(std::distance(S.begin(), itSNext));
            next.push_back(std::distance(Sp.begin(), itSpNext));

            if (abs(pos1[0] - next[0]) == 1 && abs(pos1[1] - next[1]) == 1) {
                Vect_breaks.push_back(Com_verts[i]);
                Vect_breaks.push_back(Com_verts[j]);
            }
        }
    }
}

NodeId findnext(const NodeId &v_ref, const DijkstraFuzzy::Route &path, const DijkstraFuzzy::LinkArray &linkArray)
{
// Funcion para encontrar el proximo vértice de un camino "Path" que es
// adyacente al vertice v_ref

    auto itAux = std::find(path.begin(), path.end(), v_ref);
    ++itAux;
    auto it = linkArray.find(v_ref);
    if (it == linkArray.end())
        throw cRuntimeError("Node not found in linkArray");
    for (; itAux < path.end(); ++itAux) {
        for (auto &elem : it->second) {
            if (elem->last_node() == *itAux)
                return elem->last_node();
        }
    }
    return -1;
}

// CONSTRUYENDO GAMMA_1
void DijkstraFuzzy::buildGamma1(Route& nextV_S, Route& nextV_Sp, const Route &Sprima, const Route &Spprima,
        const BreaksVect &Vect_breaks, const LinkArray &A, const NodeId &s, const NodeId &t, Route &gamma)
{
    gamma.push_back(s);
    NodeId v_ref = s;
    int ind_next = 0;

    while (v_ref != t) {
        if (v_ref == s) {
            //%         next_v en Sprima
            auto it = std::find(Sprima.begin(), Sprima.end(), v_ref);
            v_ref = *(it + 1);
            gamma.push_back(v_ref);

        }
        bool found = false;
        auto itvecbreak = std::find(Vect_breaks.begin(), Vect_breaks.end(), v_ref);
        if (itvecbreak != Vect_breaks.end())
            found = true;

        Route candidates;
        if (found) {
            auto it = std::find(Sprima.begin(), Sprima.end(), v_ref);
            if (it != Sprima.end()) {
                auto itNode = A.find(v_ref);
                for (auto & elem : itNode->second) {
                    auto itAux = std::find(nextV_Sp.begin(), nextV_Sp.end(), elem->last_node());
                    if (itAux != nextV_Sp.end())
                        candidates.push_back(elem->last_node());
                }
                ind_next = 1;
            }
            else {
                auto it2 = std::find(Spprima.begin(), Spprima.end(), v_ref);
                if (it2 != Spprima.end()) {
                    auto itNode = A.find(v_ref);
                    for (auto & elem : itNode->second) {
                        auto itAux = std::find(nextV_S.begin(), nextV_S.end(), elem->last_node());
                        if (itAux != nextV_S.end())
                            candidates.push_back(elem->last_node());
                    }
                    ind_next = 0;
                }
            }
            if (candidates.size() == 1) {
                v_ref = candidates.front();
                gamma.push_back(v_ref);
                if (ind_next == 1) {
                    auto itAux = std::find(nextV_Sp.begin(), nextV_Sp.end(), v_ref);
                    if (itAux != nextV_Sp.end())
                        nextV_Sp.erase(itAux);
                }
                else {
                    auto itAux = std::find(nextV_S.begin(), nextV_S.end(), v_ref);
                    if (itAux != nextV_S.end())
                        nextV_S.erase(itAux);
                }
            }
            else if (candidates.size() > 1) {
                Route lista = candidates;

                for (unsigned int i = 0; i < candidates.size(); i++) {
                    for (unsigned int j = 0; j < Vect_breaks.size(); j += 2) //length(Vect_breaks(:,1))+1)
                            {
                        if ((Vect_breaks[j] == v_ref || Vect_breaks[j + 1] == v_ref)
                                && (Vect_breaks[j] == candidates[i] || Vect_breaks[j + 1] == candidates[i])) {
                            auto itAux = std::find(lista.begin(), lista.end(), candidates[i]);
                            if (itAux != lista.end())
                                lista.erase(itAux);
                            // check
                            itAux = std::find(lista.begin(), lista.end(), candidates[i]);
                            if (itAux != lista.end())
                                throw cRuntimeError("nodo duplicado");
                        }
                    }
                }
                candidates = lista;
                v_ref = candidates.front();
                gamma.push_back(v_ref);

                auto itAux1 = std::find(Sprima.begin(), Sprima.end(), v_ref);
                if (itAux1 != Sprima.end()) {
                    auto itAux = std::find(nextV_S.begin(), nextV_S.end(), v_ref);
                    if (itAux != nextV_S.end())
                        nextV_S.erase(itAux);
                }
                else {
                    auto itAux1 = std::find(Spprima.begin(), Spprima.end(), v_ref);
                    if (itAux1 != Spprima.end()) {
                        auto itAux = std::find(nextV_Sp.begin(), nextV_Sp.end(), v_ref);
                        if (itAux != nextV_Sp.end())
                            nextV_Sp.erase(itAux);
                    }

                }
            }
            else if (candidates.empty()) {

            }
        }
        else {
            // Buscar el siguiente vertice del camino actual (Sprima o Spprima)
            //        que sea adyacente a v_ref
            auto itAux1 = std::find(Sprima.begin(), Sprima.end(), v_ref);

            if (itAux1 != Sprima.end()) {
                NodeId vertice = findnext(v_ref, Sprima, A);
                if (vertice != -1)
                    v_ref = vertice;
            }
            else {
                auto itAux1 = std::find(Spprima.begin(), Spprima.end(), v_ref);
                if (itAux1 != Spprima.end()) {
                    NodeId vertice = findnext(v_ref, Spprima, A);
                    if (vertice != -1)
                        v_ref = vertice;
                }
            }
            gamma.push_back(v_ref);
        }
    }
}

// CONSTRUYENDO GAMMA_1
void DijkstraFuzzy::buildGamma2(Route& nextV_S, Route& nextV_Sp, const Route &Sprima, const Route &Spprima,
        const BreaksVect &Vect_breaks, const LinkArray &A, const NodeId &s, const NodeId &t, const Route &gammaAux,
        Route &gamma)
{
    gamma.push_back(s);
    NodeId v_ref = s;
    int ind_next = 0;

    while (v_ref != t) {
        if (v_ref == s) {
            //%         next_v en Sprima
            auto it = std::find(Spprima.begin(), Spprima.end(), v_ref);
            v_ref = *(it + 1);
            gamma.push_back(v_ref);

        }
        bool found = false;
        auto itvecbreak = std::find(Vect_breaks.begin(), Vect_breaks.end(), v_ref);
        if (itvecbreak != Vect_breaks.end())
            found = true;

        Route candidates;
        if (found) {
            auto it = std::find(Sprima.begin(), Sprima.end(), v_ref);

            if (it != Sprima.end()) {
                auto itNode = A.find(v_ref);
                for (auto & elem : itNode->second) {
                    auto itAux = std::find(nextV_Sp.begin(), nextV_Sp.end(), elem->last_node());
                    if (itAux != nextV_Sp.end())
                        candidates.push_back(elem->last_node());
                }
                ind_next = 1;
            }
            else {
                auto it2 = std::find(Spprima.begin(), Spprima.end(), v_ref);
                if (it2 != Spprima.end()) {
                    auto itNode = A.find(v_ref);
                    for (auto & elem : itNode->second) {
                        auto itAux = std::find(nextV_S.begin(), nextV_S.end(), elem->last_node());
                        if (itAux != nextV_S.end())
                            candidates.push_back(elem->last_node());
                    }
                    ind_next = 0;
                }
            }
            if (candidates.size() == 1) {
                v_ref = candidates.front();
                gamma.push_back(v_ref);
                if (ind_next == 1) {
                    auto itAux = std::find(nextV_Sp.begin(), nextV_Sp.end(), v_ref);
                    if (itAux != nextV_Sp.end())
                        nextV_Sp.erase(itAux);
                }
                else {
                    auto itAux = std::find(nextV_S.begin(), nextV_S.end(), v_ref);
                    if (itAux != nextV_S.end())
                        nextV_S.erase(itAux);
                }
            }
            else if (candidates.size() > 1) {
                Route lista = candidates;

                for (unsigned int i = 0; i < candidates.size(); i++) {
                    for (unsigned int j = 0; j < Vect_breaks.size(); j += 2) //length(Vect_breaks(:,1))+1)
                            {
                        if ((Vect_breaks[j] == v_ref || Vect_breaks[j + 1] == v_ref)
                                && (Vect_breaks[j] == candidates[i] || Vect_breaks[j + 1] == candidates[i])) {
                            auto itAux = std::find(lista.begin(), lista.end(), candidates[i]);
                            if (itAux != lista.end())
                                lista.erase(itAux);
                            // check
                            itAux = std::find(lista.begin(), lista.end(), candidates[i]);
                            if (itAux != lista.end())
                                throw cRuntimeError("nodo duplicado");
                        }
                    }
                }
                candidates = lista;
                v_ref = candidates.front();
                gamma.push_back(v_ref);

                auto itAux1 = std::find(Sprima.begin(), Sprima.end(), v_ref);
                if (itAux1 != Sprima.end()) {
                    auto itAux = std::find(nextV_S.begin(), nextV_S.end(), v_ref);
                    if (itAux != nextV_S.end())
                        nextV_S.erase(itAux);
                }
                else {
                    auto itAux1 = std::find(Spprima.begin(), Spprima.end(), v_ref);
                    if (itAux1 != Spprima.end()) {
                        auto itAux = std::find(nextV_Sp.begin(), nextV_Sp.end(), v_ref);
                        if (itAux != nextV_Sp.end())
                            nextV_Sp.erase(itAux);
                    }

                }
            }
            else if (candidates.empty()) {

            }
        }
        else {
            // Buscar el siguiente vertice del camino actual (Sprima o Spprima)
            //        que sea adyacente a v_ref
            auto itAux1 = std::find(Sprima.begin(), Sprima.end(), v_ref);
            auto itAux2 = std::find(Spprima.begin(), Spprima.end(), v_ref);
            if (itAux1 != Sprima.end() && itAux2 == Spprima.end()) {
                NodeId vertice = findnext(v_ref, Sprima, A);
                if (vertice != -1) {
                    auto itAux = std::find(gammaAux.begin(), gammaAux.end(), vertice);
                    if ((vertice == t) || itAux == gammaAux.end())
                        v_ref = vertice;
                }
            }
            else if (itAux2 != Spprima.end()) {
                NodeId vertice = findnext(v_ref, Spprima, A);
                if (vertice != -1) {
                    auto itgamaAux = std::find(gammaAux.begin(), gammaAux.end(), vertice);
                    auto itvecBreacks = std::find(Vect_breaks.begin(), Vect_breaks.end(), vertice);
                    if ((vertice == t) || itgamaAux == gammaAux.end() || itvecBreacks == Vect_breaks.end())
                        v_ref = vertice;
                }
            }
            auto itAuxCheck = std::find(gamma.begin(), gamma.end(), v_ref);
            if (itAuxCheck != gamma.end())
                throw cRuntimeError("Vref en gamma");
            gamma.push_back(v_ref);

        }
    }
}

void DijkstraFuzzy::Pair_Paths(const Route &S, const Route &Sp, BreaksVect &Vect_breaks, const LinkArray &A,
        const NodeId& s, const NodeId &t, Route &gamma1, Route &gamma2)
{
    //  Pair_Paths: Construccion del par de caminos arista disjuntos
    // ------------------------------------------------------------------------

//ind_next: indica donde se encuentran los vertices candidatos a ser el
    // siguiente en el camino. ind_next =1 (Esta(n) en nextV_Sp), ind_next =0 (Esta(n) en nextV_S),

    Route remv_S;
    Route remv_Sp;
    Route nextV_S;
    Route nextV_Sp;

    std::vector<unsigned int> posSp;
    std::vector<unsigned int> posS;

    for (unsigned int i = 0; i < Vect_breaks.size() / 2; i++) {
        //     Detectar posicion del segundo vertice que conforma el break en S y Sp
        auto itS = std::find(S.begin(), S.end(), Vect_breaks[2 * i]);
        auto itS1 = std::find(S.begin(), S.end(), Vect_breaks[(2 * i) + 1]);
        posS.push_back(std::distance(S.begin(), itS));
        posS.push_back(std::distance(S.begin(), itS1));

        auto itSp = std::find(Sp.begin(), Sp.end(), Vect_breaks[2 * i]);
        auto itSp1 = std::find(Sp.begin(), Sp.end(), Vect_breaks[(2 * i) + 1]);
        posSp.push_back(std::distance(Sp.begin(), itSp));
        posSp.push_back(std::distance(Sp.begin(), itSp1));

        int positionS = *std::max_element(posS.begin(), posS.end());
        int positionSp = *std::max_element(posSp.begin(), posSp.end());

        remv_S.push_back(S[positionS]);
        remv_Sp.push_back(Sp[positionSp]);

        nextV_S.push_back(S[positionS + 1]);
        nextV_Sp.push_back(Sp[positionSp + 1]);
    }

    Route sprima = S;
    Route spprima = Sp;

// Eliminar los segundos vertices de todos los breaks en S y Sp
    for (unsigned int i = 0; i < remv_S.size(); i++) {
        auto it = std::find(sprima.begin(), sprima.end(), remv_S[i]);
        sprima.erase(it);
    }
    for (unsigned int i = 0; i < remv_Sp.size(); i++) {
        auto it = std::find(spprima.begin(), spprima.end(), remv_Sp[i]);
        spprima.erase(it);
    }

    buildGamma1(nextV_S, nextV_Sp, sprima, spprima, Vect_breaks, A, s, t, gamma1);
    buildGamma2(nextV_S, nextV_Sp, sprima, spprima, Vect_breaks, A, s, t, gamma1, gamma2);

}

void DijkstraFuzzy::discoverAllPartitionedLinks(const LinkArray & topo, NodePairs &links)
{
    Dijkstra dj;
    for (auto elem : topo) {
        for (auto elem2 : elem.second) {
            dj.addEdge(elem.first,elem2->last_node(),1,10000);
        }
    }
    dj.discoverAllPartitionedLinks(links);
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

}

void Dijkstra::cleanLinkArray()
{
    cleanLinkArray(linkArray);
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

void Dijkstra::addEdge(const NodeId & originNode, const NodeId & last_node, double cost, double cost2, LinkArray & linkArray)
{
    auto it = linkArray.find(originNode);
    if (it != linkArray.end()) {
        for (unsigned int i = 0; i < it->second.size(); i++) {
            if (last_node == it->second[i]->last_node_) {
                // check changes in the cost
                if (it->second[i]->Cost() != cost) {
                    it->second[i]->Cost() = cost;
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
                return edge;
            }
        }
    }
    return nullptr;
}

void Dijkstra::addEdge(const NodeId & originNode, const NodeId & last_node, double cost,  double cost2)
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
    std::multiset<SetElem> heap;
    routeMap.clear();

    auto it = linkArray.find(rootNode);
    if (it == linkArray.end())
        throw cRuntimeError("Node not found");

    State state(0,1e300);
    state.label = perm;
    routeMap[rootNode] = state;

    SetElem elem;
    elem.iD = rootNode;
    elem.cost = 0;
    heap.insert(elem);
    while (!heap.empty()) {
        SetElem elem = *heap.begin();
        heap.erase(heap.begin());
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

            double maxCost = itNext->second.cost;
            double maxCost2 = itNext->second.cost2;

            if (itNext == routeMap.end()) {
                State state;
                state.idPrev = elem.iD;
                state.cost = cost;
                state.label = tent;
                routeMap[current_edge->last_node()] = state;

                SetElem newElem;
                newElem.iD = current_edge->last_node();
                newElem.cost = cost;
                heap.insert(newElem);
            }
            else {
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
                    for (auto it = heap.begin(); it != heap.end(); ++it) {
                        if (it->iD == newElem.iD && it->cost > newElem.cost) {
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

bool Dijkstra::getRoute(const NodeId &nodeId, std::vector<NodeId> &pathNode, const RouteMap &routeMap)
{
    auto it = routeMap.find(nodeId);
    if (it == routeMap.end())
        return false;
    std::vector<NodeId> path;
    NodeId currentNode = nodeId;

    while (currentNode != rootNode) {
        path.push_back(currentNode);
        currentNode = it->second.idPrev;
        it = routeMap.find(currentNode);
        if (it == routeMap.end())
            throw cRuntimeError("error in data");
    }
    pathNode.clear();
    while (!path.empty()) {
        pathNode.push_back(path.back());
        path.pop_back();
    }
    return true;

}

bool Dijkstra::getRoute(const NodeId &nodeId, std::vector<NodeId> &pathNode)
{
    return getRoute(nodeId, pathNode, routeMap);
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
    for (int i = 0; i < pathNode.size() - 1; i++) {
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

bool DijkstraFuzzy::getCostPath(const Route &route, const LinkArray &linkArray, FuzzyCost &cost) {

    FuzzyCost costAux;
    for (unsigned int i = 0; i < route.size() - 1; i++) {

        NodeId ini = route[i];
        NodeId end = route[i + 1];
        auto linkIt = linkArray.find(ini);
        if (linkIt == linkArray.end())
            throw cRuntimeError("Error link not found in linkArray");
        Edge* e = nullptr;
        for (unsigned int i = 0; i < linkIt->second.size(); i++) {
            Edge* current_edge = (linkIt->second)[i];
            if (current_edge->last_node() == end) {
                e = current_edge;
                break;
            }
        }
        if (e == nullptr)
            return false;
        costAux = costAux + e->cost;
    }
    cost = costAux;
    return true;

}
