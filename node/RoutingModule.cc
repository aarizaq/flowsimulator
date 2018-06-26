//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#include "RoutingModule.h"


Define_Module(RoutingModule);

simsignal_t RoutingModule::actualizationPortsSignal = registerSignal("actualizationPortsSignal");
simsignal_t RoutingModule::actualizationSignal = registerSignal("actualizationSignal");
simsignal_t RoutingModule::changeRoutingTableSignal = registerSignal("changeRoutingTableSignal");



RoutingModule::RoutingModule() {
    // TODO Auto-generated constructor stub

}

RoutingModule::~RoutingModule() {
    // TODO Auto-generated destructor stub
    if (dijFuzzy)
        delete dijFuzzy;
    if (dijkstra)
        delete dijkstra;
    cancelAndDelete(nextAct);
}

void RoutingModule::setRoutingType(const IRouting::RoutingType & a)
{
    rType = a;
}

IRouting::RoutingType RoutingModule::getRoutingType()
{
    return rType;
}

void RoutingModule::getPairRoutes(const int & destAddress, std::vector<int> &path, std::vector<int> &path2, const bool & forceDisj) {

    if (forceDisj || rType == DISJOINT) {
        DijkstraFuzzy::Route r1, r2, min;
        dijFuzzy->setRoot(myAddress);
        dijFuzzy->runDisjoint(destAddress);
        //dijFuzzy->getRoute(destAddress, min, cost);
        if (dijFuzzy->checkDisjoint(destAddress, r1, r2)) {
            DijkstraFuzzy::FuzzyCost costr1, costr2;
            dijFuzzy->getCostPath(r1, costr1);
            dijFuzzy->getCostPath(r2, costr2);
            double total = costr1.exp() + costr2.exp();

            DijkstraFuzzy::Route *r =
                    uniform(0, total) < costr1.exp() ? &r2 : &r1;
            path = *r;
            if (r == &r2)
                path2 = r1;
            else
                path2 = r2;
        }
    }
    else if (rType == SOURCEROUTING) {
        DijkstraFuzzy::Route min;
        DijkstraFuzzy::FuzzyCost cost;
        dijFuzzy->setRoot(myAddress);
        path.clear();
        path2.clear();
        if (!dijFuzzy->getRoute(destAddress, min, cost))
            dijFuzzy->run();
        //dijFuzzy->getRoute(destAddress, min, cost);
        if (dijFuzzy->getRoute(destAddress, min, cost)) {
            path = min;
        }
    }
    else if (rType == SOURCEROUTINGNORMAL) {
        Dijkstra::Route min;
        path.clear();
        path2.clear();
        dijkstra->setRoot(myAddress);
        if (!dijkstra->getRoute(destAddress, min))
            dijkstra->run();
        //dijFuzzy->getRoute(destAddress, min, cost);
        if (dijkstra->getRoute(destAddress, min)) {
            path = min;
        }
    }
    else if (rType == SW || rType == SWFUZZY) {
        Dijkstra::Route min;
        path.clear();
        path2.clear();
        dijkstra->setRoot(myAddress);
        dijkstra->setMethod(Dijkstra::Method::shortestwidest);
        if (!dijkstra->getRoute(destAddress, min))
            dijkstra->run();
        //dijFuzzy->getRoute(destAddress, min, cost);
        if (dijkstra->getRoute(destAddress, min)) {
            path = min;
        }
    }
    else if (rType == WS || rType == WSFUZZY) {
        Dijkstra::Route min;
        path.clear();
        path2.clear();
        dijkstra->setRoot(myAddress);
        dijkstra->setMethod(Dijkstra::Method::widestshortest);
        if (!dijkstra->getRoute(destAddress, min))
            dijkstra->run();
        //dijFuzzy->getRoute(destAddress, min, cost);
        if (dijkstra->getRoute(destAddress, min)) {
            path = min;
        }
    } else if (rType == BACKUPROUTE) {
        // TODO: backup mode Se deben enviar dos paquetes, uno por cada ruta
        path.clear();
        path2.clear();
        DijkstraFuzzy::Route r1, r2, min;
        dijFuzzy->setRoot(myAddress);
        dijFuzzy->runDisjoint(destAddress);
        //dijFuzzy->getRoute(destAddress, min, cost);
        if (dijFuzzy->checkDisjoint(destAddress, r1, r2)) {
            DijkstraFuzzy::FuzzyCost costr1, costr2;
            dijFuzzy->getCostPath(r1, costr1);
            dijFuzzy->getCostPath(r2, costr2);

            if (costr1 < costr2) {
                path = r1;
                path2 = r2;

            } else {
                path = r2;
                path2 = r1;
            }
        }
    }
    else if (rType == BACKUPROUTEKSH) {
        // TODO: backup mode Se deben enviar dos paquetes, uno por cada ruta
        if (dijkstraks == nullptr)
            throw cRuntimeError("dijkstraks is null");
        path.clear();
        path2.clear();

        DijkstraKshortest::Kroutes routes;
        dijkstraks->setRoot(myAddress);
        dijkstraks->getRouteMapK(destAddress, routes);
        if (routes.empty())
            dijkstraks->runUntil(destAddress);
        dijkstraks->getRouteMapK(destAddress, routes);
        DijkstraKshortest::Route *route1 = nullptr;
        DijkstraKshortest::Route *route2 = nullptr;
        //dijFuzzy->getRoute(destAddress, min, cost);
        if (!routes.empty()) {
            // search two routes with less common links
            unsigned int minCommon = 1000;
            unsigned int totalSizes = 10000;
            for (unsigned int i = 0; i < routes.size() - 1; i++) {
                for (unsigned int j = i + 1; j < routes.size() - 1; j++) {
                    unsigned int common = dijkstraks->commonLinks(routes[i],
                            routes[j]);
                    if (minCommon > common || (minCommon == common  && totalSizes > routes[i].size() + routes[j].size())) {
                        route1 = &routes[i];
                        route2 = &routes[j];
                        totalSizes = routes[i].size() + routes[j].size();
                        minCommon = common;
                    }
                }
            }
            if (route2->size() > route1->size()) {
                path = *route1;
                path2 = *route2;
            } else {
                path = *route2;
                path2 = *route1;
            }
        }
    }
}


void RoutingModule::readTopo()
{
    if (dijFuzzy == nullptr) {
        dijFuzzy = new DijkstraFuzzy;
    }
    else
        dijFuzzy->clearAll();

    if (dijkstra == nullptr) {
        dijkstra = new Dijkstra();
      }
      else
          dijkstra->clearAll();

    if (rType == BACKUPROUTEKSH) {
        if (dijkstraks == nullptr)
            dijkstraks = new DijkstraKshortest(10); // maximum patsh
        else
            dijkstraks->cleanLinkArray();

    }

    if (rType == BACKUPROUTEKSHFUZZY) {
        if (dijkstraksFuzzy == nullptr)
            dijkstraksFuzzy = new DijkstraKshortestFuzzy(10); // maximum patsh
        else
            dijkstraksFuzzy->cleanLinkArray();

    }



    dijFuzzy->setRoot(getParentModule()->par("address"));
    dijkstra->setRoot(getParentModule()->par("address"));
    if (dijkstraks)
        dijkstraks->setRoot(getParentModule()->par("address"));
    if (dijkstraksFuzzy)
        dijkstraksFuzzy->setRoot(getParentModule()->par("address"));

    std::vector<std::string> nedTypes;
    nedTypes.push_back(getParentModule()->getNedTypeName());

    cTopology topo("topo");
    topo.extractByNedTypeName(nedTypes);

    Dijkstra dj;

    for (int i = 0; i < topo.getNumNodes(); i++) {
        cTopology::Node *node = topo.getNode(i);
        for (int j = 0; j < node->getNumOutLinks(); j++) {
            if (node->getLinkOut(j)->getLocalGate()->getTransmissionChannel()->getNominalDatarate() > maxCapacity)
                maxCapacity = node->getLinkOut(j)->getLocalGate()->getTransmissionChannel()->getNominalDatarate();
        }
    }


    for (int i = 0; i < topo.getNumNodes(); i++) {
        cTopology::Node *node = topo.getNode(i);
        int address = node->getModule()->par("address");

        destination.push_back(address);

        for (int j = 0; j < node->getNumOutLinks(); j++) {

            int addressAux = node->getLinkOut(j)->getRemoteNode()->getModule()->par("address");

            double minResidual = node->getLinkOut(j)->getLocalGate()->getTransmissionChannel()->getNominalDatarate();
            double meanResidual = node->getLinkOut(j)->getLocalGate()->getTransmissionChannel()->getNominalDatarate();
            double maxResidual = node->getLinkOut(j)->getLocalGate()->getTransmissionChannel()->getNominalDatarate();

            double alpha = node->getLinkOut(j)->getLocalGate()->getTransmissionChannel()->getNominalDatarate()/maxCapacity;

            if (residual) {
                if (minResidual < 0.0001)
                    minResidual = 1 / 0.0001;
                else
                    minResidual = 1 / minResidual;

                if (meanResidual < 0.0001)
                    meanResidual = 1 / 0.0001;
                else
                    meanResidual = 1 / meanResidual;

                if (maxResidual < 0.0001)
                    maxResidual = 1 / 0.0001;
                else
                    maxResidual = 1 / maxResidual;
            } else {
                // Usar funciones lineales  o hiperbólicas?

                if (useAlpha) {
                    minResidual = 1/alpha;
                    meanResidual = 1/alpha;
                    maxResidual = 1/alpha;
                }
                else {
                    minResidual = 1;
                    meanResidual = 1;
                    maxResidual = 1;
                }
            }
            dijFuzzy->addEdge(address, addressAux, minResidual, meanResidual, maxResidual);
            dijkstra->addEdge(address, addressAux, minResidual,1);
            dj.addEdge(address, addressAux, 1, 10000);
            if (dijkstraks)
                dijkstraks->addEdgeWs(address, addressAux,1,minResidual);
            if (dijkstraksFuzzy)
                dijkstraksFuzzy->addEdge(address, addressAux, minResidual, meanResidual, maxResidual);
        }
    }
    NodePairs links;
    dj.discoverAllPartitionedLinks(links);
}



void RoutingModule::procActualize(Actualize *pkt)
{
    // actualiza los estados para ejecutar disjtra.
    int nodeId = pkt->getSrcAddr();
    for (unsigned int i = 0; i < pkt->getLinkDataArraySize(); i++) {
        LinkData linkData = pkt->getLinkData(i);
        if (linkData.nominal == 0) {
            dijFuzzy->deleteEdge(nodeId, linkData.node);
            dijkstra->deleteEdge(nodeId, linkData.node);
            if (dijkstraks)
                dijkstraks->deleteEdge(nodeId, linkData.node);
            if (dijkstraksFuzzy)
                 dijkstraksFuzzy->deleteEdge(nodeId, linkData.node);
        }
        else {
            double minResidual = linkData.nominal - linkData.max;
            double meanResidual = linkData.nominal - linkData.mean;
            double maxResidual = linkData.nominal - linkData.min;
            double instResidual = linkData.nominal - linkData.actual;

            if (residual) {
                if (minResidual < 1e30)
                    minResidual = 1 / 1e30;
                else
                    minResidual = 1 / minResidual;

                if (meanResidual < 1e30)
                    meanResidual = 1 / 1e30;
                else
                    meanResidual = 1 / meanResidual;

                if (maxResidual < 1e30)
                    maxResidual = 1 / 1e30;
                else
                    maxResidual = 1 / maxResidual;
            }
            else {
                // Usar funciones lineales  o hiperbólicas?

                double overCost = 1;
                if (par("fuzzyCorregido").boolValue()) {
                    overCost = 10 * pow (20, - linkData.mean / linkData.nominal);
                }
                else if (!percentajesValues.empty()) {
                    double res = linkData.nominal - linkData.mean;
                    double percentaje = (res/linkData.nominal) * 100;
                    //double percentaje = (linkData.mean/linkData.nominal) * 100;
                    for (unsigned int i = 0; i < percentajesValues.size(); i++) {
                        if (percentaje < percentajesValues[i]) {
                            overCost = sanctionValues[i];
                            break;
                        }
                    }
                }
                minResidual = (linkData.min / linkData.nominal) * overCost;
                meanResidual = (linkData.mean / linkData.nominal) * overCost;
                maxResidual = (linkData.max / linkData.nominal) * overCost;
                instResidual = linkData.actual / linkData.nominal * overCost;


                if (useAlpha) {
                    double alpha = linkData.nominal /maxCapacity;
                    minResidual /= alpha;
                    meanResidual /= alpha;
                    maxResidual /= alpha;
                    instResidual /= alpha;
                }

            }

            if (minResidual == 0)
                throw cRuntimeError("Problems detected");
            dijFuzzy->addEdge(nodeId, linkData.node, minResidual, meanResidual, maxResidual);
            if (dijkstraksFuzzy)
                dijkstraksFuzzy->addEdge(nodeId, linkData.node, minResidual, meanResidual, maxResidual);
            if (par("instValue").boolValue()) {
                if (rType == SW || rType == WS)
                    dijkstra->addEdge(nodeId, linkData.node, 1, linkData.nominal - linkData.actual);
                else
                    dijkstra->addEdge(nodeId, linkData.node, instResidual, 1);
                if (dijkstraks)
                    dijkstraks->addEdgeWs(nodeId, linkData.node, 1, instResidual);

            }
            else {
                double minResidual = linkData.nominal - linkData.max;
                double meanResidual = linkData.nominal - linkData.mean;
                double maxResidual = linkData.nominal - linkData.min;
                DijkstraFuzzy::FuzzyCost costFuzzy(minResidual,meanResidual,maxResidual);
                if (rType == SW || rType == WS)
                    dijkstra->addEdge(nodeId, linkData.node, 1, meanResidual);
                else if (rType == SWFUZZY || rType == WSFUZZY)
                    dijkstra->addEdge(nodeId, linkData.node, 1, costFuzzy.exp());
                else
                    dijkstra->addEdge(nodeId, linkData.node, meanResidual, 1);
                if (dijkstraks)
                    dijkstraks->addEdgeWs(nodeId, linkData.node, 1 ,meanResidual);
            }
        }
    }
}

void RoutingModule::receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj, cObject *details)
{
    Enter_Method_Silent();
    if (actualizationSignal == signalID) {
        Actualize *pkt = dynamic_cast<Actualize *>(obj);
        if (pkt == nullptr || dijFuzzy == nullptr) {
            throw cRuntimeError("Sennal no esperada");
            return; //
        }
        procActualize(pkt);
        if (!nextAct->isSelfMessage())
            scheduleAt(simTime(),nextAct);
    }
}

void RoutingModule::receiveSignal(cComponent *source, simsignal_t signalID, bool change, cObject *details)
{
    Enter_Method_Silent();

    if (signalID != actualizationPortsSignal)
        return;
    // TODO:
}


void RoutingModule::initialize(int stage)
{
    if (stage == 0)
        return;

    residual = par("useHyperbolic").boolValue();

    if (strcmp(par("RoutingType").stringValue(),"HopByHop") ==0)
        rType = HOPBYHOP;
    else if (strcmp(par("RoutingType").stringValue(),"SourceRoutingFuzzy") ==0)
        rType = SOURCEROUTING;
    else if (strcmp(par("RoutingType").stringValue(),"Disjoint") ==0)
        rType = DISJOINT;
    else if (strcmp(par("RoutingType").stringValue(),"BackupRouting") ==0)
        rType = BACKUPROUTE;
    else if (strcmp(par("RoutingType").stringValue(),"SourceRouting") ==0)
        rType = SOURCEROUTINGNORMAL;
    else if (strcmp(par("RoutingType").stringValue(),"Sw") ==0)
        rType = SW;
    else if (strcmp(par("RoutingType").stringValue(),"Ws") ==0)
        rType = WS;
    else if (strcmp(par("RoutingType").stringValue(),"SwFuzzy") ==0)
        rType = SWFUZZY;
    else if (strcmp(par("RoutingType").stringValue(),"WsFuzzy") ==0)
        rType = WSFUZZY;
    else
        throw cRuntimeError("Routing type unknown");


    const char *levelsValues = par("levelsValues");
    cStringTokenizer tokenizerLevesl(levelsValues);
    const char *token;

    while ((token = tokenizerLevesl.nextToken()) != NULL)
    {
        double val = atof(token);
        percentajesValues.push_back(val);
    }

    const char *santionValues = par("santionValues");
    cStringTokenizer tokenizerSantionValues(santionValues);
    while ((token = tokenizerSantionValues.nextToken()) != NULL)
    {
        double val = atof(token);
        sanctionValues.push_back(val);
    }
    if (!percentajesValues.empty() && percentajesValues.size() != sanctionValues.size())
        throw cRuntimeError("Size of levelsValues and santionValues are different");

    useAlpha = par("useAlpha");

    nextAct = new cMessage("NewEvent");
    readTopo();


    cModule * mod = gate("out")->getPathEndGate()->getOwnerModule();
    forwarding = check_and_cast<IForwarding *> (mod);

    myAddress = forwarding->getAddress();

    dijkstra->setRoot(myAddress);
    dijkstra->run();

    // TODO: actualize forwarding
    for (auto elem : destination) {
        if (elem == myAddress)
            continue;
        int oldPort = forwarding->getRouting(elem);
        std::vector<NodeId> pathNode;
        dijkstra->getRoute(elem, pathNode);

        int newPort = forwarding->getNeighborConnectPort(pathNode[1]);
        if (oldPort != newPort) {
            forwarding->setRoute(elem,newPort);
        }
    }

    // register in the root module to receive the actualization of all nodes.
    cSimulation::getActiveSimulation()->getSystemModule()->subscribe(actualizationSignal,this);
    this->getParentModule()->subscribe(actualizationPortsSignal,this);
}

void RoutingModule::handleMessage(cMessage *msg)
{

    if (nextAct == msg) {
        if (actualizeForwarding) {
            dijkstra->setRoot(myAddress);
            dijkstra->run();

            for (auto elem : destination) {
                if (elem == myAddress)
                    continue;
                int oldPort = forwarding->getRouting(elem);
                std::vector<NodeId> pathNode;
                dijkstra->getRoute(elem, pathNode);

                int newPort = forwarding->getNeighborConnectPort(pathNode[1]);
                if (oldPort != newPort) {
                    forwarding->setRoute(elem, newPort);
                    ChangeRoutingTable changeRoutingTable;
                    changeRoutingTable.destination = elem;
                    changeRoutingTable.newPort = newPort;
                    changeRoutingTable.oldPort = oldPort;
                    emit(changeRoutingTableSignal, &changeRoutingTable);
                }
            }
        }
        return;
    }

    if (!msg->isPacket()) {
        delete msg;
        return;
    }

    Base *pkaux = dynamic_cast<Base *>(msg);
    if (pkaux == nullptr) {
        delete msg;
        return;
    }

    if (pkaux->getType() == ACTUALIZE) {
        Actualize *pk = dynamic_cast<Actualize *>(msg);
        procActualize(pk);
        delete msg;
        return;
    }
    else {
        delete msg;
    }

    if (actualizeForwarding) {
        dijkstra->setRoot(myAddress);
        dijkstra->run();

        for (auto elem : destination) {
            if (elem == myAddress)
                continue;
            int oldPort = forwarding->getRouting(elem);
            std::vector<NodeId> pathNode;
            dijkstra->getRoute(elem, pathNode);

            int newPort = forwarding->getNeighborConnectPort(pathNode[1]);
            if (oldPort != newPort) {
                forwarding->setRoute(elem, newPort);
                ChangeRoutingTable changeRoutingTable;
                changeRoutingTable.destination = elem;
                changeRoutingTable.newPort = newPort;
                changeRoutingTable.oldPort = oldPort;
                emit(changeRoutingTableSignal, &changeRoutingTable);
            }
        }
    }
    return;
}


