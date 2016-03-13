//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// Copyright (C) 1992-2008 Andras Varga
// Copyright (C) 2016 Alfonso Ariza
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//

#ifdef _MSC_VER
#pragma warning(disable:4786)
#endif

#include "FlowRouting.h"
#include "FailureEvent.h"

Define_Module(FlowRouting);

FlowRouting::~FlowRouting()
{
    cancelAndDelete(actualizeTimer);
    callInfomap.clear();
    ocupation.clear();
    flowOcupation.clear();
    nominalbw.clear();
}

void FlowRouting::initialize()
{
    eventSignal = registerSignal("EventSignal");
    getSimulation()->getSystemModule()->subscribe(eventSignal, this);

    myAddress = getParentModule()->par("address");

    dropSignal = registerSignal("drop");
    outputIfSignal = registerSignal("outputIf");

    //
    // Brute force approach -- every node does topology discovery on its own,
    // and finds routes to all other nodes independently, at the beginning
    // of the simulation. This could be improved: (1) central routing database,
    // (2) on-demand route calculation
    //
    cTopology *topo = new cTopology("topo");

    std::vector<std::string> nedTypes;
    nedTypes.push_back(getParentModule()->getNedTypeName());
    topo->extractByNedTypeName(nedTypes);
    EV << "cTopology found " << topo->getNumNodes() << " nodes\n";

    cTopology::Node *thisNode = topo->getNodeFor(getParentModule());

    for (int j = 0; j < thisNode->getNumOutLinks(); j++) {
        int address = thisNode->getLinkOut(j)->getRemoteNode()->getModule()->par("address");
        NeighborsPorts portsState;
        portsState.port = thisNode->getLinkOut(j)->getLocalGate()->getIndex();
        portsState.state = UP;
        neighbors[address] = portsState;
    }

    // find and store next hops
    for (int i = 0; i < topo->getNumNodes(); i++) {
        if (topo->getNode(i) == thisNode)
            continue; // skip ourselves
        topo->calculateUnweightedSingleShortestPathsTo(topo->getNode(i));

        if (thisNode->getNumPaths() == 0)
            continue; // not connected

        cGate *parentModuleGate = thisNode->getPath(0)->getLocalGate();
        int gateIndex = parentModuleGate->getIndex();
        int address = topo->getNode(i)->getModule()->par("address");
        rtable[address] = gateIndex;
        EV << "  towards address " << address << " gateIndex is " << gateIndex << endl;
    }

    cModule *node = this->getParentModule();
    ocupation.resize(this->gateSize("in"));
    flowOcupation.resize(this->gateSize("in"));
    nominalbw.resize(this->gateSize("in"));
    lastInfoOcupation.resize(this->gateSize("in"));
    lastInfoNominal.resize(this->gateSize("in"));
    portStatus.resize(this->gateSize("in"));;
    for (unsigned int i = 0; i < ocupation.size(); i++) {
        cChannel * channel = node->gate("port$o", i)->getTransmissionChannel();
        ocupation[i] = channel->getNominalDatarate();
        flowOcupation[i] = channel->getNominalDatarate();
        nominalbw[i] = channel->getNominalDatarate();
        portStatus[i] = UP;
    }
    delete topo;
    localOutSize = this->gateSize("localOut");

    actualizeTimer = new cMessage("actualize timer");
    scheduleAt(simTime(), actualizeTimer);
}

bool FlowRouting::actualize(Actualize *other)
{
    if (other) // check first if attach information
    {
        simtime_t now = simTime();
        if (SIMTIME_DBL(now - lastTimeActualize) < par("minimumTimeActualize").doubleValue())
            return false;
        cancelEvent(actualizeTimer);
    }

    Actualize *pkt = new Actualize("actualize");

    pkt->setSourceId(myAddress);
    pkt->setDestAddr(-1);
    pkt->setType(ACTUALIZE);
    pkt->setLinkDataArraySize(neighbors.size());
    pkt->setSequence(seqnum++);

    for (auto elem : neighbors) {
        LinkData auxdata;
        auxdata.node = elem.first;
        if (elem.second.state == UP) {
            auxdata.residual = ocupation[elem.second.port];
            auxdata.nominal = nominalbw[elem.second.port];
            lastInfoOcupation[elem.second.port] = ocupation[elem.second.port];
            lastInfoNominal[elem.second.port] = nominalbw[elem.second.port];
        }
        else if (elem.second.state == DOWN) {
            auxdata.residual = 0;
            auxdata.nominal = 0;
            lastInfoOcupation[elem.second.port] = 0;
            lastInfoNominal[elem.second.port] = 0;
        }
        pkt->setLinkData(elem.second.port, auxdata);
    }
    for (int i = 0; i < localOutSize; i++)
        send(pkt->dup(), "localOut", i);
    if (other)
        pkt->encapsulate(other);
    for (auto elem : neighbors)
        send(pkt->dup(), "out", elem.second.port);

    lastTimeActualize = simTime();
    scheduleAt(simTime() + par("actualizeState"), actualizeTimer);
    delete pkt;
    return true;
}

void FlowRouting::procBroadcast(Base *pkbase)
{
    // arrival gate index
    int gateIndex = -1;
    if (strcmp(pkbase->getArrivalGate()->getName(), "in") == 0)
        gateIndex = pkbase->getArrivalGate()->getIndex();
    // check sequence number

    int srcAddr = pkbase->getSrcAddr();
    std::list<Base *> packets;
    do {
        packets.push_back(pkbase);
        if (pkbase->getEncapsulatedPacket())
            pkbase = check_and_cast<Base *>(pkbase->decapsulate());
        else
            pkbase = nullptr;
    } while (pkbase);

    for (auto itAux = packets.begin(); itAux != packets.end();) {
        Base * pktAux = *(itAux);

        if (pktAux->getSrcAddr() == myAddress) {
            delete pktAux;
            itAux = packets.erase(itAux);
            continue;
        }
        auto it = sequenceTable.find(pktAux->getSrcAddr());
        if (it == sequenceTable.end()) {
            // add new
            sequenceTable[srcAddr] = pktAux->getSequence();
        }
        else {
            if (it->second >= pktAux->getSequence()) {
                // duplicate, delete it
                delete pktAux;
                itAux = packets.erase(itAux);
                continue;
            }
            else
                it->second = pktAux->getSequence();
        }
        ++itAux;
    }
    // actualize data
    if (packets.empty())
        return;
    for (auto &elem : packets) {
        if (elem->getType() == ACTUALIZE) {
            Actualize *pkt = check_and_cast<Actualize *>(elem);
            procActualize(pkt);
            for (int i = 0; i < localOutSize; i++)
                send(elem->dup(), "localOut", i);
        }
    }
    // send a copy to neighbors
    pkbase = packets.back();
    packets.pop_back();
    while (!packets.empty()) {
        packets.back()->encapsulate(pkbase);
        pkbase = packets.back();
        packets.pop_back();
    }

    if (!actualize(dynamic_cast<Actualize*>(pkbase))) // in case that actualize take the control the method will delete the packet
            {
        for (auto elem : neighbors) {
            if (gateIndex != elem.second.port && elem.second.state == UP)
                send(pkbase->dup(), "out", elem.second.port);
        }
        delete pkbase;
    }
}

void FlowRouting::procActualize(Actualize *pkt)
{

    if (dijkstra == nullptr)
        return;
    for (unsigned int i = 0; i < pkt->getLinkDataArraySize(); i++) {
        if (pkt->getLinkData(i).residual > 0)
            dijkstra->addEdge(pkt->getSourceId(), pkt->getLinkData(i).node, 1.0 / (double) pkt->getLinkData(i).residual,
                    0, 1000, 0);
        else
            dijkstra->deleteEdge(pkt->getSourceId(), pkt->getLinkData(i).node);
    }
    dijkstra->setRoot(myAddress);
    dijkstra->run();
    for (auto &elem : rtable) {
        std::vector<NodeId> pathNode;
        dijkstra->getRoute(elem.first, pathNode);
        if (pathNode.empty())
            elem.second = -1;
        else {
            auto it = neighbors.find(pathNode[0]);
            if (it == neighbors.end())
                throw cRuntimeError("Node not in neighbors table");
            elem.second = it->second.port;
        }
    }
}

// process link break and link restore events
void FlowRouting::processLinkEvents(cObject *obj)
{
    Event * event = dynamic_cast<Event *>(obj);

    if (event) {
        int neigAddr = -1;
        bool process = false;
        if (event->linkId.first == myAddress || event->linkId.second == -1) {
            // general failure
            process = true;
            neigAddr = -1;
        }
        else if (event->linkId.first == myAddress || event->linkId.second == myAddress) {
            process = true;
            // check if neighbor
            if (event->linkId.first == myAddress) {
                neigAddr = event->linkId.second;
                auto itNeig = neighbors.find(event->linkId.second);
                if (itNeig == neighbors.end())
                    throw cRuntimeError("Neighbor address not found check break event list");
            }
            else {
                neigAddr = event->linkId.first;
                auto itNeig = neighbors.find(event->linkId.first);
                if (itNeig == neighbors.end())
                    throw cRuntimeError("Neighbor address not found check break event list");
            }

        }
        else if (event->linkId.first != myAddress || event->linkId.second == -1) // node failure check if it is a neighbor
                {
            auto itNeig = neighbors.find(event->linkId.first);
            if (itNeig != neighbors.end()) {
                neigAddr = event->linkId.first;
                process = true;
            }
        }
        //
        if (process) {
            // failure the whole actual node
            if (neigAddr == -1) {
                // node failure or recovery
                if (event->type == NODE_FAILURE_EV || event->type == LINK_FAILURE_EV) {
                    for (auto & elem : neighbors) {
                        elem.second.state = DOWN;
                    }
                    for (unsigned int i = 0; i < ocupation.size(); i++) {
                        ocupation[i] = 0;
                        portStatus[i] = DOWN;
                        //lastInfoOcupation[i] = ocupation[i];
                        //lastInfoNominal[i] = nominalbw[i];
                    }
                    // es necesario terminar los flujos hacia arriba inmediatamente
                    for (auto & elem : callInfomap) {
                        // send messages to up layer.
                        if (elem.second.node1 == myAddress || elem.second.node2 == myAddress) {
                            // es necesario terminar los flujos hacia arriva inmediatamente
                             for (auto & elem2 : elem.second.outputFlows) {
                                 if (elem2.port == -1) {
                                     // enviar mensaje de terminación de fujo
                                     Packet *pkt = new Packet();
                                     pkt->setCallId(elem.first);
                                     pkt->setType(ENDFLOW);
                                     pkt->setFlowId(elem2.flowId);
                                     pkt->setSrcAddr(elem2.src);
                                     pkt->setDestAddr(myAddress);

                                     pkt->setDestinationId(elem.second.applicationId);
                                     auto it = sourceIdGate.find(elem.second.applicationId);
                                     if (it == sourceIdGate.end())
                                         throw cRuntimeError("Source id %i not registered", pkt->getDestinationId());
                                     send(pkt, "localOut", it->second);
                                 }
                             }
                            //
                            // send call release messages to application if it is necessary, the neighbors send the end messages to the other parts.

                            Packet *pkt = new Packet();
                            if (elem.second.node1 == myAddress) {
                                pkt->setSrcAddr(elem.second.node1);
                                pkt->setDestAddr(elem.second.node2);
                            }
                            else {
                                pkt->setSrcAddr(elem.second.node2);
                                pkt->setDestAddr(elem.second.node1);
                            }

                            pkt->setCallId(elem.first);
                            pkt->setType(RELEASE);
                            pkt->setSourceId(elem.second.applicationId);
                            // se envía hacia el destino
                            sendDelayed(pkt, par("breakDelay"),"localOut", elem.second.applicationId);
                        }
                    }
                    callInfomap.clear(); // clean call information
                }
                else if (event->type == NODE_RECOVERY_EV || event->type == LINK_RECOVERY_EV) {
                    for (auto & elem : neighbors) {
                        elem.second.state = UP;
                    }
                    for (unsigned int i = 0; i < ocupation.size(); i++) {
                        ocupation[i] = nominalbw[i];
                        flowOcupation[i] = nominalbw[i];
                        portStatus[i] = UP;
                    }
                    if (inmediateNotificationLink)
                        actualize(nullptr);
                }
                return;
            }
            // failure a link in the actual node
            if (event->type == NODE_FAILURE_EV || event->type == LINK_FAILURE_EV) {
                auto itNeig = neighbors.find(neigAddr);
                if (itNeig == neighbors.end())
                    throw cRuntimeError("Neighbor address not found check break event list");
                // search in the list the flows and active calls
                itNeig->second.state = DOWN;
                portStatus[itNeig->second.port] = DOWN;
                itNeig->second.failureTime = simTime();
                std::vector<std::pair<int, int> > breakCommunication;
                for (auto itCallInfo = callInfomap.begin(); itCallInfo != callInfomap.end();++itCallInfo) {
                    if (itCallInfo->second.port1 == itNeig->second.port
                            || itCallInfo->second.port2 == itNeig->second.port) {
                        //if (itCallInfo->second.port1 >= 0)
                        //    ocupation[itCallInfo->second.port1] += itCallInfo->second.reserve;

                        //if (itCallInfo->second.port2 >= 0)
                        //    ocupation[itCallInfo->second.port2] += itCallInfo->second.reserve;

                        while (!itCallInfo->second.outputFlows.empty())
                        {
                            FlowInfo flowinfo = itCallInfo->second.outputFlows.back();
                            itCallInfo->second.outputFlows.pop_back();
                            if (flowinfo.port != itNeig->second.port)
                            {
                                // free bandwidth
                                flowOcupation[flowinfo.port] += flowinfo.used;
                                // Send end flow
                                Packet *pkt = new Packet();
                                pkt->setType(ENDFLOW);
                                pkt->setFlowId(flowinfo.flowId);
                                pkt->setSrcAddr(flowinfo.src);
                                if (itCallInfo->second.node1 == flowinfo.src)
                                    pkt->setDestAddr(itCallInfo->second.node2);
                                else
                                    pkt->setDestAddr(itCallInfo->second.node1);
                                send(pkt,"out",flowinfo.port);
                            }
                        }

                        // send call release messages
                        Packet *pkt = new Packet();
                        int forwartPort;
                        // check the direction that it is necessary to send the packet
                        if (itNeig->second.port == itCallInfo->second.port2)
                        {
                            forwartPort = itCallInfo->second.port1;
                            pkt->setSrcAddr(itCallInfo->second.node1);
                            pkt->setDestAddr(itCallInfo->second.node2);
                        }
                        else if (itNeig->second.port == itCallInfo->second.port1)
                        {
                            forwartPort = itCallInfo->second.port2;
                            pkt->setSrcAddr(itCallInfo->second.node2);
                            pkt->setDestAddr(itCallInfo->second.node1);
                        }
                        else
                            throw cRuntimeError("Unknown port");

                        // prepare the release of the call with a delay
                        pkt->setCallId(itCallInfo->first);
                        pkt->setType(RELEASE);
                        pkt->setSourceId(itCallInfo->second.applicationId);

                        if (itCallInfo->second.node1 == myAddress && itCallInfo->second.node2 == myAddress) {
                            // se envía hacia el destino, se comprueba
                            send(pkt->dup(), "localOut", itCallInfo->second.applicationId);
                        }

                        pkt->setKind(forwartPort);
                        if (itCallInfo->second.node1 == myAddress && itCallInfo->second.node2 == myAddress)
                            scheduleAt(simTime(), pkt); // release immediately
                        else
                            scheduleAt(simTime() + par("breakRelease"), pkt); // release with a delay
                        itCallInfo->second.state = END;
                    }
                }
                if (inmediateNotificationLink)
                    actualize(nullptr);
            }
            else if (event->type == NODE_RECOVERY_EV || event->type == LINK_RECOVERY_EV) {
                //
                auto itNeig = neighbors.find(neigAddr);
                if (itNeig == neighbors.end())
                    throw cRuntimeError("Neighbor address not found check break event list");
                itNeig->second.state = UP;
                portStatus[itNeig->second.port] = UP;

                // TODO: Gestionar microcortes
                // al terminar las llamadas debería estar el ancho de banda disponible ya que no se gestionan microcortes
                if (ocupation[itNeig->second.port] != nominalbw[itNeig->second.port])
                    throw cRuntimeError("Restore link, ocupation %llu nominal %llu",ocupation[itNeig->second.port], nominalbw[itNeig->second.port]);
               // if (simTime() - itNeig->second.failureTime > par("breakRelease"))
               //     ocupation[itNeig->second.port] = nominalbw[itNeig->second.port];
                if (inmediateNotificationLink)
                    actualize(nullptr);
            }
        }
    }
}

void FlowRouting::procReserve(Packet *pk, int &portForward, int &sourceId)
{
    auto it = rtable.end();
    int port1 = -1;
    int port2 = -1;
    int destAddr = pk->getDestAddr();
    int srcAddr = pk->getSrcAddr();
    bool rejected = false;
    if (destAddr != myAddress) {
        // Internal routing table,
        if (pk->getRouteArraySize() == 0) {
            it = rtable.find(destAddr);
            if (it != rtable.end())
                port1 = (*it).second;
            // check port status
            if (port1 != -1 && portStatus[port1] == DOWN)
                rejected = true;
        }
        else {
            // Source routing case
            int next = -1;
            for (unsigned int i = 0; i < pk->getRouteArraySize(); i++) {
                if (pk->getRoute(i) == myAddress) {
                    if (i + 1 < pk->getRouteArraySize()) {
                        next = pk->getRoute(i + 1);
                        break;
                    }
                }
            }
            if (next == -1)
                throw cRuntimeError("Error in route, next node is not found");
            // find port to neighbor
            auto itNeig = neighbors.find(next);
            if (itNeig != neighbors.end())
                throw cRuntimeError("Neighbor node not found");
            port1 = itNeig->second.port;
            // check neighbor port status
            if (itNeig->second.state == DOWN)
                rejected = true;
        }
    }

    // check if it is possible reserve enough bandwidth in both directions
    if (srcAddr != myAddress) {
        port2 = pk->getArrivalGate()->getIndex();
    }

    if ((port1 != -1 && ocupation[port1] < pk->getReserve()) && (port2 != -1 && ocupation[port2] < pk->getReserve()))
        rejected = true;

    if (port1 == -1 && destAddr != myAddress)
        rejected = true;

    // check link status
    if (rejected) {
        // reject call
        pk->setType(REJECTED);
        int tempAddr = pk->getSrcAddr();
        pk->setSrcAddr(pk->getDestAddr());
        pk->setDestAddr(tempAddr);
        if (srcAddr == myAddress) {
            // send up
            send(pk, "localOut", pk->getArrivalGate()->getIndex());
        }
        else {
            // send Reject message to the sender node.
            send(pk, "out", pk->getArrivalGate()->getIndex());
        }
        return;
    }

    // enough resources, bandwidth reserved.
    if (port1 != -1)
        ocupation[port1] -= pk->getReserve();

    if (port2 != -1)
        ocupation[port2] -= pk->getReserve();

    CallInfo callInfo;
    callInfo.node1 = pk->getSrcAddr();
    callInfo.node2 = pk->getDestAddr();
    callInfo.port1 = port1;
    callInfo.port2 = port2;
    callInfo.reserve = pk->getReserve();
    if (srcAddr == myAddress)
        callInfo.applicationId = pk->getArrivalGate()->getIndex();
    else if (destAddr == myAddress)
        callInfo.applicationId = pk->getSourceId();
    else
        callInfo.applicationId = -1;
    callInfomap[pk->getCallId()] = callInfo;
    portForward = port1;
    sourceId = callInfo.applicationId;
}

void FlowRouting::checkPendingList()
{
    if (!pendingFlows.empty()) {
        for (auto it = pendingFlows.begin(); it != pendingFlows.end();) {
            //
            auto itCallInfoAux = callInfomap.find(it->callId);
            if (itCallInfoAux == callInfomap.end()) {
                it = pendingFlows.erase(it);
                continue;
            }
            // check if the flow is in the input but not in the output
            bool isInOutput = false;
            for (auto elem : itCallInfoAux->second.outputFlows) {
                if (elem.flowId == it->flowId) {
                    // flow in the input erase it from pending
                    isInOutput = true;
                    it = pendingFlows.erase(it);
                    break;
                }
            }
            if (isInOutput)
                continue;

            // check the infput list
            bool isInInput = false;
            for (auto elem : itCallInfoAux->second.inputFlows) {
                if (elem.flowId == it->flowId) {
                    isInInput = true;
                    break;
                }
            }
            if (!isInInput) {
                it = pendingFlows.erase(it);
                continue;
            }


            if (it->port != -1 && flowOcupation[it->port] > it->used && portStatus[it->port] == UP) {

                flowOcupation[it->used] -= it->used;
                itCallInfoAux->second.outputFlows.push_back(*it);
                // send start flow to the next node
                Packet *pkStartFlow = new Packet();
                if (itCallInfoAux->second.node1 == it->src) {
                    pkStartFlow->setSrcAddr(itCallInfoAux->second.node1);
                    pkStartFlow->setDestAddr(itCallInfoAux->second.node2);
                }
                else {
                    pkStartFlow->setSrcAddr(itCallInfoAux->second.node2);
                    pkStartFlow->setDestAddr(itCallInfoAux->second.node1);
                }
                pkStartFlow->setCallId(itCallInfoAux->first);
                pkStartFlow->setFlowId(it->flowId);
                pkStartFlow->setType(STARTFLOW);
                pkStartFlow->setReserve(it->used);
                send(pkStartFlow, "out", it->port);
                it = pendingFlows.erase(it);
            }
            else
                ++it;
        }
    }
}

void FlowRouting::handleMessage(cMessage *msg)
{

    if (actualizeTimer == msg) {
        actualize();
        return;
    }

    if (dynamic_cast<RegisterMsg *>(msg)) {
        RegisterMsg * msgref = dynamic_cast<RegisterMsg *>(msg);
        //check if exist
        auto it = sourceIdGate.find(msgref->getSourceId());
        if (it != sourceIdGate.end())
            throw cRuntimeError("Source id %i exist", msgref->getSourceId());
        sourceIdGate[msgref->getSourceId()] = msg->getArrivalGate()->getIndex();
        delete msg;
        return;
    }

    Base *pkbase = check_and_cast<Base *>(msg);

    if (strcmp(pkbase->getArrivalGate()->getName(), "localIn") == 0)
        pkbase->setSrcAddr(myAddress);

    int destAddr = pkbase->getDestAddr();
    int srcAddr = pkbase->getSrcAddr();

    // broadcast packets pre-processing
    if (destAddr == -1) {
        procBroadcast(pkbase);
        return;
    }

    Packet *pk = check_and_cast<Packet *>(msg);
    // integrity check, pre-processing unicast packets
    auto itCallInfo = callInfomap.find(pk->getCallId());

    // actions
    if (pk->getType() == RESERVE) {
        if (itCallInfo != callInfomap.end())
            throw cRuntimeError("Error in callId parameter, call exist already : RESERVE");

    }
    else {
        if (itCallInfo == callInfomap.end()) {
            if (pk->getType() == STARTFLOW || pk->getType() == ENDFLOW) {
                // discard
                delete msg;
                return;
            }
            else if  (pk->getType() == RELEASE) {
                // discard
                delete msg;
                return;
            }
            else
                throw cRuntimeError("Error in callId parameter, call doesn't exist ");
        }
        else if (pk->isSelfMessage() && pk->getType() == RELEASE) {
            // TODO : Micro cortes, comprobar si está el enlace activo.

            if (itCallInfo->second.port1 >= 0)
            {
                // check port status

                // TODO : Microcortes
/*                NeighborsPorts * neig = nullptr;
                for (auto &elem : neighbors) {
                    if(elem.second.port == itCallInfo->second.port1) {
                        neig = &elem.second;
                        if (elem.second.state == UP) // micro corte, no debería enviarse,
                            ocupation[itCallInfo->second.port1] += itCallInfo->second.reserve;
                        break;
                    }
                }
                if (neig && neig->state == UP)
                    ocupation[itCallInfo->second.port1] += itCallInfo->second.reserve;
                else
                    ocupation[itCallInfo->second.port1] += itCallInfo->second.reserve;*/
                //
                ocupation[itCallInfo->second.port1] += itCallInfo->second.reserve;
            }

            if (itCallInfo->second.port2 >= 0)
            {
                  // check port status
                // TODO : Microcortes
/*
                NeighborsPorts * neig = nullptr;
                for (auto &elem : neighbors) {
                    if(elem.second.port == itCallInfo->second.port1) {
                        neig = &elem.second;
                        if (elem.second.state == UP) // micro corte, no debería enviarse,
                            ocupation[itCallInfo->second.port1] += itCallInfo->second.reserve;
                        break;
                    }
                }
                if (neig && neig->state == UP)
                    ocupation[itCallInfo->second.port2] += itCallInfo->second.reserve;
                else
                    ocupation[itCallInfo->second.port2] += itCallInfo->second.reserve;*/
                  ocupation[itCallInfo->second.port2] += itCallInfo->second.reserve;
            }
            // send packet to next node
            send(pk,"out",pk->getKind());


            // check that flows are empty
            if (!itCallInfo->second.outputFlows.empty())
                throw cRuntimeError("flows list is not empty");
            callInfomap.erase(itCallInfo);
            checkPendingList();
            delete pk;
            return;
        }
    }

    int portForward = -1;
    int portInput = -1;
    int sourceId = -1;

    // processing packets

    if (pk->getType() == RESERVE) {
        procReserve(pk, portForward, sourceId);
    }
    else {
        if (srcAddr == myAddress) {
            if (itCallInfo->second.port1 != -1)
                portForward = itCallInfo->second.port1;
            else
                portForward = itCallInfo->second.port2;
        }
        else if (destAddr != myAddress) {
            if (pk->getArrivalGate()->getIndex() == itCallInfo->second.port1)
                portForward = itCallInfo->second.port2;
            else
                portForward = itCallInfo->second.port1;

        }

        if (srcAddr != myAddress)
            portInput = pk->getArrivalGate()->getIndex();

        sourceId = itCallInfo->second.applicationId;
        // check that the forward port is correctly set
        if (portForward == -1 && destAddr != myAddress)
            throw cRuntimeError("Error in forward port identification");

        if (pk->getType() == RELEASE || pk->getType() == REJECTED) // reject and release free resources.
        {
            if (itCallInfo->second.port1 >= 0)
                ocupation[itCallInfo->second.port1] += itCallInfo->second.reserve;

            if (itCallInfo->second.port2 >= 0)
                ocupation[itCallInfo->second.port2] += itCallInfo->second.reserve;

            // It is necessary to erase all flows of the same call
            // search, and delete flows
            for (auto elem : itCallInfo->second.outputFlows) {
                flowOcupation[elem.port] += elem.used;
            }
            if (!pendingFlows.empty()) {
                for (auto it = pendingFlows.begin(); it != pendingFlows.end();) {
                    //
                    if (it->callId == itCallInfo->first) {
                        it = pendingFlows.erase(it);
                    }
                    else
                        ++it;
                }
            }
            callInfomap.erase(itCallInfo);
        }
        else if (pk->getType() == ACEPTED) {
            itCallInfo->second.state = CALLUP;
        }
        else if (pk->getType() == STARTFLOW) {
            // flow
            if (itCallInfo->second.state != CALLUP) {            // call not established yet, delete flow
                delete pk;
                return;
            }

            FlowInfo flowInfo;
            flowInfo.flowId = pk->getFlowId();
            flowInfo.src = pk->getSrcAddr();
            flowInfo.callId = pk->getCallId();
            flowInfo.used = pk->getReserve();
            flowInfo.port = portForward;
            for (auto elem : itCallInfo->second.outputFlows) {
                if (elem.flowId == flowInfo.flowId)
                    throw cRuntimeError("Error Flow id already reserved in the output flows");;
            }

            for (auto elem : itCallInfo->second.inputFlows) {
                if (elem.flowId == flowInfo.flowId)
                    throw cRuntimeError("Error Flow id already reserved in the input flows");;
            }

            // register the flow in the input list
            FlowInfo flowInfoInput =  flowInfo;
            flowInfo.port = portInput;
            itCallInfo->second.inputFlows.push_back(flowInfoInput);

            if (portForward != -1 && portStatus[portForward] == UP) {
                if (flowOcupation[portForward] > pk->getReserve()) {
                    flowOcupation[portForward] -= pk->getReserve();
                    itCallInfo->second.outputFlows.push_back(flowInfo);
                }
                else // flow lost, include in pending flows.
                {
                    if (portForward != -1 )
                        pendingFlows.push_back(flowInfo);
                    delete pk;
                    return;
                }
            }
        }
        else if (pk->getType() == ENDFLOW) {
            FlowInfo flowInfo;
            flowInfo.flowId = pk->getFlowId();
            flowInfo.src = pk->getSrcAddr();
            flowInfo.callId = pk->getCallId();

            for (auto it = itCallInfo->second.inputFlows.begin(); it != itCallInfo->second.inputFlows.end(); ++it) {
                if (it->flowId == pk->getFlowId()) {
                    itCallInfo->second.inputFlows.erase(it);
                    break;
                }
            }

            auto it = itCallInfo->second.outputFlows.begin();

            while (it != itCallInfo->second.outputFlows.end()) {
                if (it->flowId == pk->getFlowId())
                    break;
                ++it;
            }

            if (it == itCallInfo->second.outputFlows.end()) {
                // It has been impossible to send the start flow message to the next hop,  delete and return.
                for (auto it = pendingFlows.begin(); it != pendingFlows.end(); ++it) {
                    if (it->flowId == pk->getFlowId() && it->callId == itCallInfo->first) {
                        pendingFlows.erase(it);
                        break;
                    }
                }
                delete pk;
                return;
                // throw cRuntimeError("Error Flow id not found reserved");
            }

            if (it->port != -1)
                flowOcupation[it->port] += it->used;
            itCallInfo->second.outputFlows.erase(it);
        }
    }

    // check if event is a release type event.
    bool releaseResources = pk->getType() ==  RELEASE || pk->getType() == REJECTED ||  pk->getType() == ENDFLOW ; // end a call

    if (destAddr == myAddress) {
        EV << "local delivery of packet " << pk->getName() << endl;

        if (sourceId == -1)
            send(pk, "localOut", 0);
        else {
            auto it = sourceIdGate.find(pk->getDestinationId());
            if (it == sourceIdGate.end())
                throw cRuntimeError("Source id %i not registered", pk->getDestinationId());
            send(pk, "localOut", it->second);
        }
        emit(outputIfSignal, -1); // -1: local
    }
    else {
        if (pk->getDestAddr() != -1) {
            EV << "forwarding packet " << pk->getName() << " on gate index " << portForward << endl;
            pk->setHopCount(pk->getHopCount() + 1);
            emit(outputIfSignal, portForward);
            send(pk, "out", portForward);
        }
        else
            delete pk;
    }
    if (releaseResources)
        checkPendingList();
}

void FlowRouting::receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj, cObject *details)
{
    Enter_Method_Silent();
    if (signalID == eventSignal) {
        Event * event = dynamic_cast<Event *>(obj);
        if (event)
            processLinkEvents(event);

    }
}
