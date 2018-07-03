//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// Copyright (C) 1992-2008 Andras Varga
// Copyright (C) 2016 Alfonso Ariza
// Copyright (C) 2017 Alfonso Ariza
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//

#ifdef _MSC_VER
#pragma warning(disable:4786)
#endif

#include "FlowForwarding.h"
#include "FailureEvent.h"

Define_Module(FlowForwarding);

simsignal_t FlowForwarding::actualizationSignal = registerSignal("actualizationSignal");
simsignal_t FlowForwarding::eventSignal = registerSignal("EventSignal");
simsignal_t FlowForwarding::actualizationPortsSignal = registerSignal("actualizationPortsSignal");
simsignal_t FlowForwarding::changeRoutingTableSignal = registerSignal("changeRoutingTableSignal");



unsigned int FlowForwarding::getNumPorts()
{
    return portDataArray.size();
}

void FlowForwarding::getPorts(std::vector<PortData> &p)
{
    p = portDataArray;
}

PortData FlowForwarding::getPort(const int &i)
{
    if (i <0 || i >=  (int)portDataArray.size())
        throw cRuntimeError("Port id is out of the array size");
    return portDataArray[i];
}

int FlowForwarding::getPortNeighbor(const int &i)
{
    if (i <0 || i >=  (int)portDataArray.size())
        throw cRuntimeError("Port id is out of the array size");
    return portDataArray[i].neighbor;

}

int FlowForwarding::getNeighborConnectPort(const int &i) const
{
    auto it = neighbors.find(i);
    if (it != neighbors.end())
        return it->second.port;
    return -1;
}

void FlowForwarding::getRoutingTable(std::map<int, int> &t)
{
    t = rtable;
}

int FlowForwarding::getRouting(const int &a)
{
    auto it = rtable.find(a);
    if (it == rtable.end())
        return -1;
    return it->second;
}

int FlowForwarding::getAddress()
{
    return myAddress;
}

void FlowForwarding::setRoute(const int & a,const int & p)
{
    auto it = rtable.find(a);
    if (it != rtable.end() && p == -1) {
        rtable.erase(it);
        return;
    }

    if (it == rtable.end())
        rtable.insert(std::make_pair(a,p));
    else
        it->second = p;
}



// TODO: Mecanismos de reserva y comparticion cuando los enlaces estan llenos, el ancho de banda se reparte y se puede cambiar el ancho de banda en funcion del reparto.

void FlowForwarding::recordOccupation(PortData &port, const ChangeBw &val)
{
    if (port.accmin > val.value)
        port.accmin = val.value;
    if (port.accmax < val.value)
        port.accmax = val.value;
    if (val.value > port.nominalbw)
        throw cRuntimeError("Incorrect value");
    double auxVal = port.lastC.value * SIMTIME_DBL(simTime() - port.lastC.instant);
    port.accmean +=  auxVal;
    port.varSamples.push_back(auxVal);
    port.lastC = val;
}


FlowForwarding::~FlowForwarding()
{
    cancelAndDelete(actualizeTimer);
    callInfomap.clear();
    portDataArray.clear();
}

void FlowForwarding::computeUsedBw()
{
    simtime_t interval = simTime() - computationInterval;
    if (interval == simtime_t::ZERO) {

        for (auto &elem : portDataArray) {
            elem.mean = 0;
            elem.min = 0;
            elem.max = 0;
            elem.accmax = 0;
            elem.accmin = 1e300;
            elem.accmean = 0;
            elem.varSamples.clear();
            elem.lastC.instant = simTime();
        }
        emit(actualizationPortsSignal,true);
        return;
    }

    for (auto &elem : portDataArray) {
#if 0
        simtime_t previous = computationInterval;
        double total = 0;
        double min = elem.nominalbw;
        double max = 0;
        if (elem.changeRegister.size() == 1) {
            simtime_t time = simTime() - elem.changeRegister.front().instant;
            double valP = elem.changeRegister.front().value * (time.dbl()/interval.dbl());
            if (max < elem.changeRegister.front().value)
                max = elem.changeRegister.front().value;
            if (min > elem.changeRegister.front().value)
                min = elem.changeRegister.front().value;
            total += valP;
        }
        else {
            if (!elem.changeRegister.empty()) {
                for (unsigned int i = 0; i < elem.changeRegister.size(); i++) {
                    simtime_t time;
                    if (i != elem.changeRegister.size()-1)
                        time = elem.changeRegister[i + 1].instant - elem.changeRegister[i].instant;
                    else
                        time = simTime() - elem.changeRegister[i].instant;

                    uint64_t val = elem.changeRegister[i].value;
                    double valP = val * (time.dbl() / interval.dbl());
                    total += valP;
                    if (max < elem.changeRegister[i].value)
                        max = elem.changeRegister[i].value;
                    if (min > elem.changeRegister[i].value)
                        min = elem.changeRegister[i].value;
                }
            }
        }

        if (!elem.changeRegister.empty()) {
            auto elem2 = elem.changeRegister.back();
            elem2.instant = simTime();
            elem.changeRegister.clear();
            elem.changeRegister.push_back(elem2);
        }
        elem.mean = total;
        elem.min = min;
        elem.max = max;
#endif
        double timeVal = SIMTIME_DBL((simTime() - elem.lastC.instant));
        double val = elem.lastC.value * timeVal;
        elem.accmean +=  val;
        elem.varSamples.push_back(val);
        double inter = interval.dbl();
        elem.mean = elem.accmean/inter;
        if (elem.lastC.value < elem.accmin)
            elem.accmin = elem.lastC.value;
        if (elem.lastC.value > elem.accmax)
            elem.accmax = elem.lastC.value;

        elem.min = elem.accmin;
        elem.max = elem.accmax;
        if (elem.min  > elem.nominalbw)
            throw cRuntimeError("Computation min erroneous");
        if (elem.mean  > elem.nominalbw)
            throw cRuntimeError("Computation mean erroneous");
        if (elem.max  > elem.nominalbw)
            throw cRuntimeError("Computation max erroneous");

        for (auto elem2:elem.varSamples) {
            double auxVal = (elem2 - elem.accmean)/interval.dbl();
            elem.var = (auxVal * auxVal);
        }
        if (elem.varSamples.size() > 1)
            elem.var /= (elem.varSamples.size()-1);
        else
            elem.var = 0;
        elem.accmean = 0;
        elem.accmax = 0;
        elem.accmin = 1e300;
        elem.varSamples.clear();
        elem.lastC.instant = simTime();
    }
    computationInterval = simTime();
    emit(actualizationPortsSignal,true);
    // scheduleAt(simTime()+computationInterval,computeBwTimer);
}

void FlowForwarding::initialize()
{
    getSimulation()->getSystemModule()->subscribe(eventSignal, this);
    this->getParentModule()->subscribe(changeRoutingTableSignal, this);

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
    portDataArray.resize(this->gateSize("in"));

    for (unsigned int i = 0; i < portDataArray.size(); i++) {
        cChannel * channel = node->gate("port$o", i)->getTransmissionChannel();
        portDataArray[i].occupation = channel->getNominalDatarate();
        portDataArray[i].flowOccupation = channel->getNominalDatarate();
        portDataArray[i].nominalbw = channel->getNominalDatarate();

        portDataArray[i].max = portDataArray[i].occupation;
        portDataArray[i].min = portDataArray[i].occupation;
        portDataArray[i].mean = portDataArray[i].occupation;

        portDataArray[i].portStatus = UP;
        portDataArray[i].overload = false;
        ChangeBw val;
        val.instant = simTime();
        val.value = portDataArray[i].flowOccupation;
        recordOccupation(portDataArray[i], val);
        for (auto elem: neighbors) {
            if (elem.second.port == (int)i) {
                portDataArray[i].neighbor = elem.first;
                break;
            }
        }
        if (portDataArray[i].neighbor == -1)
            throw cRuntimeError("Neighbor not found");
    }

    actualizePercentaje();
    delete topo;
    localOutSize = this->gateSize("localOut");

    const char *flowClass = par("flowClass").stringValue();
    if (strcmp(flowClass, "") != 0) {
        if ((strcmp(flowClass, "Discard") != 0) && (strcmp(flowClass, "StoreAndForward") != 0))
            flowDist = check_and_cast<BaseFlowDistribution*>(createOne(flowClass));

        if (strcmp(flowClass, "Discard") == 0)
            flowAdmisionMode = DISCARD;
        if (strcmp(flowClass, "StoreAndForward") == 0)
            flowAdmisionMode = STOREANDFORWARD;
        if (strcmp(flowClass, "FiniteQueue") == 0)
            flowAdmisionMode = FINITEQUEUE;
    }
    //flowAdmisionMode = DISCARD;

    if (par("packetMode"))
        simulationMode = PACKETMODE;

    cModule *mod = gate("toRouting")->getPathEndGate()->getOwnerModule();
    if (mod) {
        routingModule = check_and_cast<IRouting *>(mod);
    }

    actualizeTimer = new cMessage("actualize timer");
    //computeBwTimer = new cMessage("actualize bw timer");
    //scheduleAt(simTime()+computationInterval,computeBwTimer);
    computationInterval = simTime();
    scheduleAt(simTime(), actualizeTimer);
}

bool FlowForwarding::actualize(Actualize *other)
{
    if (par("actPercentage")) {
        return actualizePercentaje();
    }

    if (other) { // check first if attach information
        simtime_t now = simTime();
        if (SIMTIME_DBL(now - lastTimeActualize) < par("minimumTimeActualize").doubleValue())
            return false;
    }

    if (actualizeTimer->isScheduled())
        cancelEvent(actualizeTimer);

    char pkname[40];
    sprintf(pkname, "Actualize-%d", myAddress);
    Actualize *pkt = new Actualize(pkname);

    pkt->setSrcAddr(myAddress);
    pkt->setSourceId(-1);
    pkt->setDestAddr(-1);
    pkt->setType(ACTUALIZE);
    pkt->setLinkDataArraySize(neighbors.size());
    pkt->setSequence(seqnum++);

    for (auto elem : neighbors) {
        LinkData auxdata;
        auxdata.node = elem.first;
        if (portDataArray[elem.second.port].portStatus == UP) {
            auxdata.residual = portDataArray[elem.second.port].occupation;
            auxdata.nominal = portDataArray[elem.second.port].nominalbw;
            auxdata.actual = portDataArray[elem.second.port].flowOccupation;
            if (simTime() == simtime_t::ZERO)
            {
                auxdata.max = portDataArray[elem.second.port].occupation;
                auxdata.min = portDataArray[elem.second.port].occupation;
                auxdata.mean = portDataArray[elem.second.port].occupation;
            }
            else {
                auxdata.max = portDataArray[elem.second.port].max;
                auxdata.min = portDataArray[elem.second.port].min;
                auxdata.mean = portDataArray[elem.second.port].mean;
            }
            portDataArray[elem.second.port].lastInfoOccupation = portDataArray[elem.second.port].occupation;
            portDataArray[elem.second.port].lastInfoNominal = portDataArray[elem.second.port].nominalbw;
        }
        else if (portDataArray[elem.second.port].portStatus  == DOWN) {
            auxdata.residual = 0;
            auxdata.nominal = 0;
            if (portDataArray[elem.second.port].lastInfoNominal !=0) {
                portDataArray[elem.second.port].lastInfoOccupation = 0;
                portDataArray[elem.second.port].lastInfoNominal = 0;
            }
        }
        pkt->setLinkData(elem.second.port, auxdata);
    }
    if (par("actualizeWithSignals"))
        emit(actualizationSignal,pkt);
    else {
        for (int i = 0; i < localOutSize; i++)
            send(pkt->dup(), "localOut", i);
        if (other)
            pkt->encapsulate(other);
        for (auto elem : neighbors)
            send(pkt->dup(), "out", elem.second.port);
    }

    lastTimeActualize = simTime();
    scheduleAt(simTime() + par("actualizeState"), actualizeTimer);
    delete pkt;
    return true;
}

bool FlowForwarding::actualizePercentaje()
{
    if (!par("actPercentage"))
        return false;

    if (SIMTIME_DBL(simTime() - lastTimeActualize) < par("minimumTimeActualize").doubleValue())
            return false;

    std::vector<LinkData> statedata;
    for (auto elem : neighbors) {
        LinkData auxdata;
        auxdata.node = elem.first;
        if (elem.second.state == UP) {
            if ((fabs((double)portDataArray[elem.second.port].lastInfoOccupation - (double)portDataArray[elem.second.port].occupation)/(double)portDataArray[elem.second.port].occupation)>par("percentage").doubleValue()) {
                auxdata.residual = portDataArray[elem.second.port].occupation;
                auxdata.nominal = portDataArray[elem.second.port].nominalbw;
                portDataArray[elem.second.port].lastInfoOccupation = portDataArray[elem.second.port].occupation;
                portDataArray[elem.second.port].lastInfoNominal = portDataArray[elem.second.port].nominalbw;
                statedata.push_back(auxdata);
            }
        }
        else if (elem.second.state == DOWN) {
            auxdata.residual = 0;
            auxdata.nominal = 0;
            if (portDataArray[elem.second.port].lastInfoNominal !=0) {
                portDataArray[elem.second.port].lastInfoOccupation = 0;
                portDataArray[elem.second.port].lastInfoNominal = 0;
                statedata.push_back(auxdata);
            }
        }
    }


    if (statedata.empty())
        return false;

    char pkname[40];
    sprintf(pkname, "Actualize-%d", myAddress);
    Actualize *pkt = new Actualize(pkname);

    pkt->setSourceId(myAddress);
    pkt->setDestAddr(-1);
    pkt->setType(ACTUALIZE);
    pkt->setLinkDataArraySize(statedata.size());
    pkt->setSequence(seqnum++);

    for (unsigned int i = 0; i < statedata.size(); i++) {
        pkt->setLinkData(i, statedata[i]);
    }

    for (int i = 0; i < localOutSize; i++)
        send(pkt->dup(), "localOut", i);

    for (auto elem : neighbors)
        send(pkt->dup(), "out", elem.second.port);

    lastTimeActualize = simTime();
    delete pkt;
    return true;
}

void FlowForwarding::getListFlowsToModifyStartFlow(const int &port,  std::vector<FlowInfo *> &listFlows,  std::vector<FlowInfo *> &listFlowsInput)
{
    for (auto it = callInfomap.begin(); it != callInfomap.end(); ++it) {
        if (it->second.port1 == port || it->second.port2 == port) {
            for (unsigned int i = 0; i < it->second.outputFlows.size(); i++) {
                if (it->second.outputFlows[i].port == port) {
                    listFlows.push_back(&(it->second.outputFlows[i]));
                }
                auto itAux = std::find(it->second.inputFlows.begin(), it->second.inputFlows.end(),
                        it->second.outputFlows[i]);
                if (itAux == it->second.inputFlows.end())
                    throw cRuntimeError("Flow in output list but not in input list of the call");
                listFlowsInput.push_back(&(*itAux));
            }
        }
    }

    for (auto it = outputFlows.begin(); it != outputFlows.end();++it) {
        if (it->second.port== port ) {
            listFlows.push_back(&(it->second));
            auto itAux = inputFlows.find(it->first);
            if (itAux == inputFlows.end())
                throw cRuntimeError("Flow in output list but not in input list, free flow");
            listFlowsInput.push_back(&(itAux->second));
        }
    }
}

void FlowForwarding::procBroadcast(Base *pkbase)
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
            if (routingModule)
                send(pkt->dup(),"toRouting");
        }
        else {
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

    Actualize* actPk = dynamic_cast<Actualize*>(pkbase);
    if (actPk == nullptr || actPk != nullptr )
        actualize(actPk);

    for (auto elem : neighbors) {
        if (gateIndex != elem.second.port && elem.second.state == UP)
            send(pkbase->dup(), "out", elem.second.port);
    }
    delete pkbase;
}

bool FlowForwarding::sendChangeFlow(FlowInfo &flow, const int &portForward)
{
 // check if exist in the imput list

    if (flow.identify.callId() > 0)
        return false;

    auto it = inputFlows.find(flow.identify);
    if (it == inputFlows.end())
        return false;
// env�a mensaje de cambio de ruta en el flujo.
    Packet * pkt = new Packet();
    pkt->setSrcAddr(flow.identify.src());
    pkt->setCallId(flow.identify.callId());
    pkt->setFlowId(flow.identify.flowId());
    pkt->setSourceId(flow.identify.srcId());
    pkt->setDestAddr(flow.destId);
    pkt->setReserve(flow.used);
    pkt->setType(CROUTEFLOWEND);

    if (hasGUI()) {
        char pkname[100];
        sprintf(pkname, "CROUTEFLOWENDL3-%d-to-%d-Call id#%llud-flow-%llud-dest Id %i", pkt->getSrcAddr(),
                pkt->getDestAddr(), pkt->getCallId(), pkt->getFlowId(), pkt->getDestinationId());
        pkt->setName(pkname);
    }
    send(pkt, "out", flow.port);
    // now check the bandwidth and if there is enough bandwidth in the new port
    uint64_t bw = it->second.used;
    // control admission
    // consume bandwidth
    auto itCallInfo = callInfomap.end();
    if (flow.identify.callId() > 0)
        itCallInfo = callInfomap.find(flow.identify.callId());

    it->second.port = portForward;
    // free the bandwith in the old port
    auto itF = outputFlows.find(flow.identify);

    FlowInfo *outflowPtr = nullptr;
    if (itF != outputFlows.end()) {
        portDataArray[itF->second.port].flowOccupation += itF->second.used;
        itF->second.port = portForward;
        outflowPtr = &(itF->second);
        ChangeBw val;
        val.instant = simTime();
        val.value = portDataArray[itF->second.port].flowOccupation;
        recordOccupation(portDataArray[itF->second.port], val);
    }
    flow.port = portForward;

    if (portForward != -1) {
        if (portDataArray[portForward].flowOccupation > pkt->getReserve()) {
            outputFlows[flow.identify] = flow;
        }
        else {
            if (!flodAdmision(bw, outflowPtr, &(it->second), portForward, it->second.portInput, CROUTEFLOWSTART)) {
                return false;
            }
        }
    }
    // send the new
    return true;
}


// process link break and link restore events
void FlowForwarding::processLinkEvents(cObject *obj)
{
    Event * event = dynamic_cast<Event *>(obj);

    if (event) {
        int neigAddr = -1;
        bool process = false;
        bool endCalls = false;
        if (event->linkId.first == myAddress && event->linkId.second == -1) {
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
                endCalls = true;
                neigAddr = event->linkId.first;
                auto itNeig = neighbors.find(event->linkId.first);
                if (itNeig == neighbors.end())
                    throw cRuntimeError("Neighbor address not found check break event list");
            }
        }
        else if (event->linkId.first != myAddress && event->linkId.second == -1) {// node failure check if it is a neighbor
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
                    for (unsigned int i = 0; i < portDataArray.size(); i++) {
                        portDataArray[i].occupation = 0;
                        portDataArray[i].portStatus = DOWN;
                        //lastInfoOccupation[i] = occupation[i];
                        //lastInfoNominal[i] = nominalbw[i];
                    }
                    // es necesario terminar los flujos hacia arriba inmediatamente
                    for (auto & elem : callInfomap) {
                        // send messages to up layer.
                        if (elem.second.node1 == myAddress || elem.second.node2 == myAddress) {
                            // es necesario terminar los flujos hacia arriva inmediatamente
                            for (auto & elem2 : elem.second.inputFlows) {
                                if (elem2.identify.src() != myAddress) { // flow received from other node
                                    // enviar mensaje de terminación de flujo
                                    Packet *pkt = new Packet();
                                    pkt->setCallId(elem.first);
                                    pkt->setType(ENDFLOW);
                                    pkt->setFlowId(elem2.identify.flowId());
                                    pkt->setSrcAddr(elem2.identify.src());
                                    pkt->setSourceId(elem2.identify.srcId());
                                    pkt->setDestAddr(myAddress);

                                    if (elem.second.node1 == myAddress)
                                        pkt->setDestinationId(elem.second.applicationId1);
                                    else
                                        pkt->setDestinationId(elem.second.applicationId2);

                                    auto it = sourceIdGate.find(pkt->getDestinationId());
                                    if (it == sourceIdGate.end())
                                        throw cRuntimeError("Source id %i not registered", pkt->getDestinationId());

                                    if (hasGUI()) {
                                        char pkname[100];
                                        sprintf(pkname, "EndFlowL3-%d-to-%d-Call id#%llud-flow-%llud-dest Id %i", pkt->getSrcAddr(),
                                                pkt->getDestAddr(), pkt->getCallId(), pkt->getFlowId(), pkt->getDestinationId());
                                        pkt->setName(pkname);
                                    }
                                    send(pkt, "localOut", it->second);
                                }
                            }
                            //
                            // send call release messages to application if it is necessary, the neighbors send the end messages to the other parts.

                            Packet *pkt = new Packet();
                            if (elem.second.node1 == myAddress) {
                                pkt->setSrcAddr(elem.second.node2);
                                pkt->setDestAddr(elem.second.node1);
                                pkt->setSourceId(elem.second.applicationId2);
                                pkt->setDestinationId(elem.second.applicationId1);
                            }
                            else {
                                pkt->setSrcAddr(elem.second.node1);
                                pkt->setDestAddr(elem.second.node2);
                                pkt->setSourceId(elem.second.applicationId1);
                                pkt->setDestinationId(elem.second.applicationId2);
                            }

                            pkt->setCallId(elem.first);
                            pkt->setType(RELEASEBREAK);

                            if (hasGUI()) {
                                char pkname[100];
                                sprintf(pkname, "ReleaseL3-%d-to-%d-Call id#%llud-flow-%llud-dest Id %i", pkt->getSrcAddr(),
                                        pkt->getDestAddr(), pkt->getCallId(), pkt->getFlowId(), pkt->getDestinationId());
                                pkt->setName(pkname);
                            }

                            // se envía hacia el destino
                            auto it = sourceIdGate.find(pkt->getDestinationId());
                            if (it == sourceIdGate.end())
                                throw cRuntimeError("Source id %i not registered", pkt->getDestinationId());
                            sendDelayed(pkt, par("breakDelay"), "localOut", it->second);
                        }
                    }
                    callInfomap.clear(); // clean call information
                    // end flee flows
                    for (auto elem : inputFlows) {
                        if (elem.second.destId == myAddress) {
                            Packet *pkt = new Packet();
                            pkt->setCallId(0);
                            pkt->setType(ENDFLOW);
                            pkt->setFlowId(elem.second.identify.flowId());
                            pkt->setSrcAddr(elem.second.identify.src());
                            pkt->setSourceId(elem.second.identify.srcId());
                            pkt->setDestAddr(myAddress);
                            pkt->setDestinationId(elem.second.destId);
                            auto it = sourceIdGate.find(pkt->getDestinationId());
                            if (it == sourceIdGate.end())
                                throw cRuntimeError("Source id %i not registered", pkt->getDestinationId());
                            if (hasGUI()) {
                                char pkname[100];
                                sprintf(pkname, "EndFlowL3-%d-to-%d-Call id#%llud-flow-%llud-dest Id %i", pkt->getSrcAddr(),
                                        pkt->getDestAddr(), pkt->getCallId(), pkt->getFlowId(), pkt->getDestinationId());
                                pkt->setName(pkname);
                            }
                            send(pkt, "localOut", it->second);
                        }
                    }
                    inputFlows.clear();
                    outputFlows.clear();
                }
                else if (event->type == NODE_RECOVERY_EV || event->type == LINK_RECOVERY_EV) {
                    for (auto & elem : neighbors) {
                        elem.second.state = UP;
                    }
                    for (unsigned int i = 0; i < portDataArray.size(); i++) {
                        portDataArray[i].occupation = portDataArray[i].nominalbw;
                        portDataArray[i].flowOccupation = portDataArray[i].nominalbw;
                        portDataArray[i].portStatus = UP;
                        portDataArray[i].max = portDataArray[i].occupation;
                        portDataArray[i].min = portDataArray[i].occupation;
                        portDataArray[i].mean = portDataArray[i].occupation;

                        ChangeBw val;
                        val.instant = simTime();
                        val.value = portDataArray[i].flowOccupation;
                        recordOccupation(portDataArray[i], val);
                        actualizePercentaje();
                    }
                }
                if (inmediateNotificationLink)
                    actualize(nullptr);
                actualizePercentaje();
                return;
            }
            // failure a link in the actual node
            if (event->type == NODE_FAILURE_EV || event->type == LINK_FAILURE_EV) {
                auto itNeig = neighbors.find(neigAddr);
                if (itNeig == neighbors.end())
                    throw cRuntimeError("Neighbor address not found check break event list");
                // search in the list the flows and active calls

                if (!endCalls) {
                    itNeig->second.state = DOWN; // only unidirectional
                    portDataArray[itNeig->second.port].portStatus = DOWN;
                }

                itNeig->second.failureTime = simTime();
                CallInfoMap breakCommunication;
                // list of call that must end
                for (auto itCallInfo = callInfomap.begin(); itCallInfo != callInfomap.end(); ) {
                    if (itCallInfo->second.port1 == itNeig->second.port
                            || itCallInfo->second.port2 == itNeig->second.port) {
                        //if (itCallInfo->second.port1 >= 0)
                        //    occupation[itCallInfo->second.port1] += itCallInfo->second.reserve;

                        //if (itCallInfo->second.port2 >= 0)
                        //    occupation[itCallInfo->second.port2] += itCallInfo->second.reserve;
                        breakCommunication.insert(std::make_pair(itCallInfo->first,itCallInfo->second));
                        callInfomap.erase(itCallInfo++);
                    }
                    else
                        ++itCallInfo;
                }
                for (auto elem : breakCommunication ){
                    while (!elem.second.outputFlows.empty()) {
                        FlowInfo flowinfo = elem.second.outputFlows.back();
                        elem.second.outputFlows.pop_back();
                        if (flowinfo.port != itNeig->second.port) {
                            // free bandwidth
                            portDataArray[flowinfo.port].flowOccupation += flowinfo.used;
                            ChangeBw val;
                            val.instant = simTime();
                            val.value = portDataArray[flowinfo.port].flowOccupation;
                            recordOccupation(portDataArray[flowinfo.port], val);
                            // Send end flow
                            Packet *pkt = new Packet();
                            pkt->setType(ENDFLOW);
                            pkt->setFlowId(flowinfo.identify.flowId());
                            pkt->setSrcAddr(flowinfo.identify.src());
                            pkt->setSourceId(flowinfo.identify.srcId());
                            if (elem.second.node1 == flowinfo.identify.src())
                                pkt->setDestAddr(elem.second.node2);
                            else
                                pkt->setDestAddr(elem.second.node1);
                            if (hasGUI()) {
                                char pkname[100];
                                sprintf(pkname, "EndFlowL3-%d-to-%d-Call id#%llud-flow-%llud-dest Id %i", pkt->getSrcAddr(),
                                        pkt->getDestAddr(), pkt->getCallId(), pkt->getFlowId(), pkt->getDestinationId());
                                pkt->setName(pkname);
                            }
                            send(pkt, "out", flowinfo.port);
                        }
                    }
                    // send call release messages
                    Packet *pkt = new Packet();
                    int forwartPort;
                    // check the direction that it is necessary to send the packet
                    if (itNeig->second.port == elem.second.port2) {
                        forwartPort = elem.second.port1;
                        pkt->setSrcAddr(elem.second.node1);
                        pkt->setDestAddr(elem.second.node2);
                        pkt->setSourceId(elem.second.applicationId1);
                        pkt->setDestinationId(elem.second.applicationId2);
                    }
                    else if (itNeig->second.port == elem.second.port1) {
                        forwartPort = elem.second.port2;
                        pkt->setSrcAddr(elem.second.node2);
                        pkt->setDestAddr(elem.second.node1);
                        pkt->setSourceId(elem.second.applicationId2);
                        pkt->setDestinationId(elem.second.applicationId1);
                    }
                    else
                        throw cRuntimeError("Unknown port");
                    // prepare the release of the call with a delay
                    pkt->setCallId(elem.first);
                    pkt->setType(RELEASEBREAK);
                    if (elem.second.node1 == myAddress || elem.second.node2 == myAddress) {
                        // se envía hacia el destino, se comprueba
                        Packet * pktAux = pkt->dup();
                        if (elem.second.node1 == myAddress) {
                            pktAux->setSrcAddr(elem.second.node2);
                            pktAux->setDestAddr(elem.second.node1);
                            pktAux->setSourceId(elem.second.applicationId2);
                            pktAux->setDestinationId(elem.second.applicationId1);
                        }
                        else {
                            pktAux->setSrcAddr(elem.second.node1);
                            pktAux->setDestAddr(elem.second.node2);
                            pktAux->setSourceId(elem.second.applicationId1);
                            pktAux->setDestinationId(elem.second.applicationId2);
                        }
                        auto it = sourceIdGate.find(pktAux->getDestinationId());
                        if (it == sourceIdGate.end())
                            throw cRuntimeError("Source id %i not registered", pkt->getDestinationId());
                        if (hasGUI()) {
                            char pkname[100];
                            printf(pkname, "ReleaseL3-%d-to-%d-Call id#%llud-flow-%llud-dest Id %i", pkt->getSrcAddr(),
                                    pkt->getDestAddr(), pkt->getCallId(), pkt->getFlowId(), pkt->getDestinationId());
                            pkt->setName(pkname);
                        }
                        send(pktAux, "localOut", it->second);
                    }
                    //pkt->setKind(forwartPort);

                    if (forwartPort != -1) {
                        if (elem.second.node1 == myAddress && elem.second.node2 == myAddress)
                            send(pkt, "out", forwartPort);
                        //scheduleAt(simTime(), pkt); // release immediately
                        else
                            sendDelayed(pkt,par("breakRelease"), "out", forwartPort);
                        //scheduleAt(simTime() + par("breakRelease"), pkt); // release with a delay
                        elem.second.state = END;
                    }
                    else
                        delete pkt;
                }
                if (!endCalls) {
                    for (auto it = inputFlows.begin(); it != inputFlows.end();) {
                        if ((it->second.port != -1 && it->second.portInput != -1)
                                && (portDataArray[it->second.port].portStatus == UP && portDataArray[it->second.portInput].portStatus == UP)) {
                            ++it;
                            continue;
                        }
                        else if ((it->second.port != -1) && (portDataArray[it->second.port].portStatus == UP)) {
                            ++it;
                            continue;
                        }
                        else if ((it->second.portInput != -1) && (portDataArray[it->second.portInput].portStatus == UP)) {
                            ++it;
                            continue;
                        }
                        if (it->second.dest == myAddress) {
                            // send end flow to application {
                            Packet *pkt = new Packet();
                            pkt->setCallId(0);
                            pkt->setType(ENDFLOW);
                            pkt->setFlowId(it->second.identify.flowId());
                            pkt->setSrcAddr(it->second.identify.src());
                            pkt->setSourceId(it->second.identify.srcId());
                            pkt->setDestAddr(myAddress);
                            pkt->setDestinationId(it->second.destId);
                            auto itAux = sourceIdGate.find(
                                    pkt->getDestinationId());
                            if (itAux == sourceIdGate.end())
                                throw cRuntimeError(
                                        "Source id %i not registered",
                                        pkt->getDestinationId());
                            send(pkt, "localOut", itAux->second);
                        }
                        inputFlows.erase(it++);
                    }
                    for (auto it = outputFlows.begin(); it != outputFlows.end();) {
                        if ((it->second.port != -1 && it->second.portInput != -1)
                                && (portDataArray[it->second.port].portStatus == UP
                                        && portDataArray[it->second.portInput].portStatus == UP)) {
                            ++it;
                            continue;
                        }
                        else if ((it->second.port != -1)
                                && (portDataArray[it->second.port].portStatus == UP)) {
                            ++it;
                            continue;
                        }
                        else if ((it->second.portInput != -1)
                                && (portDataArray[it->second.portInput].portStatus == UP)) {
                            ++it;
                            continue;
                        }
                        // send end flow to the other part
                        if (portDataArray[it->second.port].portStatus == UP) {
                            // send end flow to application {
                            Packet *pkt = new Packet();
                            pkt->setCallId(0);
                            pkt->setType(ENDFLOW);
                            pkt->setFlowId(it->second.identify.flowId());
                            pkt->setSrcAddr(it->second.identify.src());
                            pkt->setSourceId(it->second.identify.srcId());
                            pkt->setDestAddr(it->second.dest);
                            pkt->setDestinationId(it->second.destId);
                            if (hasGUI()) {
                                char pkname[100];
                                sprintf(pkname,
                                        "EndFlowL3-%d-to-%d-Call id#%llud-flow-%llud-dest Id %i",
                                        pkt->getSrcAddr(), pkt->getDestAddr(),
                                        pkt->getCallId(), pkt->getFlowId(),
                                        pkt->getDestinationId());
                                pkt->setName(pkname);
                            }
                            send(pkt, "out", it->second.port);
                        }
                        outputFlows.erase(it++);
                    }
                }
            }
            else if (event->type == NODE_RECOVERY_EV || event->type == LINK_RECOVERY_EV) {
                //
                auto itNeig = neighbors.find(neigAddr);
                if (itNeig == neighbors.end())
                    throw cRuntimeError("Neighbor address not found check break event list");
                itNeig->second.state = UP;
                portDataArray[itNeig->second.port].portStatus = UP;

                std::vector<FlowInfo *> listFlowsToModify;
                std::vector<FlowInfo *> listFlowsToModifyInput;
                getListFlowsToModifyStartFlow(itNeig->second.port, listFlowsToModify,listFlowsToModifyInput);

                if (portDataArray[itNeig->second.port].occupation != portDataArray[itNeig->second.port].nominalbw)
                    throw cRuntimeError("Restore link, occupation %llu nominal %llu",
                            portDataArray[itNeig->second.port].occupation, portDataArray[itNeig->second.port].nominalbw);

                if (!listFlowsToModifyInput.empty()) {
                    // TODO: Gestionar microcortes
                    // al terminar las llamadas debería estar el ancho de banda disponible ya que no se gestionan microcortes
                    if (!listFlowsToModify.empty()) // error
                        throw cRuntimeError("List of output flows should be empty if is a failure");


                }
                // if (simTime() - itNeig->second.failureTime > par("breakRelease"))
                //     occupation[itNeig->second.port] = nominalbw[itNeig->second.port];

            }
            if (inmediateNotificationLink)
                actualize(nullptr);
            actualizePercentaje();
        }
    }
}

// this method checks if it is possible to reserve enough bandwidth, it it possible return true, if not is possible sends a reject message to the origin and returns false.
bool FlowForwarding::procReserve(Packet *pk, int &portForward, int &destId)
{
    if (pk->getCallId() == 0)
        return false;

    auto it = rtable.end();
    int outPort = -1; // port of the next node
    int inputPort = -1; // port to the previous node
    int destAddr = pk->getDestAddr();
    int srcAddr = pk->getSrcAddr();
    int portInput = -1;

    bool rejected = false;

    if (portInput != -1 && portDataArray[portInput].portStatus == DOWN) // Impossible bidirectional communications
        rejected = true;

    if (destAddr != myAddress && !rejected) { // if rejected = true it isn't necessary more checks
        // Internal routing table,
        if (pk->getRouteArraySize() == 0) {
            it = rtable.find(destAddr);
            if (it != rtable.end())
                outPort = (*it).second;
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
            if (itNeig == neighbors.end())
                throw cRuntimeError("Neighbor node not found");
            outPort = itNeig->second.port;
        }
        // check next hop port status
        if (outPort != -1 && portDataArray[outPort].portStatus == DOWN) // check link status
            rejected = true;
    }

    // check if it is possible reserve enough bandwidth in both directions
    if (srcAddr != myAddress) {
        inputPort = pk->getArrivalGate()->getIndex();
    }

    if (!rejected) {
        if ((outPort != -1 && portDataArray[outPort].occupation < pk->getReserve())
            && (inputPort != -1 && portDataArray[inputPort].occupation < pk->getReserve()))
            rejected = true;
        if (outPort == -1 && destAddr != myAddress)
            rejected = true;
    }

    if (rejected) {
        // reject call
        Packet *pkt = pk->dup();
        callLost++;
        pkt->setType(REJECTED);

        pkt->setSrcAddr(pk->getDestAddr());
        pkt->setDestAddr(pk->getSrcAddr());

        pkt->setDestinationId(pk->getSourceId());
        pkt->setSourceId(pk->getDestinationId());

        if (hasGUI()) {
            char pkname[100];
            sprintf(pkname, "RejectedL3-%d-to-%d-Call id#%llud-flow-%llud-dest Id %i", pkt->getSrcAddr(),
                    pkt->getDestAddr(), pkt->getCallId(), pkt->getFlowId(), pkt->getDestinationId());
            pkt->setName(pkname);
        }

        if (srcAddr == myAddress) {
            // send up
            send(pkt, "localOut", pk->getArrivalGate()->getIndex());
        }
        else {
            // send Reject message to the sender node.
            send(pkt, "out", pk->getArrivalGate()->getIndex());
        }
        return false;
    }

    // enough resources, bandwidth reserved.
    if (outPort != -1)
        portDataArray[outPort].occupation -= pk->getReserve();

    if (inputPort != -1)
        portDataArray[inputPort].occupation -= pk->getReserve();

    actualizePercentaje();

    CallInfo callInfo;
    callInfo.node1 = pk->getSrcAddr();
    callInfo.node2 = pk->getDestAddr();
    callInfo.port1 = outPort;
    callInfo.port2 = inputPort;
    callInfo.reserve = pk->getReserve();
    callInfo.applicationId1 = pk->getSourceId();
    callInfo.applicationId2 = pk->getDestinationId();

    callInfomap.insert(std::make_pair(pk->getCallId(),callInfo));

    portForward = outPort;
    destId = callInfo.applicationId2;
    return true;
}

// this method obtain the output port of a free flow, if it is possible to determome the output port return true, in other case false, port portForward = -1 the destination is this node
bool FlowForwarding::getForwarPortFreeFlow(Packet *pk, int &portForward)
{
    portForward = -1;
    auto it = rtable.end();
    int destAddr = pk->getDestAddr();

    if (destAddr != myAddress) {
        // Internal routing table,
        if (pk->getRouteArraySize() == 0) {
            it = rtable.find(destAddr);
            if (it != rtable.end())
                portForward = (*it).second;
            else
                return false;

            // check port status
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
            portForward = itNeig->second.port;
        }
    }
    return true;
}

void FlowForwarding::checkPendingList()
{

    // first check delayed list.
    if (!delayedFlows.empty()) {
        for (auto it = delayedFlows.begin();it != delayedFlows.end();) { // search the first that you can send
        // proc
            simtime_t delay = it->end - it->start;
            if (it->port != -1 && portDataArray[it->port].flowOccupation > it->used && portDataArray[it->port].portStatus == UP) {
                portDataArray[it->port].flowOccupation -= it->used;
                ChangeBw val;
                val.instant = simTime();
                val.value = portDataArray[it->port].flowOccupation;
                recordOccupation(portDataArray[it->port], val);
                Packet *pkStartFlow = new Packet();
                it->delayed = true;
                it->delay = it->end - it->start;
                if (it->identify.callId() > 0) {
                    auto itCallInfoAux = callInfomap.find(it->identify.callId());
                    itCallInfoAux->second.outputFlows.push_back(*it);
                    // send start flow to the next node
                    if (itCallInfoAux->second.node1 == it->identify.src()) {
                        pkStartFlow->setSrcAddr(itCallInfoAux->second.node1);
                        pkStartFlow->setDestAddr(itCallInfoAux->second.node2);
                    }
                    else {
                        pkStartFlow->setSrcAddr(itCallInfoAux->second.node2);
                        pkStartFlow->setDestAddr(itCallInfoAux->second.node1);
                    }
                }
                else {
                    outputFlows[it->identify] = *it;
                    pkStartFlow->setSrcAddr(it->identify.src());
                    pkStartFlow->setDestAddr(it->dest); // source routing,
                    if (!it->sourceRouting.empty()) {
                        pkStartFlow->setRouteArraySize(it->sourceRouting.size());
                        for (unsigned int i = 0; i < pkStartFlow->getRouteArraySize(); i++) {
                            pkStartFlow->setRoute(i, it->sourceRouting[i]);
                        }
                    }
                }
                pkStartFlow->setCallId(it->identify.callId());
                pkStartFlow->setFlowId(it->identify.flowId());
                pkStartFlow->setSourceId(it->identify.srcId());
                pkStartFlow->setDestinationId(it->destId);
                pkStartFlow->setType(STARTFLOW);
                pkStartFlow->setReserve(it->used);
                if (hasGUI()) {
                    char pkname[100];
                    sprintf(pkname, "StartFlowL3-%d-to-%d-Call id#%llud-flow-%llud-dest Id %i", pkStartFlow->getSrcAddr(),
                            pkStartFlow->getDestAddr(), pkStartFlow->getCallId(), pkStartFlow->getFlowId(), pkStartFlow->getDestinationId());
                    pkStartFlow->setName(pkname);
                }
                if (simulationMode == FLOWMODE)
                    send(pkStartFlow, "out", it->port);
                else
                    delete pkStartFlow;
                scheduleAt(simTime()+delay,it->endMsg);
                it = delayedFlows.erase(it);
            }
            else
                ++it;
        }
    }

    int storeSize = par("storeAndForwardSize").intValue();
    bool full = false;
    if (storeSize != -1) {
        int delayedFlow = 0;
        for (auto it = pendingFlows.begin(); it != pendingFlows.end(); ++it) {
            if (it->delayed)
                delayedFlow++;
        }
        delayedFlow += (int)delayedFlows.size();
        if (delayedFlow >= storeSize)
            full = true;
    }


    if (!pendingFlows.empty()) {
        for (auto it = pendingFlows.begin(); it != pendingFlows.end();) {
            //
            bool isInInput = false;
            if (it->identify.callId() > 0) {
                auto itCallInfoAux = callInfomap.find(it->identify.callId());
                if (itCallInfoAux == callInfomap.end()) {
                    it = pendingFlows.erase(it);
                    continue;
                }
                // check if the flow is in the input but not in the output
                bool isInOutput = false;
                for (auto elem : itCallInfoAux->second.outputFlows) {
                    if (elem.identify.flowId() == it->identify.flowId()) {
                        // flow in the input erase it from pending
                        isInOutput = true;
                        it = pendingFlows.erase(it);
                        break;
                    }
                }
                if (isInOutput)
                    continue;
                // check the input list
                for (auto elem : itCallInfoAux->second.inputFlows) {
                    if (elem.identify.flowId() == it->identify.flowId()) {
                        isInInput = true;
                        break;
                    }
                }
            }
            else {
                auto itFlow = outputFlows.find(it->identify);
                if (itFlow != outputFlows.end())
                    continue;
                itFlow = inputFlows.find(it->identify);
                if (itFlow != inputFlows.end())
                    isInInput = true;
                // check if is alredy in output
            }

            if (!isInInput) {
                it = pendingFlows.erase(it);
                continue;
            }

            if (it->port != -1 && portDataArray[it->port].flowOccupation > it->used && portDataArray[it->port].portStatus == UP) {
                portDataArray[it->port].flowOccupation -= it->used;
                ChangeBw val;
                val.instant = simTime();
                val.value = portDataArray[it->port].flowOccupation;
                recordOccupation(portDataArray[it->port], val);
                Packet *pkStartFlow = new Packet();
                if (!full)
                    it->delayed = true;
                it->delay = simTime() - it->start;

                if (it->identify.callId() > 0) {
                    auto itCallInfoAux = callInfomap.find(it->identify.callId());
                    itCallInfoAux->second.outputFlows.push_back(*it);
                    // send start flow to the next node

                    if (itCallInfoAux->second.node1 == it->identify.src()) {
                        pkStartFlow->setSrcAddr(itCallInfoAux->second.node1);
                        pkStartFlow->setDestAddr(itCallInfoAux->second.node2);
                    }
                    else {
                        pkStartFlow->setSrcAddr(itCallInfoAux->second.node2);
                        pkStartFlow->setDestAddr(itCallInfoAux->second.node1);
                    }
                }
                else {
                    outputFlows[it->identify] = *it;
                    pkStartFlow->setSrcAddr(it->identify.src());
                    pkStartFlow->setDestAddr(it->dest); // source routing,
                    if (!it->sourceRouting.empty()) {
                        pkStartFlow->setRouteArraySize(it->sourceRouting.size());
                        for (unsigned int i = 0; i < pkStartFlow->getRouteArraySize(); i++) {
                            pkStartFlow->setRoute(i, it->sourceRouting[i]);
                        }
                    }
                }
                pkStartFlow->setCallId(it->identify.callId());
                pkStartFlow->setFlowId(it->identify.flowId());
                pkStartFlow->setSourceId(it->identify.srcId());
                pkStartFlow->setDestinationId(it->destId);
                pkStartFlow->setType(STARTFLOW);
                pkStartFlow->setReserve(it->used);

                if (hasGUI()) {
                    char pkname[100];
                    sprintf(pkname, "StartFlowL3-%d-to-%d-Call id#%llud-flow-%llud-dest Id %i", pkStartFlow->getSrcAddr(),
                            pkStartFlow->getDestAddr(), pkStartFlow->getCallId(), pkStartFlow->getFlowId(), pkStartFlow->getDestinationId());
                    pkStartFlow->setName(pkname);
                }
                if (simulationMode == FLOWMODE)
                    send(pkStartFlow, "out", it->port);
                else
                    delete pkStartFlow;
                it = pendingFlows.erase(it);
            }
            else
                ++it;
        }
    }
}

bool FlowForwarding::preProcPacket(Packet *pk)
{
    if (pk->getCallId() > 0) {
        auto itCallInfo = callInfomap.find(pk->getCallId());

        // actions
        if (pk->getType() == RESERVE || pk->getType() == RESERVEBK) {
            if (itCallInfo != callInfomap.end())
                throw cRuntimeError("Error in callId parameter, call exist already : RESERVE");

        }
        else {
            if (itCallInfo == callInfomap.end()) {
                if (pk->getType() == STARTFLOW || pk->getType() == ENDFLOW || pk->getType() == FLOWCHANGE || pk->getType() == DATATYPE) {
                    // discard
                    delete pk;
                    return false;
                }
                else if (pk->getType() == RELEASE || pk->getType() == RELEASEBREAK) {
                    // discard
                    delete pk;
                    return false;
                }
                else
                    throw cRuntimeError("Error in callId parameter, call doesn't exist ");
            }
            else if (pk->isSelfMessage() && (pk->getType() == RELEASE || pk->getType() == RELEASEBREAK)) {
                // TODO : Micro cortes, comprobar si está el enlace activo.

                if (itCallInfo->second.port1 >= 0) {
                    // check port status

                    // TODO : Microcortes
                    /*                NeighborsPorts * neig = nullptr;
                     for (auto &elem : neighbors) {
                     if(elem.second.port == itCallInfo->second.port1) {
                     neig = &elem.second;
                     if (elem.second.state == UP) // micro corte, no debería enviarse,
                     portData[itCallInfo->second.port1].occupation += itCallInfo->second.reserve;
                     break;
                     }
                     }
                     if (neig && neig->state == UP)
                     portData[itCallInfo->second.port1].occupation += itCallInfo->second.reserve;
                     else
                     portData[itCallInfo->second.port1].occupation += itCallInfo->second.reserve;*/
                    //
                    portDataArray[itCallInfo->second.port1].occupation += itCallInfo->second.reserve;
                }

                if (itCallInfo->second.port2 >= 0) {
                    // check port status
                    // TODO : Microcortes
                    /*
                     NeighborsPorts * neig = nullptr;
                     for (auto &elem : neighbors) {
                     if(elem.second.port == itCallInfo->second.port1) {
                     neig = &elem.second;
                     if (elem.second.state == UP) // micro corte, no debería enviarse,
                     portData[itCallInfo->second.port1].occupation[itCallInfo->second.port1] += itCallInfo->second.reserve;
                     break;
                     }
                     }
                     if (neig && neig->state == UP)
                     portData[itCallInfo->second.port2].occupation[itCallInfo->second.port2] += itCallInfo->second.reserve;
                     else
                     portData[itCallInfo->second.port2].occupation[itCallInfo->second.port2] += itCallInfo->second.reserve;*/
                    portDataArray[itCallInfo->second.port2].occupation += itCallInfo->second.reserve;
                    actualizePercentaje();
                }
                // send packet to next node
                if (pk->getKind() >= 0)
                    send(pk, "out", pk->getKind());
                else
                    delete pk;

                // check that flows are empty
                if (!itCallInfo->second.outputFlows.empty())
                    throw cRuntimeError("flows list is not empty");
                callInfomap.erase(itCallInfo);
                checkPendingList();
                actualizePercentaje();
                return false;
            }
        }
    }
    else {
        if (pk->getType() == ENDFLOW || pk->getType() == FLOWCHANGE || pk->getType() == CROUTEFLOWEND || (pk->getType() == DATATYPE && pk->getLast())) {
            // check if exist the for in other case
            FlowIdentification flowId(pk->getSrcAddr(),pk->getFlowId(),pk->getCallId(),pk->getSourceId());
            auto itFlow = inputFlows.find(flowId);
            if (itFlow == inputFlows.end()) {
                delete pk;
                return false;
            }
         }
    }
    return true;
}

bool FlowForwarding::procStartFlow(Packet *pk, const int & portForward, const int & portInput)
{
    bool isCallOriented = (pk->getCallId() > 0);
    auto itCallInfo = callInfomap.end();
    if (isCallOriented) {
        itCallInfo = callInfomap.find(pk->getCallId());
        // flow
        if (itCallInfo == callInfomap.end() || itCallInfo->second.state != CALLUP) { // call not established yet, delete flow
            delete pk;
            return false;
        }
    }

    FlowIdentification flowId(pk->getSrcAddr(),pk->getFlowId(),pk->getCallId(),pk->getSourceId());

    FlowInfo flowInfo;
    flowInfo.identify = flowId;
    flowInfo.dest = pk->getDestAddr(); // call flows doesn't use, only independent flows use
    flowInfo.used = pk->getReserve();
    flowInfo.port = portForward;
    flowInfo.portInput = portInput;
    flowInfo.start = simTime();
    if (!isCallOriented)
        flowInfo.destId = pk->getDestinationId();
    else
        flowInfo.destId = -1;

    // save the source route only for free flows
    if (pk->getCallId() == 0 && pk->getRouteArraySize() > 0) {
        for (unsigned int i = 0; i < pk->getRouteArraySize(); i++) {
            flowInfo.sourceRouting.push_back(pk->getRoute(i));
        }
    }

    // check if exists this flow, if it exists throw and error
    if (isCallOriented) {
        for (auto elem : itCallInfo->second.outputFlows) {
            if (elem.identify  == flowId)
                throw cRuntimeError("Error Flow id already reserved in the output flows");
        }

        for (auto elem : itCallInfo->second.inputFlows) {
            if (elem.identify == flowId)
                throw cRuntimeError("Error Flow id already reserved in the input flows");
        }
        // register the flow in the input list
        itCallInfo->second.inputFlows.push_back(flowInfo);
    }
    else {
        // flow not assigned to a call search in the ports info

        if (pk->getType() == CROUTEFLOWSTART) {

            inputFlows[flowId] = flowInfo;

            auto itFlow = outputFlows.find(flowId);
            if (itFlow != outputFlows.end()) {
                // reset bandwidth
                if (portDataArray[itFlow->second.port].portStatus == UP) {
                    portDataArray[itFlow->second.port].flowOccupation += itFlow->second.used;
                    ChangeBw val;
                    val.instant = simTime();
                    val.value = portDataArray[itFlow->second.port].flowOccupation;
                    recordOccupation(portDataArray[itFlow->second.port], val);
                }
            }
        }
        else {
            auto itFlow = inputFlows.find(flowId);
            if (itFlow != inputFlows.end())
                throw cRuntimeError("Error Flow id already  in the input port  flows: port %i", portInput);
            inputFlows[flowId] = flowInfo;

            itFlow = outputFlows.find(flowId);
            if (itFlow != outputFlows.end())
                throw cRuntimeError("Error Flow id already  in the output port  flows: port %i", portForward);
        }
    }

    // check if port is up and if there is enough bandwidth unreserved for not oriented flows.
    if (portForward != -1) {

        if (portDataArray[portForward].portStatus == DOWN) {
            delete pk;
            emit(dropSignal,pk);
            return false;
        }

        // limits for not oriented flows
        if (!isCallOriented) {
            double limitcall = (double) portDataArray[portForward].nominalbw * reserveCall;
            if (limitcall > 0 && limitcall <= (double) portDataArray[portForward].occupation) {
                delete pk;
                return false;
            }
            double limitflow = (double) portDataArray[portForward].nominalbw * reserveFlows;
            if (limitflow > 0 && limitflow <= (double) portDataArray[portForward].flowOccupation) {
                delete pk;
                return false;
            }
        }
    }

    // consume bandwidth
    if (portForward != -1) {
        if (portDataArray[portForward].flowOccupation > pk->getReserve()) {
            portDataArray[portForward].flowOccupation -= pk->getReserve();
            ChangeBw val;
            val.instant = simTime();
            val.value = portDataArray[portForward].flowOccupation;
            recordOccupation(portDataArray[portForward], val);

            if (itCallInfo != callInfomap.end())
                itCallInfo->second.outputFlows.push_back(flowInfo);
            else
                outputFlows[flowId] = flowInfo;
        }
        else {
            if (!flodAdmision(pk->getReserve(), nullptr, &flowInfo, portForward, portInput, (PacketCode) pk->getType())) {
                numDrop++;
                emit(dropSignal,numDrop);
                delete pk;
                return false;
            }
        }
    }
    return true;
}


bool FlowForwarding::procDataType(Packet *pk, const int & portForward, const int & portInput,  CallInfo* callInfo)
{
    bool isCallOriented = (pk->getCallId() > 0);
    if (isCallOriented) {
        if (callInfo == nullptr || callInfo->state != CALLUP) { // call not established yet, delete flow
            numDrop++;
            emit(dropSignal,numDrop);
            delete pk;
            return false;
        }
    }

    FlowIdentification flowId(pk->getSrcAddr(),pk->getFlowId(),pk->getCallId(),pk->getSourceId());

    FlowInfo flowInfo;
    flowInfo.identify = flowId;
    flowInfo.dest = pk->getDestAddr(); // call flows doesn't use, only independent flows use
    flowInfo.used = pk->getReserve();
    flowInfo.port = portForward;
    flowInfo.portInput = portInput;
    flowInfo.start = simTime();
    if (!isCallOriented)
        flowInfo.destId = pk->getDestinationId();
    else
        flowInfo.destId = -1;

    // save the source route only for free flows
    if (pk->getCallId() == 0 && pk->getRouteArraySize() > 0) {
        for (unsigned int i = 0; i < pk->getRouteArraySize(); i++) {
            flowInfo.sourceRouting.push_back(pk->getRoute(i));
        }
    }

    // check if exists this flow, if it exists throw and error
    bool isInOutput = false;
    bool isInInput = false;

    if (isCallOriented) {
        auto itAux = std::find(callInfo->outputFlows.begin(),callInfo->outputFlows.end(),flowInfo);
        if (itAux != callInfo->outputFlows.end())
            isInOutput = true;
        itAux = std::find(callInfo->inputFlows.begin(),callInfo->inputFlows.end(),flowInfo);
        if (itAux != callInfo->inputFlows.end())
            isInInput = true;
        // register the flow in the input list
        if (!isInInput)
            callInfo->inputFlows.push_back(flowInfo);
    }
    else {
        auto itAux1 = inputFlows.find(flowId);
        if (itAux1 != inputFlows.end())
            isInInput = true;
        auto itAux2 = outputFlows.find(flowId);
        if (itAux2 != outputFlows.end())
            isInOutput = true;
        throw cRuntimeError("TODO: Support for non call traffic");
    }

    // check if port is up and if there is enough bandwidth unreserved for not oriented flows.

    // TODO: this must be re-do for non call traffic
    if (portForward != -1) {

        if (portDataArray[portForward].portStatus == DOWN) {
            numDrop++;
            emit(dropSignal,numDrop);
            delete pk;
            return false;
        }
        // limits for not oriented flows
        if (!isCallOriented && isInOutput) {
            double limitcall = (double) portDataArray[portForward].nominalbw * reserveCall;
            if (limitcall > 0 && limitcall <= (double) portDataArray[portForward].occupation) {
                numDrop++;
                emit(dropSignal,numDrop);
                delete pk;
                return false;
            }
            double limitflow = (double) portDataArray[portForward].nominalbw * reserveFlows;
            if (limitflow > 0 && limitflow <= (double) portDataArray[portForward].flowOccupation) {
                numDrop++;
                emit(dropSignal,numDrop);
                delete pk;
                return false;
            }
        }
    }

    if (pk->getLast()) {
        // end flow, release and return
        return procEndFlow(pk);
    }

    if (portForward != -1 && isInOutput)
        return true;

    // consume bandwidth
    if (portForward != -1) {

        if (portDataArray[portForward].flowOccupation >= pk->getReserve()) {
            portDataArray[portForward].flowOccupation -= pk->getReserve();
            ChangeBw val;
            val.instant = simTime();
            val.value = portDataArray[portForward].flowOccupation;
            recordOccupation(portDataArray[portForward], val);

            if (callInfo != nullptr)
                callInfo->outputFlows.push_back(flowInfo);
            else
                outputFlows[flowId] = flowInfo;
        }
        else {
            if (!flodAdmision(pk->getReserve(), nullptr, &flowInfo, portForward, portInput, (PacketCode) pk->getType())) {
                numDrop++;
                emit(dropSignal,numDrop);
                delete pk;
                return false;
            }
        }
    }
    return true;
}


bool FlowForwarding::flodAdmision(const uint64_t &reserve, FlowInfo *flowInfoOutputPtr, FlowInfo *flowInfoInputPtr, const int & portForward, const int & portInput, PacketCode codeStart)
{
    // consume bandwidth
    auto itCallInfo = callInfomap.end();
    if (flowInfoInputPtr->identify.callId() != 0)
        itCallInfo = callInfomap.find(flowInfoInputPtr->identify.callId());
    auto itAux = pendingFlows.end();

    switch (flowAdmisionMode) {
    case STOREANDFORWARD:
    case DISCARD:
        itAux = std::find(pendingFlows.begin(), pendingFlows.end(),*flowInfoInputPtr);
        if (itAux == pendingFlows.end())
            pendingFlows.push_back(*flowInfoInputPtr);

        // delete the output flow if exist and send end flow
        if (flowInfoOutputPtr != nullptr) {
            if (itCallInfo != callInfomap.end()) {
                auto it = std::find(itCallInfo->second.outputFlows.begin(),
                        itCallInfo->second.outputFlows.end(), *flowInfoInputPtr);
                if (it == itCallInfo->second.outputFlows.end())
                    throw cRuntimeError("Error in call outputFlows list");
                itCallInfo->second.outputFlows.erase(it);
            }
            else {
                auto itFlow = outputFlows.find(flowInfoInputPtr->identify);
                if (itFlow == outputFlows.end())
                    throw cRuntimeError("Error in outputFlows list");
                outputFlows.erase(itFlow);
            }
            // send end flow
            if (simulationMode == FLOWMODE) {
                Packet * pkt = new Packet();
                pkt->setSrcAddr(flowInfoInputPtr->identify.src());
                pkt->setCallId(flowInfoInputPtr->identify.callId());
                pkt->setFlowId(flowInfoInputPtr->identify.flowId());
                pkt->setSourceId(flowInfoInputPtr->identify.srcId());
                pkt->setDestAddr(flowInfoInputPtr->destId);
                pkt->setReserve(flowInfoInputPtr->used);
                pkt->setType(ENDFLOW);
                if (!(flowInfoInputPtr->sourceRouting.empty())) {
                    pkt->setRouteArraySize(
                            flowInfoInputPtr->sourceRouting.size());
                    for (unsigned int i = 0;
                            i < flowInfoInputPtr->sourceRouting.size(); i++)
                        pkt->setRoute(i, flowInfoInputPtr->sourceRouting[i]);
                }

                if (hasGUI()) {
                    char pkname[100];
                    sprintf(pkname,
                            "EndFlowL3-%d-to-%d-Call id#%llud-flow-%llud-dest Id %i",
                            pkt->getSrcAddr(), pkt->getDestAddr(),
                            pkt->getCallId(), pkt->getFlowId(),
                            pkt->getDestinationId());
                    pkt->setName(pkname);
                }
                send(pkt, "out", portForward);
            }
        }
        flowInfoOutputPtr = nullptr;
        return false;
        break;
    default:
        // TODO: implementar el share mode
        if (flowInfoOutputPtr == nullptr) {
            if (itCallInfo != callInfomap.end() && flowInfoOutputPtr == nullptr)
                itCallInfo->second.outputFlows.push_back(*flowInfoInputPtr);
            else
                outputFlows[flowInfoInputPtr->identify] = *flowInfoInputPtr;
        }
        else {
            flowInfoOutputPtr->used = reserve;
        }

        std::vector<FlowInfo *> listFlowsToModify;
        std::vector<FlowInfo *> listFlowsToModifyInput;
        getListFlowsToModifyStartFlow(flowInfoInputPtr->port, listFlowsToModify, listFlowsToModifyInput);
        if (flowDist != nullptr) {
            if (flowDist->startShare(listFlowsToModify, listFlowsToModifyInput, portDataArray[portForward].nominalbw)) {
                uint64_t oc = 0;
                for (auto elem : listFlowsToModify)
                    oc += elem->used;

                portDataArray[portForward].flowOccupation = portDataArray[portForward].nominalbw - oc;
                portDataArray[portForward].overload = true;
                ChangeBw val;
                val.instant = simTime();
                val.value = portDataArray[portForward].flowOccupation;

                recordOccupation(portDataArray[portForward], val);
                // enviar mensajes de actualización del flujo.
                if (simulationMode == FLOWMODE) {
                    for (auto itAux = listFlowsToModify.begin(); itAux != listFlowsToModify.end(); ++itAux) {
                        //
                        if ((*itAux)->identify == flowInfoInputPtr->identify) {
                            // actualize use in the packet
                            continue;
                        }
                        Packet * pkt = new Packet();
                        pkt->setSrcAddr((*itAux)->identify.src());
                        pkt->setCallId((*itAux)->identify.callId());
                        pkt->setFlowId((*itAux)->identify.flowId());
                        pkt->setSourceId((*itAux)->identify.srcId());
                        pkt->setDestAddr((*itAux)->dest);
                        pkt->setReserve((*itAux)->used);
                        pkt->setType(FLOWCHANGE);
                        if (!(*itAux)->sourceRouting.empty()) {
                            pkt->setRouteArraySize((*itAux)->sourceRouting.size());
                            for (unsigned int i = 0; i < (*itAux)->sourceRouting.size(); i++)
                                pkt->setRoute(i, (*itAux)->sourceRouting[i]);
                        }
                        if (hasGUI()) {
                            char pkname[100];
                            sprintf(pkname,
                                    "FlowChangeL3-%d-to-%d-Call id#%llud-flow-%llud-dest Id %i",
                                    pkt->getSrcAddr(), pkt->getDestAddr(),
                                    pkt->getCallId(), pkt->getFlowId(),
                                    pkt->getDestinationId());
                            pkt->setName(pkname);
                        }
                        send(pkt, "out", portForward);
                    }
                }
            }
        }
        else
            throw cRuntimeError("Not definition present \"flowClass\": %s", par("flowClass").stringValue());
    }
    return true;
}


bool FlowForwarding::procFlowChange(Packet *pk, const int & portForward, const int & portInput)
{
    bool isCallOriented = (pk->getCallId() > 0);
    auto itCallInfo = callInfomap.end();
    if (isCallOriented) {
        itCallInfo = callInfomap.find(pk->getCallId());
        // flow
        if (itCallInfo == callInfomap.end() || itCallInfo->second.state != CALLUP) { // call not established yet, delete flow
            delete pk;
            return false;
        }
    }


    FlowIdentification flowId(pk->getSrcAddr(),pk->getFlowId(),pk->getCallId(),pk->getSourceId());

    FlowInfo flowInfo;
    flowInfo.identify = flowId;
    flowInfo.dest = pk->getDestAddr(); // call flows doesn't use, only independent flows use
    flowInfo.used = pk->getReserve();
    flowInfo.port = portForward;
    flowInfo.portInput = portInput;
    if (!isCallOriented)
        flowInfo.destId = pk->getDestinationId();
    else
        flowInfo.destId = -1;

    // is a flow change, check if the flow exist in the input list
    FlowInfo *flowInfoInputPtr = nullptr;
    if (isCallOriented) {
        auto it = std::find(itCallInfo->second.inputFlows.begin(),itCallInfo->second.inputFlows.end(),flowInfo);
        if (it == itCallInfo->second.inputFlows.end()) {
            delete pk;
            return false;
        }
        else if (it->used == pk->getReserve()) { // no change, nothing to do
            delete pk;
            return false;
        }
        else
            flowInfoInputPtr = &(*it);
    }
    else {
        auto itFlow = inputFlows.find(flowId);
        if (itFlow == inputFlows.end()) {
            delete pk;
            return false;
        }
        else if (itFlow->second.used == pk->getReserve()) { // no change, nothing to do
            delete pk;
            return false;
        }
        else
            flowInfoInputPtr = &(itFlow->second);
    }

    // the flow exist and there is a change in the used bandwidth
    flowInfoInputPtr->used =  pk->getReserve();

    // check if exists this flow, if it exists in the output

    FlowInfo *flowInfoOutputPtr = nullptr;
    if (isCallOriented) {
        auto it = std::find(itCallInfo->second.outputFlows.begin(),itCallInfo->second.outputFlows.end(),flowInfo);
        if (it != itCallInfo->second.outputFlows.end())
            flowInfoOutputPtr = &(*it);
    }
    else {
        auto itFlow = outputFlows.find(flowId);
        if (itFlow != outputFlows.end())
            flowInfoOutputPtr = &(itFlow->second);
    }

    if (flowInfoOutputPtr && flowInfoOutputPtr->used == pk->getReserve()) {
        delete pk;
        return false;
    }

    // check if port is up and if there is enough bandwidth unreserved for not oriented flows.
    if (portForward != -1) {

        if (portDataArray[portForward].portStatus == DOWN) {
            delete pk;
            return false;
        }

        // free the used bandwidth
        if (flowInfoOutputPtr) {
            portDataArray[portForward].flowOccupation += flowInfoOutputPtr->used;
            ChangeBw val;
            val.instant = simTime();
            val.value = portDataArray[portForward].flowOccupation;
            recordOccupation(portDataArray[portForward], val);
        }

        // limits for not oriented flows
        if (!isCallOriented) {
            double limitcall = (double) portDataArray[portForward].nominalbw * reserveCall;
            if (limitcall > 0 && limitcall <= (double) portDataArray[portForward].occupation) {
                delete pk;
                return false;
            }
            double limitflow = (double) portDataArray[portForward].nominalbw * reserveFlows;
            if (limitflow > 0 && limitflow <= (double) portDataArray[portForward].flowOccupation) {
                delete pk;
                return false;
            }
        }
    }

    // consume bandwidth
    if (portForward != -1) {
        if (portDataArray[portForward].flowOccupation > pk->getReserve()) {
            portDataArray[portForward].flowOccupation -= pk->getReserve();
            ChangeBw val;
            val.instant = simTime();
            val.value = portDataArray[portForward].flowOccupation;

            recordOccupation(portDataArray[portForward], val);
            if (flowInfoOutputPtr != nullptr)
            {
                flowInfoOutputPtr->used = pk->getReserve();
            }
            else
            {
                if (itCallInfo != callInfomap.end())
                    itCallInfo->second.outputFlows.push_back(*flowInfoInputPtr);
                else
                    outputFlows[flowId] = *flowInfoInputPtr;
            }
        }
        else {
            if (!flodAdmision(pk->getReserve(), flowInfoOutputPtr, flowInfoInputPtr, portForward, portInput, FLOWCHANGE)) {
                delete pk;
                return false;
            }
        }
    }
    return true;
}

bool FlowForwarding::procEndFlow(Packet *pk)
{
   if (flowAdmisionMode == STOREANDFORWARD)
       return procEndFlowStoreAndForward(pk);
   else
       return procEndFlowLost(pk);
}

bool FlowForwarding::procEndFlowLost(Packet *pk)
{
    bool isCallOriented = (pk->getCallId() > 0);

    FlowIdentification flowId(pk->getSrcAddr(),pk->getFlowId(),pk->getCallId(),pk->getSourceId());

    if (isCallOriented) {
        auto itCallInfo = callInfomap.find(pk->getCallId());

        for (auto it = itCallInfo->second.inputFlows.begin(); it != itCallInfo->second.inputFlows.end(); ++it) {
            if (it->identify == flowId) {
                itCallInfo->second.inputFlows.erase(it);
                break;
            }
        }

        auto it = itCallInfo->second.outputFlows.begin();

        while (it != itCallInfo->second.outputFlows.end()) {
            if (it->identify == flowId)
                break;
            ++it;
        }

        if (it == itCallInfo->second.outputFlows.end()) {

            // It has been impossible to send the start flow message to the next hop,  delete and return.
            for (auto it = pendingFlows.begin(); it != pendingFlows.end(); ++it) {
                if (it->identify == flowId) {
                    pendingFlows.erase(it);
                    break;
                }
            }
            if (pk->getDestAddr() == myAddress) // send
                return true;

            delete pk;
            return false;
            // throw cRuntimeError("Error Flow id not found reserved");
        }
        if (it->port != -1) {
            portDataArray[it->port].flowOccupation += it->used;
            ChangeBw val;
            val.instant = simTime();
            val.value = portDataArray[it->port].flowOccupation;
            recordOccupation(portDataArray[it->port], val);
        }
        itCallInfo->second.outputFlows.erase(it);
    }
    else
    {

        auto itFlowInput = inputFlows.find(flowId);
        auto itFlowOutput = outputFlows.find(flowId);

        if (itFlowInput == inputFlows.end()) {
            if (itFlowOutput != outputFlows.end())
                throw cRuntimeError("In outputFlows but not in inputFlows");
            else if (pk->getDestAddr() != myAddress) {
                delete pk;
                return false;
            }
        }

        inputFlows.erase(itFlowInput);
        if (itFlowOutput != outputFlows.end()) {
            portDataArray[itFlowOutput->second.port].flowOccupation += itFlowOutput->second.used;
            ChangeBw val;
            val.instant = simTime();
            val.value = portDataArray[itFlowOutput->second.port].flowOccupation;
            recordOccupation(portDataArray[itFlowOutput->second.port], val);
            outputFlows.erase(itFlowOutput);
        }
        else {
            delete pk;
            return false;
        }
    }
    return true;
}

bool FlowForwarding::procEndFlowStoreAndForward(Packet *pk)
{
    bool isCallOriented = (pk->getCallId() > 0);

    FlowIdentification flowId(pk->getSrcAddr(),pk->getFlowId(),pk->getCallId(),pk->getSourceId());

    if (pk->isSelfMessage()) {// delayed end
        for (auto it = delayedFlows.begin();it != delayedFlows.end();++it) { // search the first that you can send
        // proc
            if (it->identify == flowId) {
                it->endMsg = nullptr;
                throw cRuntimeError("Flow in delayed list error");
                break;
            }
        }
    }
    int storeSize = par("storeAndForwardSize").intValue();
    bool full = false;
    if (storeSize != -1) {
        int delayedFlow = 0;
        for (auto it = pendingFlows.begin(); it != pendingFlows.end(); ++it) {
            if (it->delayed)
                delayedFlow++;
        }
        delayedFlow += (int)delayedFlows.size();
        if (delayedFlow >= storeSize)
            full = true;
    }


    if (isCallOriented) {
        auto itCallInfo = callInfomap.find(pk->getCallId());

        // check if the flow is in the pending list.
        auto itOut = itCallInfo->second.outputFlows.begin();
        while (itOut != itCallInfo->second.outputFlows.end()) {
            if (itOut->identify == flowId)
                break;
            ++itOut;
        }

        bool delayed = false;
        if (itOut == itCallInfo->second.outputFlows.end()) {// delayed flow, move to delayed list
            delayed = true;
        }

        for (auto it = itCallInfo->second.inputFlows.begin(); it != itCallInfo->second.inputFlows.end(); ++it) {
            if (it->identify == flowId) {
                if (delayed  && !pk->isSelfMessage() && !full) {
                    if (pk->getDestAddr() != myAddress)
                        it->endMsg = pk;
                    it->end = simTime();
                    delayedFlows.push_back(*it);
                }
                itCallInfo->second.inputFlows.erase(it);
                break;
            }
        }

        if (delayed) {
            // It has been impossible to send the start flow message to the next hop,  record the time of end
            for (auto it = pendingFlows.begin(); it != pendingFlows.end(); ++it) {
                if (it->identify == flowId) {
                    pendingFlows.erase(it);
                    break;
                }
            }
            if (pk->getDestAddr() == myAddress) // send
                return true;
            if (pk->isSelfMessage())
                delete pk;
            return (false);
            // throw cRuntimeError("Error Flow id not found reserved");
        }
        // check if the flow has been delayed, in this case delay the end flow
        if (!pk->isSelfMessage() && itOut->delayed && !full) {
            scheduleAt(simTime()+itOut->delay,pk);
            return (false);
        }

        if (itOut->port != -1) {
            portDataArray[itOut->port].flowOccupation += itOut->used;
            ChangeBw val;
            val.instant = simTime();
            val.value = portDataArray[itOut->port].flowOccupation;
            recordOccupation(portDataArray[itOut->port], val);
        }
        itCallInfo->second.outputFlows.erase(itOut);
    }
    else
    {

        auto itFlowInput = inputFlows.find(flowId);
        auto itFlowOutput = outputFlows.find(flowId);

        if (itFlowInput == inputFlows.end()) {
            if (itFlowOutput != outputFlows.end() && !itFlowOutput->second.delayed)
                throw cRuntimeError("In outputFlows but not in inputFlows");
            else if (pk->getDestAddr() != myAddress) {
                delete pk;
                return false;
            }
        }

        if (itFlowOutput != outputFlows.end()) {
            // check delay
            if (!pk->isSelfMessage() && itFlowOutput->second.delayed) {
                scheduleAt(simTime()+itFlowOutput->second.delay,pk);
                return (false);
            }
            portDataArray[itFlowOutput->second.port].flowOccupation += itFlowOutput->second.used;
            ChangeBw val;
            val.instant = simTime();
            val.value = portDataArray[itFlowOutput->second.port].flowOccupation;
            recordOccupation(portDataArray[itFlowOutput->second.port], val);
            outputFlows.erase(itFlowOutput);
        }
        else {

            if (!pk->isSelfMessage() && !full) {
                if (pk->getDestAddr() != myAddress)
                    itFlowInput->second.endMsg = pk;
                itFlowInput->second.end = simTime();
                delayedFlows.push_back(itFlowInput->second);
            }
            else
                delete pk;
            inputFlows.erase(itFlowInput);
            return false;
        }
    }
    return true;
}

void FlowForwarding::postProc(Packet *pk, const int & destAddr, const int & destId, const int & portForward)
{
    // check if event is a release type event.
    bool releaseResources = pk->getType() == RELEASE || pk->getType() == REJECTED || pk->getType() == ENDFLOW || pk->getType() == CROUTEFLOWEND || pk->getType() == RELEASEBREAK || (pk->getType()== DATATYPE && pk->getLast()); // end a call


    if (destAddr == myAddress) {
        EV << "local delivery of packet " << pk->getName() << endl;

        if (destId == -1)
            send(pk, "localOut", 0);
        else {
            auto it = sourceIdGate.find(destId);
            if (it == sourceIdGate.end())
                throw cRuntimeError("Source id %i not registered", destId);
            send(pk, "localOut", it->second);
        }
        emit(outputIfSignal, -1); // -1: local
    }
    else {
        if (pk->getDestAddr() != -1) {
            if (portDataArray[portForward].portStatus == UP) {
                EV << "forwarding packet " << pk->getName() << " on gate index " << portForward << endl;
                pk->setHopCount(pk->getHopCount() + 1);
                emit(outputIfSignal, portForward);
                send(pk, "out", portForward);
            }
            else {
                // port forward down, delete packet.
                numDrop++;
                emit(dropSignal,numDrop);
                delete pk;
            }
        }
        else {
            numDrop++;
            emit(dropSignal,numDrop);
            delete pk;
        }
    }
    if (releaseResources)
        checkPendingList();
}

void FlowForwarding::handleMessage(cMessage *msg)
{
    //if (computeBwTimer == msg) {
    //    computeUsedBw();
    //    return;
    //}

    if (actualizeTimer == msg) {
        computeUsedBw();
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
        inverseSourceIdGate[msg->getArrivalGate()->getIndex()] = msgref->getSourceId();
        delete msg;
        return;
    }

    Base *pkbase = check_and_cast<Base *>(msg);

    if ((pkbase->getType() == STARTFLOW || pkbase->getType() == CROUTEFLOWSTART || pkbase->getType() == FLOWCHANGE || pkbase->getType() == ENDFLOW || pkbase->getType() == CROUTEFLOWEND) && simulationMode != FLOWMODE) {
            throw cRuntimeError("Packet of type flow  but simulator not in Flow mode");
    }

    if (pkbase->getType() == DATATYPE && simulationMode != PACKETMODE) {
        throw cRuntimeError("Packet of type data but simulator not in Packet mode");
    }

    if (strcmp(pkbase->getArrivalGate()->getName(), "localIn") == 0) {
        pkbase->setSrcAddr(myAddress);
        auto it = inverseSourceIdGate.find(msg->getArrivalGate()->getIndex());
        if (it == inverseSourceIdGate.end())
            throw cRuntimeError("Source in port %i not registered", msg->getArrivalGate()->getIndex());
        pkbase->setSourceId(it->second);
    }

    int destAddr = pkbase->getDestAddr();
    int srcAddr = pkbase->getSrcAddr();

    // broadcast packets pre-processing
    if (destAddr == -1) {
        procBroadcast(pkbase);
        return;
    }

    Packet *pk = check_and_cast<Packet *>(msg);
    // integrity check, pre-processing unicast packets
    // pre-processing
    if (!preProcPacket(pk))
        return;

    auto itCallInfo = callInfomap.end();
    if (pk->getCallId() > 0)
        itCallInfo = callInfomap.find(pk->getCallId());

    int portForward = -1;
    int portInput = -1;
    int destId = -1;

    // processing packets

    if (pk->getType() == RESERVE || pk->getType() == RESERVEBK) {
        if (!procReserve(pk, portForward, destId)) {
            delete pk;
            return; // nothing more to do
        }
    }
    else {
        if (pk->getCallId() > 0) {
            if (itCallInfo == callInfomap.end()) {
                delete pk;
                return;
            }
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
                destId = -1;
            }
            else if (destAddr == myAddress) {
                if (itCallInfo->second.node1 == myAddress) {
                    destId = itCallInfo->second.applicationId1;
                }
                else if (itCallInfo->second.node2 == myAddress) {
                    destId = itCallInfo->second.applicationId2;
                }
                else
                    throw cRuntimeError("Error in address %i %i registration call address %i - %i", pk->getSrcAddr(), pk->getDestAddr(), itCallInfo->second.node1, itCallInfo->second.node2);
            }
        }
        else {
            if (!getForwarPortFreeFlow(pk,portForward)) {
                delete pk;
                return; // nothing more to do
            }
            if (destAddr == myAddress) {
                destId = pk->getDestinationId();
            }
        }

        // check that the forward port is correctly set
        if (portForward == -1 && destAddr != myAddress)
            throw cRuntimeError("Error in forward port identification");

        if (pk->getType() == RELEASE || pk->getType() == REJECTED || pk->getType() == RELEASEBREAK) {// reject and release free resources.
            if (itCallInfo->second.port1 >= 0)
                portDataArray[itCallInfo->second.port1].occupation += itCallInfo->second.reserve;

            if (itCallInfo->second.port2 >= 0)
                portDataArray[itCallInfo->second.port2].occupation += itCallInfo->second.reserve;
            actualizePercentaje();

            // It is necessary to erase all flows of the same call
            // search, and delete flows
            for (auto elem : itCallInfo->second.outputFlows) {
                portDataArray[elem.port].flowOccupation += elem.used;
                ChangeBw val;
                val.instant = simTime();
                val.value = portDataArray[elem.port].flowOccupation;
                recordOccupation(portDataArray[elem.port], val);
            }
            if (!pendingFlows.empty()) {
                for (auto it = pendingFlows.begin(); it != pendingFlows.end();) {
                    //
                    if (it->identify.callId() == itCallInfo->first) {
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
        else if (pk->getType() == STARTFLOW || pk->getType() == CROUTEFLOWSTART) {
            if (!procStartFlow(pk, portForward, portInput))
                return; // nothing more to do
        }
        else if (pk->getType() == FLOWCHANGE) {
            if (!procFlowChange(pk, portForward, portInput))
                return; // nothing more to do
        }
        else if (pk->getType() == ENDFLOW || pk->getType() == CROUTEFLOWEND) {
            if (!procEndFlow(pk))
                return;  // nothing more to do
        }
        else if (pk->getType() == DATATYPE) {
            // Process data packet.
            if (itCallInfo != callInfomap.end()) {
              if (!procDataType(pk, portForward, portInput, &itCallInfo->second))
                return;  // nothing more to do
            }
            else {
                if (!procDataType(pk, portForward, portInput, nullptr))
                    return;  // nothing more to do
            }
        }
    }
    postProc(pk, destAddr, destId, portForward);
}

void FlowForwarding::receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj, cObject *details)
{
    Enter_Method_Silent();
    if (signalID == eventSignal) {
        Event * event = dynamic_cast<Event *>(obj);
        if (event)
        processLinkEvents(event);
    }
    else if (signalID == changeRoutingTableSignal) {
        ChangeRoutingTable * event = dynamic_cast<ChangeRoutingTable *>(obj);
        if (event)
            processChangeRoutes(event);
    }
}

void FlowForwarding::processChangeRoutes(ChangeRoutingTable *obj)
{
    // end flee flows

    for (auto &elem : outputFlows) {
        if (elem.second.port == obj->oldPort) {
            Packet *pkt = new Packet();
            pkt->setCallId(0);
            pkt->setType(ENDFLOW);
            pkt->setFlowId(elem.second.identify.flowId());
            pkt->setSrcAddr(elem.second.identify.src());
            pkt->setSourceId(elem.second.identify.srcId());
            pkt->setDestAddr(myAddress);
            pkt->setDestinationId(elem.second.destId);
            auto it = sourceIdGate.find(pkt->getDestinationId());

            if (it == sourceIdGate.end())
                throw cRuntimeError("Source id %i not registered", pkt->getDestinationId());
            if (hasGUI()) {
                char pkname[100];
                sprintf(pkname, "EndFlowL3-%d-to-%d-Call id#%llud-flow-%llud-dest Id %i", pkt->getSrcAddr(),
                        pkt->getDestAddr(), pkt->getCallId(), pkt->getFlowId(), pkt->getDestinationId());
                pkt->setName(pkname);
            }
            send(pkt->dup(), "out", obj->oldPort);
            portDataArray[obj->oldPort].flowOccupation += elem.second.used;
            ChangeBw val;
            val.instant = simTime();
            val.value = portDataArray[obj->oldPort].flowOccupation;
            recordOccupation(portDataArray[obj->oldPort], val);
            // move to new port
            pkt->setType(STARTFLOW);
            if (portDataArray[obj->newPort].flowOccupation > elem.second.used) {
                portDataArray[obj->newPort].flowOccupation -= elem.second.used;
                simtime_t t = SimTime(2,SIMTIME_PS);
                sendDelayed(pkt,t,"out", obj->newPort);
                elem.second.port = obj->newPort;
            }
            else {
                // TODO:
                throw cRuntimeError("Not supported yet");
            }
        }
    }
}

void FlowForwarding::finish()
{
    recordScalar("Total calls rejected",callLost);
    //recordScalar("Total calls received",callReceived);
}

