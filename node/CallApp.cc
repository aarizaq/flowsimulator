//
// Copyright (C)Author: alfonso ariza quintana, Universidad de Malaga
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//

#ifdef _MSC_VER
#pragma warning(disable:4786)
#endif

#include <iostream>
#include <fstream>
#include "CallApp.h"


uint64_t CallApp::callIdentifier = 1;
bool CallApp::residual = false;
simsignal_t CallApp::actualizationSignal = registerSignal("actualizationSignal");

Define_Module(CallApp);

CallApp::CallApp()
{
    callIdentifier = 1;
}

CallApp::~CallApp()
{
    if (dijFuzzy)
        delete dijFuzzy;
    if (generateCall)
        cancelAndDelete(generateCall);
    if (nextEvent)
        cancelAndDelete(nextEvent);
    while (!activeCalls.empty()) {
        delete activeCalls.begin()->second;
        activeCalls.erase(activeCalls.begin());
    }
    CallEvents.clear();
}

void CallApp::checkAlg() {
    callCounter++;
    int destAddress = destAddresses[intuniform(0, destAddresses.size() - 1)];

    // TODO: Include the source routing in the packet.
    dijFuzzy->runDisjoint(destAddress);

    DijkstraFuzzy::Route r1, r2, min;
    DijkstraFuzzy::FuzzyCost cost;
    dijFuzzy->getRoute(destAddress, min, cost);

    if (dijFuzzy->checkDisjoint(destAddress, r1, r2)) {
        // guarda minimo y disjuntas
        std::ofstream myfile;
        myfile.open("rutas.txt", std::ios::out | std::ios::app);
        myfile << "min :";
        for (auto elem : min)
            myfile << elem << "-";
        myfile << "\n";
        myfile << "dis1 :";
        for (auto elem : r1)
            myfile << elem << "-";
        myfile << "\n";
        myfile << "dis2 :";
        for (auto elem : r2)
            myfile << elem << "-";
        myfile << "\n";
    }
    if (!destAddresses.empty())
        scheduleAt(simTime() + callArrival->doubleValue(), generateCall);
}


void CallApp::readTopo()
{
    if (dijFuzzy == nullptr) {
        dijFuzzy = new DijkstraFuzzy;
    }
    else
        dijFuzzy->clearAll();

    dijFuzzy->setRoot(getParentModule()->par("address"));

    std::vector<std::string> nedTypes;
    nedTypes.push_back(getParentModule()->getNedTypeName());

    cTopology topo("topo");
    topo.extractByNedTypeName(nedTypes);

    /* DijkstraFuzzy test;

     // 0-1
     test.addEdge(0, 1, 1, 1, 1);
     test.addEdge(1, 0, 1, 1, 1);

     // 0-4
     test.addEdge(0, 4, 7, 8, 11);
     test.addEdge(4, 0, 7, 8, 11);

     // 0-5
     test.addEdge(0, 5, 1, 1, 1);
     test.addEdge(5, 0, 1, 1, 1);

     // 1-2
     test.addEdge(1, 2, 1, 2, 3);
     test.addEdge(2, 1, 1, 2, 3);

     // 1-5
     test.addEdge(1, 5, 1, 1, 1);
     test.addEdge(5, 1, 1, 1, 1);
     // 1-6
     test.addEdge(1, 6, 1, 2, 3);
     test.addEdge(6, 1, 1, 2, 3);

     // 2-3
     test.addEdge(2, 3, 1, 2, 3);
     test.addEdge(3, 2, 1, 2, 3);

     // 2-4
     test.addEdge(2, 4, 1, 1, 1);
     test.addEdge(4, 2, 1, 1, 1);

     // 3-6
     test.addEdge(3, 6, 1, 2, 5);
     test.addEdge(6, 3, 1, 2, 5);

     // 3-7
     test.addEdge(3, 7, 1, 1, 1);
     test.addEdge(7, 3, 1, 1, 1);

     // 7-4
     test.addEdge(7, 4, 1, 2, 3);
     test.addEdge(4, 7, 1, 2, 3);

     // 5-6
     test.addEdge(5, 6, 2, 4, 6);
     test.addEdge(6, 5, 2, 4, 6);

     // 6-7
     test.addEdge(7, 6, 3, 4, 6);
     test.addEdge(6, 7, 3, 4, 6);

     test.setRoot(0);
     test.runDisjoint(7);
     */
    Dijkstra dj;
    for (int i = 0; i < topo.getNumNodes(); i++) {
        cTopology::Node *node = topo.getNode(i);
        int address = node->getModule()->par("address");
        for (int j = 0; j < node->getNumOutLinks(); j++) {

            int addressAux = node->getLinkOut(j)->getRemoteNode()->getModule()->par("address");

            double minResidual = node->getLinkOut(j)->getLocalGate()->getTransmissionChannel()->getNominalDatarate();;
            double meanResidual = node->getLinkOut(j)->getLocalGate()->getTransmissionChannel()->getNominalDatarate();;
            double maxResidual = node->getLinkOut(j)->getLocalGate()->getTransmissionChannel()->getNominalDatarate();;

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
                minResidual = 1;
                meanResidual = 1;
                maxResidual = 1;
            }
            dijFuzzy->addEdge(address, addressAux, minResidual, meanResidual, maxResidual);
            dj.addEdge(address, addressAux, 1, 10000);
        }
    }
    NodePairs links;
    dj.discoverAllPartitionedLinks(links);

}

void CallApp::rescheduleEvent()
{
    if (CallEvents.empty() && FlowEvents.empty()) {
        if (nextEvent->isScheduled())
            cancelEvent(nextEvent);
        return;
    }

    simtime_t min1 = SimTime::getMaxTime();
    simtime_t min2 = SimTime::getMaxTime();

    if (!CallEvents.empty())
        min1 = CallEvents.begin()->first;
    if (!FlowEvents.empty())
        min2 = FlowEvents.begin()->first;
    simtime_t min = min1<min2?min1:min2;

    if (min<simTime())
        throw cRuntimeError("Planing error");

    if (!nextEvent->isScheduled()) {
        scheduleAt(min, nextEvent);
    }
    else if (min != nextEvent->getArrivalTime()) {
        cancelEvent(nextEvent);
        scheduleAt(min, nextEvent);
    }
}

void CallApp::bytesTraceRec(const CallInfo *callInfo)
{

    if (!trace) return;

    double brec = callInfo->recBandwith * SIMTIME_DBL(simTime() - callInfo->startOnRec);
    double tm = SIMTIME_DBL(simTime() - callInfo->startOnRec);

    char fileName[100];
    memset(fileName,0,sizeof(fileName));
    sprintf(fileName,"Rec-%i",myAddress);
    std::ofstream myfile;
    myfile.open(fileName, std::ios::out | std::ios::app);

    myfile << simTime().dbl() << " " << callInfo->startOnRec << " ";
    myfile << "Dest " << callInfo->dest  << " CalId :" << callInfo->callId <<"   bw: " << callInfo->usedBandwith << " t " << tm << " brec " << brec << "\n";
}

void CallApp::bytesTraceSend(const CallInfo *callInfo)
{

    if (!trace) return;

    double bsend = callInfo->usedBandwith * SIMTIME_DBL(simTime() - callInfo->startOn);

    char fileName[100];
    memset(fileName,0,sizeof(fileName));
    sprintf(fileName,"Send-%i",myAddress);
    std::ofstream myfile;
    myfile.open(fileName, std::ios::out | std::ios::app);

    double tm = SIMTIME_DBL(simTime() - callInfo->startOn);

    myfile << simTime().dbl() << " " << callInfo->startOn << " ";
    myfile << "Dest " << callInfo->dest  << " CalId :" << callInfo->callId <<"   bw: " << callInfo->usedBandwith << " t " << tm << " bsend " << bsend << "\n";
}


void CallApp::procNextEvent()
{
    char pkname[100];
    memset(pkname,0,sizeof(pkname));
    double delayAux;
    while (!CallEvents.empty() && CallEvents.begin()->first <= simTime()) {
        auto it = CallEvents.begin();
        CallInfo *callInfo = it->second;
        CallEvents.erase(it);

        Packet *pkFlow = new Packet();

        pkFlow->setDestAddr(callInfo->dest);
        pkFlow->setCallId(callInfo->callId);
        pkFlow->setSourceId(par("sourceId").longValue());
        pkFlow->setDestinationId(callInfo->sourceId);
        pkFlow->setSrcAddr(myAddress);

        if (callInfo->flowData.empty()) {
            if (callInfo->state == ON) {
                bytesTraceSend(callInfo);
                double bsend = callInfo->usedBandwith * SIMTIME_DBL(simTime() - callInfo->startOn);
                callInfo->acumulateSend += (bsend/1000.0);
                EV << "Call Id :" << callInfo->callId << "Flow Id :" << callInfo->flowId <<
                        " Time in on :" << callInfo->startOn << "Bsend :" << bsend;
                callInfo->state = OFF;
                pkFlow->setType(ENDFLOW);
                pkFlow->setFlowId(callInfo->flowId);
                delayAux = TimeOff->doubleValue();
                pkFlow->setReserve(callInfo->usedBandwith);

                sprintf(pkname, "FlowOff-%d-to-%d-CallId#%lud-FlowId#%lud-Sid-%d",
                                                myAddress, pkFlow->getDestAddr(),
                                                pkFlow->getCallId(),pkFlow->getFlowId(), this->getIndex());
                pkFlow->setName(pkname);

            }
            else if (callInfo->state == OFF) {
                callInfo->state = ON;
                callInfo->flowId++;
                pkFlow->setFlowId(callInfo->flowId);
                callInfo->usedBandwith = (uint64_t) usedBandwith->doubleValue();
                pkFlow->setReserve(callInfo->usedBandwith);
                pkFlow->setType(STARTFLOW);
                delayAux = TimeOn->doubleValue();
                callInfo->startOn = simTime();

                sprintf(pkname, "FlowOn-%d-to-%d-CallId#%lud-FlowId#%lud-Sid-%d",
                        myAddress, pkFlow->getDestAddr(),
                        pkFlow->getCallId(), pkFlow->getFlowId(), this->getIndex());
                pkFlow->setName(pkname);
            }
            if (pkFlow->getDestAddr() == myAddress)
                throw cRuntimeError("Destination address erroneous");
            send(pkFlow, "out");
            CallEvents.insert(std::make_pair(simTime() + delayAux, callInfo));
        }
        else {
            // multi flow system
            throw cRuntimeError("Disabled");
            for (auto &elem : callInfo->flowData) {
                if (elem.nextEvent > simTime())
                    continue;
                if (elem.state == ON) {
                    bytesTraceRec(callInfo);
                    callInfo->acumulateSend += ((elem.usedBandwith * SIMTIME_DBL(simTime() - elem.startOn))/1000.0);
                    pkFlow->setFlowId(elem.flowId);
                    elem.state = OFF;
                    pkFlow->setType(ENDFLOW);
                    pkFlow->setReserve(elem.usedBandwith);
                    delayAux = TimeOff->doubleValue();
                    sprintf(pkname, "FlowOff-%d-to-%d-CallId#%lud- FlowId#%lud -Sid-%d",
                            myAddress, pkFlow->getDestAddr(),
                            pkFlow->getCallId(),pkFlow->getFlowId(), this->getIndex());
                    pkFlow->setName(pkname);

                }
                else if (elem.state == OFF) {
                    elem.state = ON;
                    callInfo->flowId++;
                    elem.flowId = callInfo->flowId;
                    pkFlow->setFlowId(callInfo->flowId);
                    elem.usedBandwith = (uint64_t) usedBandwith->doubleValue();
                    pkFlow->setReserve(elem.usedBandwith );
                    pkFlow->setType(STARTFLOW);
                    delayAux = TimeOn->doubleValue();
                    callInfo->startOn = simTime();
                    sprintf(pkname, "FlowOn-%d-to-%d-CallId#%lud- FlowId#%lud -Sid-%d",
                            myAddress, pkFlow->getDestAddr(),
                            pkFlow->getCallId(),pkFlow->getFlowId(), this->getIndex());
                    pkFlow->setName(pkname);
                }

                elem.nextEvent = simTime() + delayAux;
                if (pkFlow->getDestAddr() == myAddress)
                    throw cRuntimeError("Destination address erroneous");
                send(pkFlow, "out");
                CallEvents.insert(std::make_pair(simTime() + delayAux, callInfo));
            }
        }
    }

    while (!FlowEvents.empty() && FlowEvents.begin()->first <= simTime()) {
        auto it = FlowEvents.begin();
        FlowEvent *flowEvent = it->second;
        FlowEvents.erase(it);

        Packet *pkFlow = new Packet();
        pkFlow->setReserve(flowEvent->usedBandwith);
        pkFlow->setDestAddr(flowEvent->dest);
        pkFlow->setCallId(0);
        pkFlow->setSourceId(par("sourceId").longValue());
        pkFlow->setDestinationId(flowEvent->destId);
        pkFlow->setSrcAddr(myAddress);
        //callInfo->acumulateSend += (callInfo->usedBandwith
        //        * callInfo->startOn.dbl());

        pkFlow->setType(ENDFLOW);
        pkFlow->setFlowId(flowEvent->flowId);
        sprintf(pkname, "FlowOff-%d-to-%d-CallId#%lud- FlowId#%lud -Sid-%d",
                myAddress, pkFlow->getDestAddr(),
                pkFlow->getCallId(),pkFlow->getFlowId(), this->getIndex());
        pkFlow->setName(pkname);
        if (pkFlow->getDestAddr() == myAddress)
            throw cRuntimeError("Destination address erroneous");
        send(pkFlow, "out");
        double bsend = flowEvent->usedBandwith * SIMTIME_DBL(simTime() - flowEvent->startOn);
        auto itStat = sendBytes.find(flowEvent->dest);

        if (itStat == sendBytes.end())
            sendBytes[flowEvent->dest] = (bsend/1000.0);
        else
            itStat->second = (bsend/1000.0);
        delete flowEvent;
    }
}

void CallApp::newCall() {
    if (destAddresses.empty())
        return;

    char pkname[100];
// Sending packet
    callCounter++;
    if (check) {
        checkAlg();
        return;
    }
    int destAddress = destAddresses[intuniform(0, destAddresses.size() - 1)];

    sprintf(pkname, "CallReserve-%d-to-%d-Call id#%lud-Did-%ld", myAddress,
            destAddress, callIdentifier, par("sourceId").longValue());
    EV << "generating packet " << pkname << endl;

    Packet *pk = new Packet(pkname);
    pk->setType(RESERVE);
    pk->setReserve(callReserve->doubleValue());
    pk->setSrcAddr(myAddress);
    pk->setDestAddr(destAddress);
    pk->setCallId(callIdentifier);
    callIdentifier++;
    if (callIdentifier == 0) // 0 is reserved for flows not assigned to a call.
        callIdentifier++;
    pk->setSourceId(par("sourceId").longValue());
    pk->setDestinationId(par("destinationId").longValue());

    if (rType == DISJOINT) {
        DijkstraFuzzy::Route r1, r2, min;
        dijFuzzy->setRoot(getParentModule()->par("address"));
        dijFuzzy->runDisjoint(destAddress);
        //dijFuzzy->getRoute(destAddress, min, cost);
        if (dijFuzzy->checkDisjoint(destAddress, r1, r2)) {
            DijkstraFuzzy::FuzzyCost costr1, costr2;
            dijFuzzy->getCostPath(r1, costr1);
            dijFuzzy->getCostPath(r2, costr2);
            double total = costr1.exp() + costr2.exp();

            DijkstraFuzzy::Route *r =
                    uniform(0, total) < costr1.exp() ? &r2 : &r1;
            pk->setRouteArraySize(r->size());
            for (unsigned int i = 0; i < r->size(); i++) {
                pk->setRoute(i, (*r)[i]);
            }
        }
    }
    else if (rType == SOURCEROUTING) {
        DijkstraFuzzy::Route min;
        DijkstraFuzzy::FuzzyCost cost;
        dijFuzzy->setRoot(getParentModule()->par("address"));
        if (!dijFuzzy->getRoute(destAddress, min, cost))
            dijFuzzy->run();
        //dijFuzzy->getRoute(destAddress, min, cost);
        if (dijFuzzy->getRoute(destAddress, min, cost)) {
            pk->setRouteArraySize(min.size());
            for (unsigned int i = 0; i < min.size(); i++) {
                pk->setRoute(i, min[i]);
            }
        }
    }
    else if (rType == BACKUPROUTE) {
        // TODO: backup mode Se deben enviar dos paquetes, uno por cada ruta
        DijkstraFuzzy::Route r1, r2, min;
        dijFuzzy->setRoot(getParentModule()->par("address"));
        dijFuzzy->runDisjoint(destAddress);
        //dijFuzzy->getRoute(destAddress, min, cost);
        if (dijFuzzy->checkDisjoint(destAddress, r1, r2)) {
            DijkstraFuzzy::FuzzyCost costr1, costr2;
            dijFuzzy->getCostPath(r1, costr1);
            dijFuzzy->getCostPath(r2, costr2);

            // new call id for backup route
            Packet *pk2 = pk->dup();
            pk2->setCallId(callIdentifier);
            callIdentifier++;

            pk2->setCallIdBk(pk->getCallId());
            pk->setCallIdBk(pk2->getCallId());

            if (pk2->getCallId() == 10)
                printf("");

            pk2->setRouteArraySize(r1.size());
            for (unsigned int i = 0; i < r1.size(); i++) {
                pk2->setRoute(i, r1[i]);
            }

            pk->setRouteArraySize(r2.size());
            for (unsigned int i = 0; i < r2.size(); i++) {
                pk->setRoute(i, r2[i]);
            }

            if (costr1 < costr2)
                pk2->setType(RESERVEBK);
            else {
                pk->setType(RESERVEBK);
                Packet *auxMsg = pk2;
                pk2 = pk;
                pk = auxMsg;
            }
            // send before the best
            send(pk2, "out");
        }
    }


// TODO : recall timer,
    send(pk, "out");
}

void CallApp::newFlow() {

    if (destAddresses.empty())
        return;
    char pkname[100];

// generate the next flow
    int destAddress = destAddresses[intuniform(0, destAddresses.size() - 1)];
    Packet *pkFlow = new Packet();

    pkFlow->setReserve(flowUsedBandwith->doubleValue());
    pkFlow->setDestAddr(destAddress);
    pkFlow->setCallId(0);
    pkFlow->setSourceId(par("sourceId").longValue());
    pkFlow->setDestinationId(par("destinationId").longValue());
    pkFlow->setSrcAddr(myAddress);
    pkFlow->setFlowId(flowIdentifier++);
    pkFlow->setType(STARTFLOW);

    sprintf(pkname, "NewFlow-%d-to-%d-Call Id #%lud- Flow Id #%lud -Did-%ld",
            myAddress, destAddress, pkFlow->getCallId(), pkFlow->getFlowId(),
            par("sourceId").longValue());
    pkFlow->setName(pkname);
    if (pkFlow->getDestAddr() == myAddress)
        throw cRuntimeError("Destination address erroneous");
    send(pkFlow, "out");

    if (hasGUI())
        getParentModule()->bubble("Generating flow..");

    FlowEvent *event = new FlowEvent;
    event->dest = destAddress;
    event->destId = pkFlow->getDestinationId();
    event->flowId = pkFlow->getFlowId();
    event->usedBandwith = pkFlow->getReserve();
    event->startOn = simTime();
    FlowEvents.insert(std::make_pair(simTime() + flowDuration->doubleValue(), event));
}

void CallApp::newReserve(Packet *pk)
{

    CallInfo *callInfo = nullptr;
    auto itAux = activeCalls.find(pk->getCallId());

    if (itAux != activeCalls.end())
        throw cRuntimeError("Call id presents in the system");

    auto itAux2 = backupCalls.find(pk->getCallId());
    if (itAux2 != backupCalls.end())
        throw cRuntimeError("Call id presents in the system");

    bool activeCall = true;
    if (itAux == activeCalls.end() &&  pk->getCallIdBk() == 0) {
        callInfo = new CallInfo();
        callInfo->dest = pk->getSrcAddr();
        callInfo->sourceId = pk->getSourceId();
        callInfo->callId = pk->getCallId();
        callInfo->callIdBk = pk->getCallIdBk();
        if (callInfo->dest == myAddress)
            throw cRuntimeError("Address destination Error");
        activeCalls.insert(std::make_pair(callInfo->callId, callInfo));
        callReceived++;
    }
    else if (itAux == activeCalls.end() &&  pk->getCallIdBk() != 0) {
        // check if the other call exist
        auto itAux2 = activeCalls.find(pk->getCallIdBk());
        if (itAux2 != activeCalls.end()) { // store this like backup
            // sanity check
            auto itAuxSanity = backupCalls.find(pk->getCallIdBk());
            if (itAuxSanity != backupCalls.end())
                throw cRuntimeError("Call id presents in the system, bk list");
            CallInfo * callInfo = itAux2->second;
            callInfo->callIdBk = pk->getCallId();
            if (callInfo->dest == myAddress)
                throw cRuntimeError("Address destination Error");
            backupCalls.insert(std::make_pair(pk->getCallId(), callInfo));
            activeCall = false;
        }
        else {
            // there isn't any available, create a new information
            callInfo = new CallInfo();
            callInfo->dest = pk->getSrcAddr();
            callInfo->sourceId = pk->getSourceId();
            callInfo->callId = pk->getCallId();
            callInfo->callIdBk = 0; //
            if (callInfo->dest == myAddress)
                throw cRuntimeError("Address destination Error");
            activeCalls.insert(std::make_pair(callInfo->callId, callInfo));
            callReceived++;
        }
    }

    // check in the list of calls

    if (activeCall) {
        if (generateFlow) {
            callInfo->state = OFF;
            callInfo->reservedBandwith = pk->getReserve();
            callInfo->usedBandwith = (uint64_t) usedBandwith->doubleValue();
            CallEvents.insert(std::make_pair(simTime() + TimeOff->doubleValue(), callInfo));
        }
        else
            callInfo->state = PASSIVE;
    }
    // return acceptance packet
    char pkname[100];
    pk->setType(ACEPTED);
    pk->setDestAddr(pk->getSrcAddr());
    pk->setSrcAddr(myAddress);
    sprintf(pkname, "PkAccepted-%d-to-%d-#%lud-Sid-%d", myAddress,
            pk->getDestAddr(), pk->getCallId(), this->getIndex());
    pk->setName(pkname);
    pk->setDestinationId(pk->getSourceId());
    pk->setSourceId(par("sourceId").longValue());
    if (pk->getDestAddr() == myAddress)
        throw cRuntimeError("Destination address erroneous");
    send(pk, "out");
}

void CallApp::newAccepted(Packet *pk) {

    // prepare the release

    CallInfo *callInfo = nullptr;

    auto itAux = activeCalls.find(pk->getCallId());

    if (itAux != activeCalls.end())
        throw cRuntimeError("Call id presents in the system");

    auto itAux2 = backupCalls.find(pk->getCallId());
    if (itAux2 != backupCalls.end())
        throw cRuntimeError("Call id presents in the system");


    bool activeCall = true;
    // check if exist the backup
    if (pk->getCallIdBk() != 0) {
        auto itAux3 = activeCalls.find(pk->getCallIdBk());
        if (itAux3 != activeCalls.end()) {
            callInfo = itAux3->second;
            callInfo->callIdBk = pk->getCallId();
            if (callInfo->dest == myAddress)
                throw cRuntimeError("Address destination Error");
            backupCalls.insert(std::make_pair(callInfo->callIdBk, callInfo));
            activeCall = false;
        }
        else {// check alternative list
            auto itAux3 = backupCalls.find(pk->getCallIdBk());
            if (itAux3 != backupCalls.end()) {
                callInfo = itAux3->second;
            }
            else {
                callInfo = new CallInfo;
                callInfo->dest = pk->getSrcAddr();
                callInfo->callId = pk->getCallId();
                callInfo->sourceId = pk->getSourceId();
            }
            if (callInfo->dest == myAddress)
                throw cRuntimeError("Address destination Error");
            activeCalls.insert(std::make_pair(callInfo->callId, callInfo));
        }
    }
    else {
        callInfo = new CallInfo;
        callInfo->dest = pk->getSrcAddr();
        callInfo->callId = pk->getCallId();
        callInfo->sourceId = pk->getSourceId();
        if (callInfo->dest == myAddress)
            throw cRuntimeError("Address destination Error");
        activeCalls.insert(std::make_pair(callInfo->callId, callInfo));
    }


    if (activeCall) {
        char pkname[100];
        if (generateFlow) {
            callInfo->state = ON;
            callInfo->flowId++;
            callInfo->reservedBandwith = pk->getReserve();
            callInfo->usedBandwith = (uint64_t) usedBandwith->doubleValue();
            simtime_t delayAux = TimeOn->doubleValue();
            callInfo->startOn = simTime();
            Packet *pkFlow = new Packet();
            pkFlow->setSourceId(par("sourceId").longValue());
            pkFlow->setDestinationId(callInfo->sourceId);
            pkFlow->setSrcAddr(myAddress);
            pkFlow->setDestAddr(callInfo->dest);
            pkFlow->setType(STARTFLOW);
            pkFlow->setReserve(callInfo->usedBandwith);
            pkFlow->setFlowId(callInfo->flowId);
            pkFlow->setCallId(callInfo->callId);
            sprintf(pkname, "FlowOn-%d-to-%d-#%lud-Sid-%d-FlowId-%d", myAddress,
                    pkFlow->getDestAddr(), pkFlow->getCallId(),
                    this->getIndex(), callInfo->flowId);
            pkFlow->setName(pkname);
            CallEvents.insert(std::make_pair(simTime() + delayAux, callInfo));
            if (pkFlow->getDestAddr() == myAddress)
                throw cRuntimeError("Destination address erroneous");
            send(pkFlow, "out");
        } else
            callInfo->state = PASSIVE;
        // prepare release event
        callEstabilized++;

        pk->setType(RELEASE);
        pk->setDestAddr(pk->getSrcAddr());
        pk->setSrcAddr(myAddress);
        pk->setDestinationId(pk->getSourceId());
        pk->setSourceId(par("sourceId").longValue());

        sprintf(pkname, "Pkrelease-%d-to-%d-#%lud-Sid-%d", myAddress,
                pk->getDestAddr(), pk->getCallId(), this->getIndex());
        pk->setName(pkname);
        scheduleAt(simTime() + callDuration->doubleValue(), pk);
    }
    else
    {
        delete pk;
    }
}

void CallApp::release(Packet *pk) {

    // Handle incoming packet
    auto it = activeCalls.find(pk->getCallId());
    auto it2 = backupCalls.find(pk->getCallId());
    if (it == activeCalls.end() && it2 == backupCalls.end()) {
        if (!pk->isSelfMessage()) // si se ha libreador por rotura debería haber llegado a la otra parte el relese con lo cual no debe mandar el mensaje de release otra vez
            throw cRuntimeError("Call Id not found in any list");
        else {
            if (pk->getCallIdBk() == 0) {
                delete pk;
                return;
            }
            auto it = activeCalls.find(pk->getCallIdBk());
            if (it == activeCalls.end()) {
                delete pk;
                return;
            }
            else {
             // El backup pasó a principal, se debe eliminar ahora el backup
                pk->setCallId(pk->getCallIdBk());
                pk->setCallIdBk(0);
            }
        }
    }

    if (it != activeCalls.end()) {
        if (pk->isSelfMessage() && it->second->callIdBk != 0)
        {
            // release backup route
            Packet * pkt = pk->dup();
            pkt->setCallId(it->second->callIdBk);
            auto itAux =  backupCalls.find(it->second->callIdBk);
            backupCalls.erase(itAux);
            if (pkt->getDestAddr() == myAddress)
                throw cRuntimeError("Destination address erroneous");
            send(pkt, "out");
            it->second->callIdBk = 0;
        }
        // record statistics
        CallInfo *callInfo = it->second;
        for (auto it = CallEvents.begin(); it != CallEvents.end();) {
            if (it->second->callId == pk->getCallId())
                CallEvents.erase(it++);
            else
                ++it;
        }

       // Accumulate the flows in curse
        if (callInfo->state == ON) {
            bytesTraceSend(callInfo);
            double bsend = callInfo->usedBandwith
                    * SIMTIME_DBL(simTime() - callInfo->startOn);
            callInfo->acumulateSend += ((bsend) / 1000.0);
        }

        if (callInfo->stateRec == ON) {
            bytesTraceRec(callInfo);
            double brec = callInfo->recBandwith
                    * SIMTIME_DBL(simTime() - callInfo->startOnRec);
            callInfo->acumulateRec += (brec / 1000.0);
        }

        // record the statistics
        auto itAccSend = sendBytes.find(callInfo->dest);
        auto itAccRec = receivedBytes.find(callInfo->dest);

        if (itAccSend == sendBytes.end())
            sendBytes[it->second->dest] = callInfo->acumulateSend;
        else
            itAccSend->second += callInfo->acumulateSend;

        if (itAccRec == receivedBytes.end())
            receivedBytes[callInfo->dest] = callInfo->acumulateRec;
        else
            itAccRec->second += callInfo->acumulateRec;

        if (callInfo->callIdBk != 0) // change to backup route
        {
            auto itAux =  backupCalls.find(it->second->callIdBk);
            backupCalls.erase(itAux);
            // change all to backup
            activeCalls.erase(it);
            uint64_t callidBk = callInfo->callIdBk;
            callInfo->callIdBk = 0;
            callInfo->callId = callidBk;
            if (callInfo->dest == myAddress)
                throw cRuntimeError("Address destination Error");
            activeCalls.insert(std::make_pair(callInfo->callId,callInfo));
            callInfo->state = OFF;
            CallEvents.insert(std::make_pair(simTime()+TimeOff->doubleValue(),callInfo));
        }
        else {
            delete callInfo;
            activeCalls.erase(it);
        }
    }
    else if (it2 != backupCalls.end())
    {
            // erase backup
            it2->second->callIdBk = 0;
            backupCalls.erase(it2);
            // Nothing more to do
    }

// if self message send the release message to the other node
    if (pk->isSelfMessage()) {
        if (pk->getDestAddr() == myAddress)
            throw cRuntimeError("Destination address erroneous");
        send(pk, "out");
    }
    else {
        // relese receive nothing more to do
        delete pk;
        if (hasGUI()) {
            getParentModule()->getDisplayString().setTagArg("i", 1, "green");
            getParentModule()->bubble("Arrived!");
        }
    }
}

void CallApp::procFlowPk(Packet *pk) {
    FlowIdentification flowId;
    flowId.callId() = pk->getCallId();
    flowId.flowId() = pk->getFlowId();
    flowId.src() = pk->getSrcAddr();
    flowId.srcId() = pk->getSourceId();


    // auto it = flowStatistics.find(flowId);
//  if (it == flowStatistics.end())
//      throw cRuntimeError("Call id is not registered");

    if (pk->getType() == ENDFLOW) {
        if (flowId.callId() > 0) {
            auto itAux = activeCalls.find(flowId.callId());
            if (itAux == activeCalls.end())
                throw cRuntimeError("Call id not found but flow recieved");
            CallInfo * callInfo = itAux->second;
            bytesTraceRec(callInfo);

            double brec = callInfo->recBandwith
                    * SIMTIME_DBL(simTime() - callInfo->startOnRec);
            callInfo->acumulateRec += (brec/1000.0);
            callInfo->stateRec = OFF;

            EV << "Rec Call Id :" << callInfo->callId << "Flow Id :"
                      << callInfo->flowId << " Time in on :"
                      << (simTime() - callInfo->startOnRec) << "Brec :" << brec;

        }
        else {
            auto it = flowStatistics.find(flowId);
            if (it == flowStatistics.end())
                throw cRuntimeError("Flow is not registered");
            double brec = it->second.used * SIMTIME_DBL(simTime() - it->second.startOnRec);

            auto itStat = receivedBytes.find(flowId.src());
            if (itStat == receivedBytes.end())
                receivedBytes[flowId.src()] = (brec/1000.0);
            else
                itStat->second = (brec/1000.0);
            // erase flow
            flowStatistics.erase(it);
        }
    }
    else if (pk->getType() == STARTFLOW) {
        if (flowId.callId() > 0) {
            CallInfo * callInfo = nullptr;
            auto itAux = activeCalls.find(flowId.callId());
            if (itAux == activeCalls.end()) {
                // check backup
                itAux = backupCalls.find(flowId.callId());
                if (itAux == backupCalls.end())
                    throw cRuntimeError("Call Id not found in any list %i",flowId.callId());
                // change to backup route
                callInfo = itAux->second;
                uint64_t callId = callInfo->callId;
                uint64_t calIdbk = callInfo->callIdBk;
                callInfo->callId = calIdbk;
                callInfo->callIdBk = callId;
                backupCalls.erase(itAux);
                itAux = activeCalls.find(callId);
                if (pk->getCallId() == 10)
                    printf("");
                if (itAux != activeCalls.end())
                    activeCalls.erase(itAux);

                if (callInfo->dest == myAddress)
                    throw cRuntimeError("Address destination Error");

                if (callId != 0)
                    backupCalls.insert(std::make_pair(callId,callInfo));
                activeCalls.insert(std::make_pair(calIdbk,callInfo));

                //
                if (callInfo->state == ON) {
                    for (auto it = CallEvents.begin(); it != CallEvents.end();) {
                        if (it->second->callId == callId)
                            CallEvents.erase(it++);
                        else
                            ++it;
                    }
                    bytesTraceSend(callInfo);
                    callInfo->acumulateSend += ((callInfo->recBandwith
                            * SIMTIME_DBL(simTime() - callInfo->startOn))/1000);
                    // send off in the o
                    callInfo->state = OFF;
                    Packet *pkFlow = new Packet();
                    pkFlow->setDestAddr(callInfo->dest);
                    pkFlow->setCallId(callId);
                    pkFlow->setSourceId(par("sourceId").longValue());
                    pkFlow->setDestinationId(callInfo->sourceId);
                    pkFlow->setType(ENDFLOW);
                    pkFlow->setFlowId(callInfo->flowId);
                    pkFlow->setReserve(callInfo->usedBandwith);
                    char pkname[60];

                    sprintf(pkname,"FlowOff-%d-to-%d-CallId#%lud-FlowId#%lud-Sid-%d", myAddress, pkFlow->getDestAddr(),  pkFlow->getCallId(), pkFlow->getFlowId(), this->getIndex());
                    pkFlow->setName(pkname);
                    if (pkFlow->getDestAddr() == myAddress)
                        throw cRuntimeError("Destination address erroneous");
                    send(pkFlow, "out");
                    CallEvents.insert(std::make_pair(simTime()+TimeOff->doubleValue(), callInfo));
                }
            }
            else
                callInfo = itAux->second;

            callInfo->recBandwith = (uint64_t) pk->getReserve();
            callInfo->startOnRec = simTime();
            callInfo->stateRec = ON;
        }
        else {
            // register the flow;
            auto it = flowStatistics.find(flowId);
            if (it != flowStatistics.end())
                throw cRuntimeError("FLow is already registered");

            FlowStat st;
            st.startOnRec = simTime();
            st.used = (uint64_t) pk->getReserve();
            flowStatistics.insert(std::make_pair(flowId,st));
        }
    }
    else if (pk->getType() == FLOWCHANGE) {
        if (flowId.callId() > 0) {
            auto itAux = activeCalls.find(flowId.callId());
            CallInfo * callInfo = itAux->second;
            if (callInfo->stateRec == ON) {
                bytesTraceRec(callInfo);
                callInfo->acumulateRec += ((callInfo->recBandwith
                        * SIMTIME_DBL(simTime() - callInfo->startOnRec))/1000);
            }
            callInfo->stateRec = ON;
            callInfo->recBandwith = (uint64_t) pk->getReserve();
            callInfo->startOnRec = simTime();
        }
        else {
            // register the flow;
            auto it = flowStatistics.find(flowId);
            if (it == flowStatistics.end()) {
                // create
                FlowStat st;
                st.startOnRec = simTime();
                st.used = (uint64_t) pk->getReserve();
                flowStatistics.insert(std::make_pair(flowId,st));
            }
            else {
                double brec = it->second.used * SIMTIME_DBL(simTime() - it->second.startOnRec);
                auto itStat = receivedBytes.find(flowId.src());
                if (itStat == receivedBytes.end())
                    receivedBytes[flowId.src()] = (brec/1000.0);
                else
                    itStat->second = (brec/1000.0);
                it->second.startOnRec = simTime();
                it->second.used = (uint64_t) pk->getReserve();
            }
        }

    }
    delete pk;
}

void CallApp::initialize()
{
    myAddress = par("address");
    callReserve = &par("callReserve");
    callArrival = &par("callArrival");  // volatile parameter
    callDuration = &par("callDuration");  // volatile parameter

    generateFlow = par("generateFlow");
    TimeOn = &par("TimeOn");
    TimeOff = &par("TimeOff");
    usedBandwith = &par("usedBandwith");
    flowArrival = &par("flowArrival");
    flowDuration = &par("flowDuration");
    flowUsedBandwith = &par("flowUsedBandwith");

    callCounter = 0;

    WATCH(callCounter);
    WATCH(callReceived);
    WATCH(myAddress);
    WATCH_MAP(receivedBytes);
    WATCH_MAP(sendBytes);

    const char *destAddressesPar = par("destAddresses");
    cStringTokenizer tokenizer(destAddressesPar);
    const char *token;
    while ((token = tokenizer.nextToken()) != NULL)
    {
        int destAddr = atoi(token);
        if (destAddr != myAddress)
            destAddresses.push_back(atoi(token));
    }

    generateCall = new cMessage("nextCall");
    if (!destAddresses.empty())
        scheduleAt(callArrival->doubleValue(), generateCall);

    if (par("independentFlows").boolValue() && !destAddresses.empty()) {
        nextFlow = new cMessage();
        scheduleAt(flowArrival->doubleValue(), nextFlow);
    }
    if (strcmp(par("RoutingType").stringValue(),"HopByHop") ==0)
        rType = HOPBYHOP;
    else if (strcmp(par("RoutingType").stringValue(),"SourceRouting") ==0)
        rType = SOURCEROUTING;
    else if (strcmp(par("RoutingType").stringValue(),"Disjoint") ==0)
        rType = DISJOINT;
    else if (strcmp(par("RoutingType").stringValue(),"BackupRouting") ==0)
        rType = BACKUPROUTE;

    nextEvent = new cMessage("NewEvent");
    readTopo();
    RegisterMsg * msg = new RegisterMsg();
    msg->setSourceId(par("sourceId").longValue());
    send(msg, "out");

    // register in the root module to receive the actualization of all nodes.
    cSimulation::getActiveSimulation()->getSystemModule()->subscribe(actualizationSignal,this);
}

void CallApp::handleMessage(cMessage *msg)
{

    if (!msg->isPacket()) {
        if (msg == generateCall) {
            newCall();
            if (!destAddresses.empty())
                scheduleAt(simTime() + callArrival->doubleValue(), generateCall);
            if (hasGUI())
                getParentModule()->bubble("Generating call..");
        }
        else if (msg == nextFlow) {
            newFlow();
            if (!destAddresses.empty())
                scheduleAt(simTime() + flowArrival->doubleValue(), nextFlow);
        }
        else if (msg == nextEvent) {
            procNextEvent();
        }
        else
            delete msg;

        rescheduleEvent();
        return;

    }

    Base *pkaux = dynamic_cast<Base *>(msg);
    if (pkaux == nullptr) {
        delete msg;
        return;
    }

    if (pkaux->getType() == ACTUALIZE) {
        Actualize *pk = dynamic_cast<Actualize *>(msg);
        for (unsigned int i = 0; i < pk->getLinkDataArraySize(); i++) {
            double residual = pk->getLinkData(i).residual;
            double cost = 1 / residual;

            if (residual > 1e20)
                dijFuzzy->deleteEdge(pk->getSrcAddr(), pk->getLinkData(i).node);
            else {
                double cost1 = 1 / (residual + 100);
                double cost2 = 1 / (residual - 100);
                if (cost2 < 0)
                    cost2 = 1e30;
                dijFuzzy->addEdge(pk->getSrcAddr(), pk->getLinkData(i).node, cost1, cost, cost2);
            }
        }
        delete msg;
        return;
    }

    Packet *pk = dynamic_cast<Packet *>(msg);
    if (pk == nullptr)
        throw cRuntimeError("Packet unknown");

    if (pk->getType() == RESERVE || pk->getType() == RESERVEBK) {
       newReserve(pk);
    }
    else if (pk->getType() == ACEPTED) {
        newAccepted(pk);
    }
    else if (pk->getType() == RELEASE || pk->getType() == RELEASEDELAYED || pk->getType() == RELEASEBREAK) {
        release(pk);
    }
    else if (pk->getType() == BREAK) {

    }
    else // flow control packets, can be used to compute the statistics
    {
        procFlowPk(pk);
    }
    rescheduleEvent();
}

void CallApp::finish()
{
    // total data of flows and calls that has finished
    long double totalSend = 0;
    long double totalRec = 0;
    for (auto elem : receivedBytes)
        totalRec += elem.second;
    for (auto elem : sendBytes)
        totalSend += elem.second;

    // register call in curse
    for (auto elem : activeCalls) {
        totalRec += elem.second->acumulateRec;
        totalSend += elem.second->acumulateSend;
    }
    activeCalls.clear();

    // pending flow "end" in transmission
    while (!FlowEvents.empty()) {
        // flows that the node is sending
        auto it = FlowEvents.begin();
        FlowEvent *flowEvent = it->second;
        FlowEvents.erase(it);
        double bsend = flowEvent->usedBandwith * SIMTIME_DBL(simTime() - flowEvent->startOn);
        totalSend += (bsend/1000);
        delete flowEvent;
    }
    // pending flow "end" in receptions
    while (!flowStatistics.empty()) {
        auto it = flowStatistics.begin();
        double brec = it->second.used * SIMTIME_DBL(simTime() - it->second.startOnRec);
        totalRec += (brec/1000);
        flowStatistics.erase(it);
    }

    recordScalar("Total Send Kb",totalSend);
    recordScalar("Total Rec Kb",totalRec);

    recordScalar("Total calls generated",callCounter);
    recordScalar("Total calls established",callEstabilized);
    recordScalar("Total calls received",callReceived);
}

void CallApp::receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj, cObject *details)
{
    Enter_Method_Silent();

    Actualize *pkt = dynamic_cast<Actualize *>(obj);
    if (pkt == nullptr || dijFuzzy == nullptr) {
        throw cRuntimeError("Sennal no esperada");
        return; //
    }
    // actualiza los estados para ejecutar disjtra.
    int nodeId = pkt->getSrcAddr();
    for (unsigned int i = 0; i < pkt->getLinkDataArraySize(); i++) {
        LinkData linkData = pkt->getLinkData(i);
        if (linkData.nominal == 0)
            dijFuzzy->deleteEdge(nodeId,linkData.node);
        else {
            double minResidual = linkData.nominal-linkData.min;
            double meanResidual = linkData.nominal-linkData.mean;
            double maxResidual = linkData.nominal-linkData.max;

            if (residual) {
                if (minResidual < 0.0001)
                    minResidual = 1/0.0001;
                else
                    minResidual = 1/minResidual;

                if (meanResidual < 0.0001)
                    meanResidual = 1/0.0001;
                else
                    meanResidual = 1/meanResidual;

                if (maxResidual < 0.0001)
                    maxResidual = 1/0.0001;
                else
                    maxResidual = 1/maxResidual;
            }
            else {
                // Usar funciones lineales  o hiperbólicas?

                minResidual =  (linkData.min/linkData.nominal);
                meanResidual = (linkData.mean/linkData.nominal);
                maxResidual = (linkData.max/linkData.nominal);
            }

            if (minResidual == 0)
                throw cRuntimeError("Problems detected");
            dijFuzzy->addEdge(nodeId,linkData.node,minResidual, meanResidual, maxResidual);
        }
    }
}
