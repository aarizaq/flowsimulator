//
// Copyright (C)Author: alfonso ariza quintana, Universidad de Malaga
// Copyright (C) 2017 Alfonso Ariza
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
#include <cinttypes>

uint64_t CallApp::callIdentifier = 1;
bool CallApp::residual = false;
simsignal_t CallApp::actualizationSignal = registerSignal("actualizationSignal");
simsignal_t CallApp::rcvdPk = registerSignal("rcvdPk");

//#define ONLYONECALL

Define_Module(CallApp);

CallApp::CallApp()
{
    callIdentifier = 1;
}

CallApp::~CallApp()
{
    if (generateCall)
        cancelAndDelete(generateCall);
    if (nextEvent)
        cancelAndDelete(nextEvent);
    CallPacketsEvents.clear();
    while (!activeCalls.empty()) {
        delete activeCalls.begin()->second;
        activeCalls.erase(activeCalls.begin());
    }
    CallEvents.clear();
}


void CallApp::rescheduleEvent()
{
    if (CallEvents.empty() && FlowEvents.empty() && CallPacketsEvents.empty() && FlowPacketsEvents.empty()) {
        if (nextEvent->isScheduled())
            cancelEvent(nextEvent);
        return;
    }

    simtime_t min1 = SimTime::getMaxTime();
    simtime_t min2 = SimTime::getMaxTime();
    simtime_t min3 = SimTime::getMaxTime();
    simtime_t min4 = SimTime::getMaxTime();

    if (!CallEvents.empty())
        min1 = CallEvents.begin()->first;
    if (!FlowEvents.empty())
        min2 = FlowEvents.begin()->first;
    if (!CallPacketsEvents.empty())
        min3 = CallPacketsEvents.begin()->first;
    if (!FlowPacketsEvents.empty())
        min4 = FlowPacketsEvents.begin()->first;

    simtime_t min = min1<min2?min1:min2;
    if (min > min3)
        min = min3;
    if (min>min4)
        min = min4;

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

// Event flow generate methods,
// Call associated methods.



void CallApp::newCallFlow(CallInfo *callInfo, const uint64_t  &bw)
{
    char pkname[100];
    callInfo->state = ON;
    callInfo->flowId++;
    callInfo->reservedBandwith = bw;
    callInfo->usedBandwith = (uint64_t) usedBandwith->doubleValue();
    callInfo->paketSize = packetSize->intValue();
    simtime_t delayAux = TimeOn->doubleValue();
    callInfo->startOn = simTime();
    CallEvents.insert(std::make_pair(simTime() + delayAux, callInfo));

    if (callInfo->dest == myAddress)
        throw cRuntimeError("Destination address erroneous");

    if (simulationMode == PACKETMODE) {
        // TODO: Compute interarrival time of the packet
        callInfo->interArrivalTime = (double) callInfo->paketSize/ (double) callInfo->usedBandwith;
        newCallPacket(callInfo);
        return;
    }

    Packet *pkFlow = new Packet();
    pkFlow->setSourceId(par("sourceId").intValue());
    pkFlow->setDestinationId(callInfo->sourceId);
    pkFlow->setSrcAddr(myAddress);
    pkFlow->setDestAddr(callInfo->dest);
    pkFlow->setType(STARTFLOW);
    pkFlow->setReserve(callInfo->usedBandwith);
    pkFlow->setFlowId(callInfo->flowId);
    pkFlow->setCallId(callInfo->callId);
    sprintf(pkname, "FlowOn-%d-to-%d-#%" PRIu64 "-Sid-%d-FlowId-%" PRIu64 "", myAddress,
            pkFlow->getDestAddr(), pkFlow->getCallId(),
            this->getIndex(), callInfo->flowId);
    pkFlow->setName(pkname);

    send(pkFlow, "out");
}


double CallApp::startCallFlow(CallInfo *callInfo, Packet *pkFlow)
{
    char pkname[100];
    callInfo->state = ON;
    callInfo->flowId++;
    pkFlow->setFlowId(callInfo->flowId);
    callInfo->usedBandwith = (uint64_t) usedBandwith->doubleValue();
    callInfo->paketSize = packetSize->intValue();
    pkFlow->setReserve(callInfo->usedBandwith);
    pkFlow->setType(STARTFLOW);
    callInfo->startOn = simTime();

    sprintf(pkname, "FlowOn-%d-to-%d-CallId#%" PRIu64 "-FlowId#%" PRIu64 "-Sid-%d", myAddress, pkFlow->getDestAddr(),
            pkFlow->getCallId(), pkFlow->getFlowId(), this->getIndex());
    pkFlow->setName(pkname);
    return TimeOn->doubleValue();

}


double CallApp::endCallFlow(CallInfo *callInfo, Packet *pkFlow)
{

    // delete pending packets
    // TODO: Multiflow support for packets.
    for (auto it = CallPacketsEvents.begin(); it != CallPacketsEvents.end();) {
        if (it->second == callInfo)
            CallPacketsEvents.erase(it++);
        else
            ++it;
    }

    char pkname[100];
    bytesTraceSend(callInfo);
    double bsend = callInfo->usedBandwith * SIMTIME_DBL(simTime() - callInfo->startOn);
    callInfo->acumulateSend += (bsend/1000.0);
    EV << "Call Id :" << callInfo->callId << "Flow Id :" << callInfo->flowId <<
            " Time in on :" << callInfo->startOn << "Bsend :" << bsend;
    callInfo->state = OFF;
    pkFlow->setType(ENDFLOW);
    pkFlow->setFlowId(callInfo->flowId);
    pkFlow->setReserve(callInfo->usedBandwith);

    sprintf(pkname, "FlowOff-%d-to-%d-CallId#%" PRIu64 "-FlowId#%" PRIu64 "-Sid-%d",
                                    myAddress, pkFlow->getDestAddr(),
                                    pkFlow->getCallId(),pkFlow->getFlowId(), this->getIndex());
    pkFlow->setName(pkname);
    return TimeOff->doubleValue();
}

double CallApp::startCallFlow(CallInfo *callInfo, Packet *pkFlow, FlowData & elem)
{
    char pkname[100];
    elem.state = ON;
    callInfo->flowId++;
    elem.flowId = callInfo->flowId;
    pkFlow->setFlowId(callInfo->flowId);
    elem.usedBandwith = (uint64_t) usedBandwith->doubleValue();
    elem.paketSize = packetSize->intValue();
    pkFlow->setReserve(elem.usedBandwith);
    pkFlow->setType(STARTFLOW);
    callInfo->startOn = simTime();
    sprintf(pkname, "FlowOn-%d-to-%d-CallId#%" PRIu64 " - FlowId#%" PRIu64 " -Sid-%d",
            myAddress, pkFlow->getDestAddr(),
            pkFlow->getCallId(),pkFlow->getFlowId(), this->getIndex());
    pkFlow->setName(pkname);
    return TimeOn->doubleValue();

}

double CallApp::endCallFlow(CallInfo *callInfo, Packet *pkFlow, FlowData & elem)
{
    char pkname[100];

    bytesTraceRec(callInfo);
    callInfo->acumulateSend += ((elem.usedBandwith * SIMTIME_DBL(simTime() - elem.startOn))/1000.0);
    pkFlow->setFlowId(elem.flowId);
    elem.state = OFF;
    pkFlow->setType(ENDFLOW);
    pkFlow->setReserve(elem.usedBandwith);
    sprintf(pkname, "FlowOff-%d-to-%d-CallId#%" PRIu64 "- FlowId#%" PRIu64 " -Sid-%d",
            myAddress, pkFlow->getDestAddr(),
            pkFlow->getCallId(),pkFlow->getFlowId(), this->getIndex());
    pkFlow->setName(pkname);
    return TimeOff->doubleValue();
}

// Flow associated methods.

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
    pkFlow->setSourceId(par("sourceId").intValue());
    pkFlow->setDestinationId(par("destinationId").intValue());
    pkFlow->setSrcAddr(myAddress);
    pkFlow->setFlowId(flowIdentifier++);
    pkFlow->setType(STARTFLOW);

    sprintf(pkname, "NewFlow-%d-to-%d-Call Id #%" PRIu64 "- Flow Id #%" PRIu64 " -Did-%ld",
            myAddress, destAddress, pkFlow->getCallId(), pkFlow->getFlowId(),
            par("sourceId").intValue());
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
    event->paketSize = packetSize->intValue();
    event->startOn = simTime();
    FlowEvents.insert(std::make_pair(simTime() + flowDuration->doubleValue(), event));
}


void  CallApp::endFlow(FlowEvent *flowEvent) {
    char pkname[100];

    Packet *pkFlow = new Packet();
    pkFlow->setReserve(flowEvent->usedBandwith);
    pkFlow->setDestAddr(flowEvent->dest);
    pkFlow->setCallId(0);
    pkFlow->setSourceId(par("sourceId").intValue());
    pkFlow->setDestinationId(flowEvent->destId);
    pkFlow->setSrcAddr(myAddress);
//callInfo->acumulateSend += (callInfo->usedBandwith
//        * callInfo->startOn.dbl());

    pkFlow->setType(ENDFLOW);
    pkFlow->setFlowId(flowEvent->flowId);
    sprintf(pkname, "FlowOff-%d-to-%d-CallId#%" PRIu64 "- FlowId#%" PRIu64 " -Sid-%d",
            myAddress, pkFlow->getDestAddr(),
            pkFlow->getCallId(),pkFlow->getFlowId(), this->getIndex());
    pkFlow->setName(pkname);
    if (pkFlow->getDestAddr() == myAddress)
        throw cRuntimeError("Destination address erroneous");
    send(pkFlow, "out");
    if (simulationMode == FLOWMODE) {
        double bsend = flowEvent->usedBandwith * SIMTIME_DBL(simTime() - flowEvent->startOn);
        auto itStat = sendBytes.find(flowEvent->dest);
        if (itStat == sendBytes.end())
            sendBytes[flowEvent->dest] = (bsend/1000.0);
        else
            itStat->second += (bsend/1000.0);
    }
}

// packet generaration
void CallApp::newCallPacket(CallInfo *callInfo)
{
    char pkname[100];

    if (callInfo->interArrivalTime == SimTime::ZERO)
        return;

    Packet *pkFlow = new Packet();
    pkFlow->setSourceId(par("sourceId").intValue());
    pkFlow->setDestinationId(callInfo->sourceId);
    pkFlow->setSrcAddr(myAddress);
    pkFlow->setDestAddr(callInfo->dest);
    pkFlow->setType(DATATYPE);
    pkFlow->setReserve(callInfo->usedBandwith);
    pkFlow->setFlowId(callInfo->flowId);
    pkFlow->setCallId(callInfo->callId);
    pkFlow->setBitLength(callInfo->paketSize);

    sprintf(pkname, "Data-%d-to-%d-#%" PRIu64 "-Sid-%d-FlowId-%" PRIu64, myAddress, pkFlow->getDestAddr(), pkFlow->getCallId(),
            this->getIndex(), callInfo->flowId);
    pkFlow->setName(pkname);
    // check if it is the latest packet of the flow.
    for (auto elem : CallEvents) {
        if (elem.second == callInfo) {
            if (elem.second->state != ON)
                throw cRuntimeError("Call state Error");


            if (simTime() + callInfo->interArrivalTime < elem.first)
                CallPacketsEvents.insert(std::make_pair(simTime() + callInfo->interArrivalTime, callInfo));
            else if (simTime() > elem.first)
                throw cRuntimeError("CallEvents Scheduler error compute time off");
            else
                pkFlow->setLast(true);                 // last
            break;
        }
    }

    if (pkFlow->getDestAddr() == myAddress)
        throw cRuntimeError("Destination address erroneous");

    auto itStat = sendBytes.find(pkFlow->getDestAddr());
    if (itStat == sendBytes.end())
        sendBytes[pkFlow->getDestAddr()] = pkFlow->getBitLength();
    else
        itStat->second += pkFlow->getBitLength();
    totalPkSent++;
    send(pkFlow, "out");
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
        pkFlow->setSourceId(par("sourceId").intValue());
        pkFlow->setDestinationId(callInfo->sourceId);
        pkFlow->setSrcAddr(myAddress);

        if (callInfo->flowData.empty()) {
            if (callInfo->state == ON) {
                callInfo->interArrivalTime = SimTime::ZERO;
                delayAux = endCallFlow(callInfo, pkFlow);
            }
            else if (callInfo->state == OFF) {
                double timePackets = (double) callInfo->paketSize/ (double) callInfo->usedBandwith;
                callInfo->interArrivalTime = SimTime(timePackets);
                delayAux = startCallFlow(callInfo, pkFlow);
            }
            if (pkFlow->getDestAddr() == myAddress)
                throw cRuntimeError("Destination address erroneous");
            if (simulationMode == FLOWMODE)
                send(pkFlow, "out");
            else
                delete pkFlow;
            CallEvents.insert(std::make_pair(simTime() + delayAux, callInfo));
            // If simulation is Packet mode and state is in on, generate next
            if (simulationMode == PACKETMODE && callInfo->state == ON) {
                newCallPacket(callInfo);
            }
        }
        else {
            // multi flow system
            throw cRuntimeError("Disabled");
            for (auto &elem : callInfo->flowData) {
                if (elem.nextEvent > simTime())
                    continue;
                if (elem.state == ON) {
                    delayAux = endCallFlow(callInfo, pkFlow, elem);
                }
                else if (elem.state == OFF) {
                    delayAux = startCallFlow(callInfo, pkFlow, elem);
                    if (simulationMode == PACKETMODE) {
                        // TODO: Compute interarrival time of the packet
                        callInfo->interArrivalTime = (double) callInfo->paketSize/ (double) callInfo->usedBandwith;
                        newCallPacket(callInfo);
                    }
                }
                elem.nextEvent = simTime() + delayAux;
                if (pkFlow->getDestAddr() == myAddress)
                    throw cRuntimeError("Destination address erroneous");
                if (simulationMode == FLOWMODE)
                    send(pkFlow, "out");
                else
                    delete pkFlow;
                CallEvents.insert(std::make_pair(simTime() + delayAux, callInfo));
            }
        }
    }

    while (!FlowEvents.empty() && FlowEvents.begin()->first <= simTime()) {
        auto it = FlowEvents.begin();
        FlowEvent *flowEvent = it->second;
        FlowEvents.erase(it);
        endFlow(flowEvent);
        delete flowEvent;
    }


    while (!CallPacketsEvents.empty() && CallPacketsEvents.begin()->first <= simTime()) {
        auto it = CallPacketsEvents.begin();

        CallInfo *callEvent = it->second;
        CallPacketsEvents.erase(it);
        newCallPacket(callEvent);
    }

    while (!FlowPacketsEvents.empty() && FlowPacketsEvents.begin()->first <= simTime()) {
        auto it = FlowPacketsEvents.begin();
        //FlowEvent *flowEvent = it->second;
        FlowPacketsEvents.erase(it);
        throw cRuntimeError("implementation pending");
        // procFlowPacketEvent(flowEvent);
    }
}



void CallApp::newCall() {
    if (destAddresses.empty())
        return;

    char pkname[100];
// Sending packet
    callCounter++;

    int destAddress = destAddresses[intuniform(0, destAddresses.size() - 1)];

    sprintf(pkname, "CallReserve-%d-to-%d-Call id#%" PRIu64 "-Did-%ld", myAddress,
            destAddress, callIdentifier, par("sourceId").intValue());
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
    pk->setSourceId(par("sourceId").intValue());
    pk->setDestinationId(par("destinationId").intValue());

    DijkstraFuzzy::Route r1, r2, min;

    if (routingModule->getRoutingType() == IRouting::BACKUPROUTE || routingModule->getRoutingType() == IRouting::BACKUPROUTEKSH) {
        routingModule->getRoute(destAddress,r1,r2);
        // TODO: backup mode Se deben enviar dos paquetes, uno por cada ruta
        // new call id for backup route
        Packet *pk2 = pk->dup();
        pk2->setCallId(callIdentifier);
        callIdentifier++;
        pk2->setCallIdBk(pk->getCallId());
        pk->setCallIdBk(pk2->getCallId());
        pk2->setRouteArraySize(r1.size());
        for (unsigned int i = 0; i < r1.size(); i++) {
            pk2->setRoute(i, r1[i]);
        }
        pk->setRouteArraySize(r2.size());
        for (unsigned int i = 0; i < r2.size(); i++) {
            pk->setRoute(i, r2[i]);
        }
        pk2->setType(RESERVEBK);
        // send before the best
        send(pk2, "out");
    }
    else {
        routingModule->getRoute(destAddress,r1,r2);
        pk->setRouteArraySize(r1.size());
        for (unsigned int i = 0; i < r1.size(); i++) {
            pk->setRoute(i, r1[i]);
        }
    }
// TODO : recall timer,
    send(pk, "out");
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
    sprintf(pkname, "PkAccepted-%d-to-%d-#%" PRIu64 "-Sid-%d", myAddress, pk->getDestAddr(), pk->getCallId(), this->getIndex());
    pk->setName(pkname);
    pk->setDestinationId(pk->getSourceId());
    pk->setSourceId(par("sourceId").intValue());
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
        if (generateFlow) {
            newCallFlow(callInfo, pk->getReserve());
        }
        else
            callInfo->state = PASSIVE;
        // prepare release event
        callEstabilized++;
        char pkname[100];
        pk->setType(RELEASE);
        pk->setDestAddr(pk->getSrcAddr());
        pk->setSrcAddr(myAddress);
        pk->setDestinationId(pk->getSourceId());
        pk->setSourceId(par("sourceId").intValue());

        sprintf(pkname, "Pkrelease-%d-to-%d-#%" PRIu64 "-Sid-%d", myAddress,
                pk->getDestAddr(), pk->getCallId(), this->getIndex());
        pk->setName(pkname);
        scheduleAt(simTime() + callDuration->doubleValue(), pk);
        listPendingRelease.push_back(pk);
    }
    else
    {
        delete pk;
    }
}

void CallApp::storeCallStatistics(const CallInfo *callInfo) {

    if (callInfo->state == ON) {
        bytesTraceSend (callInfo);
        double bsend = callInfo->usedBandwith
                * SIMTIME_DBL(simTime() - callInfo->startOn);
        const_cast<CallInfo *>(callInfo)->acumulateSend += ((bsend) / 1000.0);
    }

    if (callInfo->stateRec == ON) {
        bytesTraceRec (callInfo);
        double brec = callInfo->recBandwith
                * SIMTIME_DBL(simTime() - callInfo->startOnRec);
        const_cast<CallInfo *>(callInfo)->acumulateRec += (brec / 1000.0);
    }

// record the statistics
    if (simulationMode == FLOWMODE) {
        auto itAccSend = sendBytes.find(callInfo->dest);
        auto itAccRec = receivedBytes.find(callInfo->dest);
        if (itAccSend == sendBytes.end())
            sendBytes[callInfo->dest] = callInfo->acumulateSend;
        else
            itAccSend->second += callInfo->acumulateSend;
        if (itAccRec == receivedBytes.end())
            receivedBytes[callInfo->dest] = callInfo->acumulateRec;
        else
            itAccRec->second += callInfo->acumulateRec;
    }
}

void CallApp::release(Packet *pk) {

    // Handle incoming packet

    auto it = activeCalls.find(pk->getCallId());
    auto it2 = backupCalls.find(pk->getCallId());



    if (pk->isSelfMessage()) {
        // search in the list
        auto itPendingRelease = std::find(listPendingRelease.begin(),listPendingRelease.end(),pk);
        if (itPendingRelease == listPendingRelease.end()) {
            throw cRuntimeError("Packet not found in the pending release");
        }
        else
            listPendingRelease.erase(itPendingRelease);
    }

    if (it == activeCalls.end() && it2 == backupCalls.end()) {
        if (!pk->isSelfMessage()) // si se ha liberador por rotura debería haber llegado a la otra parte el relese con lo cual no debe mandar el mensaje de release otra vez
            throw cRuntimeError("Call Id not found in any list");
        else {
            if (pk->getCallIdBk() == 0) {
                delete pk;
                return;
            }
            auto it = activeCalls.find(pk->getCallIdBk());
            if (it == activeCalls.end()) {
                callRejected++;
                delete pk;
                return;
            }
            else {
             // El backup pasó a principal, se debe eliminar ahora el backup
                throw cRuntimeError("Call Id not found in any list");
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
        // delete all events relative to this call
        CallInfo *callInfo = it->second;
        for (auto it = CallEvents.begin(); it != CallEvents.end();) {
            if (it->second->callId == pk->getCallId())
                CallEvents.erase(it++);
            else
                ++it;
        }

        if (callInfo->callIdBk != 0) // change to backup route
        {
            for (auto & elem : listPendingRelease) {
                if (elem->getCallId() == callInfo->callId) {
                    elem->setCallId(callInfo->callIdBk);
                    break;
                }
            }

            auto itAux =  backupCalls.find(it->second->callIdBk);
            if (itAux != backupCalls.end())
                backupCalls.erase(itAux);
            else
                throw cRuntimeError("Check bk table");

            // if active flows send end
            if (callInfo->state == ON) {
                 bytesTraceSend(callInfo);
                callInfo->acumulateSend += ((callInfo->usedBandwith
                        * SIMTIME_DBL(simTime() - callInfo->startOn))/1000);
                // send off in the o
                callInfo->state = OFF;
                if (simulationMode != PACKETMODE) {
                    Packet *pkFlow = new Packet();
                    pkFlow->setDestAddr(callInfo->dest);
                    pkFlow->setCallId(callInfo->callId);
                    pkFlow->setSourceId(par("sourceId").intValue());
                    pkFlow->setDestinationId(callInfo->sourceId);
                    pkFlow->setType(ENDFLOW);
                    pkFlow->setFlowId(callInfo->flowId);
                    pkFlow->setReserve(callInfo->usedBandwith);
                    char pkname[60];
                    sprintf(pkname,"FlowOff-%d-to-%d-CallId#%" PRIu64 "-FlowId#%" PRIu64 "Sid-%d", myAddress, pkFlow->getDestAddr(),  pkFlow->getCallId(), pkFlow->getFlowId(), this->getIndex());
                    pkFlow->setName(pkname);
                    send(pkFlow, "out");
                }
                CallEvents.insert(std::make_pair(simTime() + TimeOff->doubleValue(), callInfo));
            }
            // if active flows send end
            if (callInfo->stateRec == ON) {
                bytesTraceRec(callInfo);
                callInfo->acumulateRec += ((callInfo->recBandwith
                        * SIMTIME_DBL(simTime() - callInfo->startOnRec))/1000);
                // send off in the o
                callInfo->stateRec = OFF;
            }
            // change all to backup
            activeCalls.erase(it);
            uint64_t callidBk = callInfo->callIdBk;
            callInfo->callIdBk = 0;
            callInfo->callId = callidBk;
            if (callInfo->dest == myAddress)
                throw cRuntimeError("Address destination Error");
            activeCalls.insert(std::make_pair(callInfo->callId,callInfo));
        }
        else {
            // Accumulate the flows in curse
            storeCallStatistics(callInfo);
            for (auto itAux = CallPacketsEvents.begin(); itAux != CallPacketsEvents.end();) {
                if (itAux->second == callInfo)
                    CallPacketsEvents.erase(itAux++);
                else
                    ++itAux;
            }
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
        emit(rcvdPk,pk);
        if (flowId.callId() > 0) {
            auto itAux = activeCalls.find(flowId.callId());
            if (itAux == activeCalls.end())
                throw cRuntimeError("Call id not found but flow received");
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
                // if there is a active flow stop it and change to the other
                /*if (callInfo->state == ON) {

                    for (auto it = CallEvents.begin(); it != CallEvents.end();) {
                        if (it->second->callId == callInfo->callId)
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
                    pkFlow->setCallId(callInfo->callId);
                    pkFlow->setSourceId(par("sourceId").intValue());
                    pkFlow->setDestinationId(callInfo->sourceId);
                    pkFlow->setType(ENDFLOW);
                    pkFlow->setFlowId(callInfo->flowId);
                    pkFlow->setReserve(callInfo->usedBandwith);
                    char pkname[60];
                    sprintf(pkname,"FlowOff-%d-to-%d-CallId#%" PRIu64 "-FlowId#%" PRIu64 "-Sid-%d", myAddress, pkFlow->getDestAddr(),  pkFlow->getCallId(), pkFlow->getFlowId(), this->getIndex());
                    pkFlow->setName(pkname);
                    if (pkFlow->getDestAddr() == myAddress)
                        throw cRuntimeError("Destination address erroneous");
                    send(pkFlow, "out");
                    CallEvents.insert(std::make_pair(simTime()+TimeOff->doubleValue(), callInfo));
                }*/

                uint64_t callId = callInfo->callId;
                uint64_t calIdbk = callInfo->callIdBk;
                callInfo->callId = calIdbk;
                callInfo->callIdBk = callId;
                backupCalls.erase(itAux);
                itAux = activeCalls.find(callId);

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
                    callInfo->acumulateSend += ((callInfo->usedBandwith
                            * SIMTIME_DBL(simTime() - callInfo->startOn))/1000);
                    // send off in the o
                    callInfo->state = OFF;
                    Packet *pkFlow = new Packet();
                    pkFlow->setDestAddr(callInfo->dest);
                    pkFlow->setCallId(callId);
                    pkFlow->setSourceId(par("sourceId").intValue());
                    pkFlow->setDestinationId(callInfo->sourceId);
                    pkFlow->setType(ENDFLOW);
                    pkFlow->setFlowId(callInfo->flowId);
                    pkFlow->setReserve(callInfo->usedBandwith);
                    char pkname[60];

                    sprintf(pkname,"FlowOff-%d-to-%d-CallId#%" PRIu64 "-FlowId#%" PRIu64 "-Sid-%d", myAddress, pkFlow->getDestAddr(),  pkFlow->getCallId(), pkFlow->getFlowId(), this->getIndex());
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

void CallApp::getNodesAddress(const char *destAddressesPar, std::vector<int> &listAddres)
{
    listAddres.clear();
    const char *token;
    if (strcmp(destAddressesPar,"any")==0) {
        cTopology topo("topo");
        std::vector<std::string> nedTypes;
        nedTypes.push_back(getParentModule()->getNedTypeName());
        topo.extractByNedTypeName(nedTypes);
        for (int i = 0; i < topo.getNumNodes(); i++) {
            cTopology::Node *node = topo.getNode(i);
            int destAddr = node->getModule()->par("address");
            if (destAddr != myAddress)
                listAddres.push_back(destAddr);
       }
    }
    else {
        cStringTokenizer tokenizer(destAddressesPar);
        while ((token = tokenizer.nextToken()) != NULL)
        {
            int destAddr = atoi(token);
            if (destAddr != myAddress)
                destAddresses.push_back(atoi(token));
        }
    }
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

    packetSize = &par("packetSize");
    flowPacketSize = &par("flowPacketSize");

    std::vector<double> a;
    std::vector<uint32_t> b;

    callCounter = 0;
    if (par("packetMode"))
        simulationMode = PACKETMODE;

    WATCH(callCounter);
    WATCH(callReceived);
    WATCH(myAddress);
    WATCH_MAP(receivedBytes);
    WATCH_MAP(sendBytes);


    getNodesAddress(par("destAddresses"),destAddresses);

    generateCall = new cMessage("nextCall");
#ifndef ONLYONECALL
    if (!destAddresses.empty())
        scheduleAt(callArrival->doubleValue(), generateCall);
#else
    if (!destAddresses.empty())
        scheduleAt(simTime(), generateCall);
#endif


    routingModule = check_and_cast<IRouting *>(this->getParentModule()->getSubmodule("routing"));


    if (par("independentFlows").boolValue() && !destAddresses.empty()) {
        nextFlow = new cMessage();
        scheduleAt(flowArrival->doubleValue(), nextFlow);
    }

    nextEvent = new cMessage("NewEvent");
    RegisterMsg * msg = new RegisterMsg();
    msg->setSourceId(par("sourceId").intValue());
    send(msg, "out");

    initTime = time(nullptr);
#if 1
    DijkstraFuzzy dijFuzzy;
    dijFuzzy.setAlpha(0.6);
    dijFuzzy.addLink(1, 2, 1, 2, 4);
    dijFuzzy.addLink(1, 5, 6, 13, 15);
    dijFuzzy.addLink(1, 6, 11, 14, 14);
    dijFuzzy.addLink(2, 3, 0, 2, 4);
    dijFuzzy.addLink(2, 4, 0, 2, 6);
    dijFuzzy.addLink(3, 4, 3, 4, 8);
    dijFuzzy.addLink(3, 5, 2, 3, 3);
    dijFuzzy.addLink(3, 7, 5, 7, 11);
    dijFuzzy.addLink(5, 6, 1, 5, 8);
    dijFuzzy.addLink(5, 7, 1, 3, 6);
    dijFuzzy.addLink(6, 7, 4, 6, 6);
    dijFuzzy.addLink(6, 8, 0, 1, 3);
    dijFuzzy.addLink(7, 9, 9, 10, 12);
    dijFuzzy.addLink(7, 12, 7, 12, 15);
    dijFuzzy.addLink(8, 9, 0, 1, 2);
    dijFuzzy.addLink(8, 10, 3, 5, 6);
    dijFuzzy.addLink(9, 10, 0, 2, 3);
    dijFuzzy.addLink(9, 11, 2, 3, 3);
    dijFuzzy.addLink(9, 12, 7, 8, 8);
    dijFuzzy.addLink(10, 11, 0, 2, 4);
    dijFuzzy.addLink(11, 12, 9, 10, 13);

    dijFuzzy.setRoot(myAddress);
    dijFuzzy.setHasFindDisjoint(true);
    dijFuzzy.run();
    std::ofstream myfile;
    std::string name= "rutas"+std::to_string(myAddress)+".txt";
    myfile.open (name);

    for (int i = 1; i <=12; i ++) {
        if (i == myAddress) continue;
        dijFuzzy.runDisjoint(i);
        DijkstraFuzzy::Route r1;
        DijkstraFuzzy::Route r2;
        if (dijFuzzy.checkDisjoint(i, r1, r2)) {
            // print routes
            myfile << "Origin : " + std::to_string(myAddress);
            myfile << "destination : " + std::to_string(i) << "\n";
            myfile << "r1 : " ;
            for (auto elem : r1) {
                myfile << std::to_string(elem)+"-";
            }
            myfile << "\n";
            myfile << "r2 : " ;
            for (auto elem : r2) {
                myfile << std::to_string(elem)+"-";
            }
            myfile << "\n";
        }
        else {
            throw cRuntimeError("ERROR");
        }
    }
    myfile.close();
#endif
}

void CallApp::handleMessage(cMessage *msg)
{

    if (!msg->isPacket()) {
        if (msg == generateCall) {
            newCall();
#ifndef ONLYONECALL
            if (!destAddresses.empty())
                scheduleAt(simTime() + callArrival->doubleValue(), generateCall);
            if (hasGUI())
                getParentModule()->bubble("Generating call..");
#endif
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
    else if (pk->getType() == REJECTED) {
        callRejected++;
        auto it = activeCalls.find(pk->getCallId());
        if (it != activeCalls.end())
            throw cRuntimeError("This call must not be registered");
        delete pk;
    }
    else if (pk->getType() == DATATYPE) {
        totalPkRec++;
        auto itAccRec = receivedBytes.find(pk->getSrcAddr());
        if (itAccRec == receivedBytes.end())
            receivedBytes[pk->getSrcAddr()] = pk->getBitLength();
        else
            itAccRec->second += pk->getBitLength();

        emit(rcvdPk,pk);
        delete pk;
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
    if (simulationMode == FLOWMODE) {
        for (auto elem : receivedBytes)
            totalRec += elem.second;
        for (auto elem : sendBytes)
            totalSend += elem.second;

        // register call in curse
        for (auto elem : activeCalls) {
            if (elem.second->state == ON) {
                bytesTraceSend(elem.second);
                elem.second->acumulateSend += ((elem.second->usedBandwith * SIMTIME_DBL(simTime() - elem.second->startOn))/1000);
            }
            if (elem.second->stateRec == ON) {
                bytesTraceRec(elem.second);
                elem.second->acumulateRec += ((elem.second->recBandwith * SIMTIME_DBL(simTime() - elem.second->startOnRec))/1000);
            }

            totalRec += elem.second->acumulateRec;
            totalSend += elem.second->acumulateSend;
        }

    }
    else {
        for (auto elem : receivedBytes)
            totalRec += (elem.second/1000);
        for (auto elem : sendBytes)
            totalSend += (elem.second/1000);
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

    recordScalar("Total Pk Sent",totalPkSent);
    recordScalar("Total Pk Rec",totalPkRec);

    recordScalar("Total calls generated",callCounter);
    recordScalar("Total calls established",callEstabilized);
    recordScalar("Total calls received",callReceived);

    recordScalar("Total calls callRejected",callRejected);

    recordScalar("simulation time",time(nullptr)-initTime);

}
