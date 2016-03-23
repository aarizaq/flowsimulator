//
// Copyright (C)Author: alfonso ariza quintana, Universidad de Malaga
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//

#ifdef _MSC_VER
#pragma warning(disable:4786)
#endif

#include "CallApp.h"
#include "Packet_m.h"

uint64_t CallApp::callIdentifier = 1;

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
    for (int i = 0; i < topo.getNumNodes(); i++) {
        cTopology::Node *node = topo.getNode(i);
        int address = node->getModule()->par("address");
        for (int j = 0; j < node->getNumOutLinks(); j++) {
            int addressAux = node->getLinkOut(j)->getRemoteNode()->getModule()->par("address");
            dijFuzzy->addEdge(address, addressAux, 1, 2, 3);
        }
    }
}

void CallApp::rescheduleEvent()
{
    if (CallEvents.empty()) {
        if (nextEvent->isScheduled())
            cancelEvent(nextEvent);
        return;
    }
    if (!nextEvent->isScheduled()) {
        scheduleAt(CallEvents.begin()->first, nextEvent);
    }
    else if (CallEvents.begin()->first != nextEvent->getArrivalTime()) {
        cancelEvent(nextEvent);
        scheduleAt(CallEvents.begin()->first, nextEvent);
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
    nextEvent = new cMessage("NewEvent");
    readTopo();
    RegisterMsg * msg = new RegisterMsg();
    msg->setSourceId(par("sourceId"));
    send(msg, "out");
}

void CallApp::handleMessage(cMessage *msg)
{
    char pkname[40];
    if (msg == generateCall) {
        // Sending packet
        callCounter++;
        int destAddress = destAddresses[intuniform(0, destAddresses.size() - 1)];

        sprintf(pkname, "pkReserve-%d-to-%d-#%lud-Did-%ld", myAddress, destAddress, callIdentifier,
                par("sourceId").longValue());
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
        pk->setSourceId(par("sourceId"));
        pk->setDestinationId(par("destinationId"));

        // TODO: Include the source routing in the packet.
        // dijFuzzy->runDisjoint(destAddress);

        // TODO : recall timer,

        send(pk, "out");

        if (!destAddresses.empty())
            scheduleAt(simTime() + callArrival->doubleValue(), generateCall);
        if (hasGUI())
            getParentModule()->bubble("Generating call..");
        rescheduleEvent();
        return;
    }
    else if (msg == nextEvent) {
        double delayAux;
        while (!CallEvents.empty() && CallEvents.begin()->first <= simTime()) {
            auto it = CallEvents.begin();
            CallInfo *callInfo = it->second;
            CallEvents.erase(it);

            Packet *pkFlow = new Packet();
            pkFlow->setReserve(callInfo->usedBandwith);
            pkFlow->setDestAddr(callInfo->dest);
            pkFlow->setCallId(callInfo->callId);
            pkFlow->setSourceId(par("sourceId"));
            pkFlow->setDestinationId(callInfo->sourceId);
            pkFlow->setSrcAddr(myAddress);

            if (callInfo->flowData.empty()) {
                if (callInfo->state == ON) {
                    callInfo->acumulateSend += (callInfo->usedBandwith * callInfo->startOn.dbl());
                    sprintf(pkname, "FlowOff-%d-to-%d-CallId#%lud-Sid-%d", myAddress, pkFlow->getDestAddr(), callIdentifier, this->getIndex());
                    pkFlow->setName(pkname);
                    callInfo->state = OFF;
                    pkFlow->setType(ENDFLOW);
                    pkFlow->setFlowId(callInfo->flowId);
                    delayAux = TimeOff->doubleValue();

                }
                else if (callInfo->state == OFF) {
                    sprintf(pkname, "FlowOn-%d-to-%d-#%lud-Sid-%d", myAddress, pkFlow->getDestAddr(), callIdentifier, this->getIndex());
                    pkFlow->setName(pkname);
                    callInfo->state = ON;
                    callInfo->flowId++;
                    pkFlow->setFlowId(callInfo->flowId);
                    callInfo->usedBandwith = (uint64_t) usedBandwith->doubleValue();
                    pkFlow->setType(STARTFLOW);
                    delayAux = TimeOn->doubleValue();
                    callInfo->startOn = delayAux;
                }
                send(pkFlow, "out");
                CallEvents.insert(std::make_pair(simTime() + delayAux, callInfo));
            }
            else {
                // multi flow system
                for (auto &elem : callInfo->flowData) {
                    if (elem.nextEvent > simTime())
                        continue;
                    if (elem.state == ON) {
                        callInfo->acumulateSend += (elem.usedBandwith * elem.startOn.dbl());
                        sprintf(pkname, "FlowOff-%d-to-%d-CallId#%lud-Sid-%d", myAddress, pkFlow->getDestAddr(),
                                callIdentifier, this->getIndex());

                        pkFlow->setFlowId(elem.flowId);
                        pkFlow->setName(pkname);
                        elem.state = OFF;
                        pkFlow->setType(ENDFLOW);
                        delayAux = TimeOff->doubleValue();

                    }
                    else if (elem.state == OFF) {
                        sprintf(pkname, "FlowOn-%d-to-%d-#%lud-Sid-%d", myAddress, pkFlow->getDestAddr(),
                                callIdentifier, this->getIndex());
                        pkFlow->setName(pkname);
                        elem.state = ON;
                        callInfo->flowId++;
                        elem.flowId = callInfo->flowId;
                        pkFlow->setFlowId(callInfo->flowId);
                        elem.usedBandwith = (uint64_t) usedBandwith->doubleValue();
                        pkFlow->setType(STARTFLOW);
                        delayAux = TimeOn->doubleValue();
                        callInfo->startOn = delayAux;
                    }

                    elem.nextEvent = simTime() + delayAux;
                    send(pkFlow, "out");
                    CallEvents.insert(std::make_pair(simTime() + delayAux, callInfo));
                }

            }
        }

        rescheduleEvent();
        return;
    }
    Base *pkaux = dynamic_cast<Base *>(msg);
    if (pkaux->getType() == ACTUALIZE) {
        Actualize *pk = dynamic_cast<Actualize *>(msg);
        for (unsigned int i = 0; i < pk->getLinkDataArraySize(); i++) {
            double residual = pk->getLinkData(i).residual;
            double cost = 1 / residual;

            if (residual > 1e20)
                dijFuzzy->eraseEdge(pk->getSrcAddr(), pk->getLinkData(i).node);
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

    if (pk->getType() == RESERVE) {
        pk->setType(ACEPTED);
        pk->setDestAddr(pk->getSrcAddr());
        pk->setSrcAddr(myAddress);
        sprintf(pkname, "PkAccepted-%d-to-%d-#%lud-Sid-%d", myAddress, pk->getDestAddr(), pk->getCallId(),
                this->getIndex());
        pk->setName(pkname);

        callReceived++;

        auto itAux = activeCalls.find(pk->getCallId());
        if (itAux != activeCalls.end())
            throw cRuntimeError("Call id presents in the system");

        CallInfo *callInfo = new CallInfo();
        callInfo->dest = pk->getDestAddr();
        callInfo->callId = pk->getCallId();
        callInfo->sourceId = pk->getSourceId();

        pk->setSourceId(par("sourceId"));
        pk->setDestinationId(callInfo->sourceId);

        // check in the list of calls

        activeCalls.insert(std::make_pair(callInfo->callId, callInfo));
        if (generateFlow) {
            callInfo->state = OFF;
            callInfo->reservedBandwith = pk->getReserve();
            callInfo->usedBandwith = (uint64_t) usedBandwith->doubleValue();
            CallEvents.insert(std::make_pair(simTime() + TimeOff->doubleValue(), callInfo));
        }
        else
            callInfo->state = PASSIVE;

        send(pk, "out");
    }
    else if (pk->getType() == ACEPTED) {
        pk->setType(RELEASE);
        pk->setDestAddr(pk->getSrcAddr());
        pk->setSrcAddr(myAddress);
        sprintf(pkname, "Pkrelease-%d-to-%d-#%lud-Sid-%d", myAddress, pk->getDestAddr(), callIdentifier,
                this->getIndex());
        pk->setName(pkname);

        auto itAux = activeCalls.find(pk->getCallId());
        if (itAux != activeCalls.end())
            throw cRuntimeError("Call id presents in the system");

        CallInfo *callInfo = new CallInfo;
        callInfo->dest = pk->getDestAddr();
        callInfo->callId = pk->getCallId();
        callInfo->sourceId = pk->getSourceId();

        activeCalls.insert(std::make_pair(callInfo->callId, callInfo));

        if (generateFlow) {
            callInfo->state = ON;
            callInfo->flowId++;
            callInfo->reservedBandwith = pk->getReserve();
            callInfo->usedBandwith = (uint64_t) usedBandwith->doubleValue();
            simtime_t delayAux = TimeOn->doubleValue();
            callInfo->startOn = delayAux;
            Packet *pkFlow = pk->dup();
            pkFlow->setSourceId(par("sourceId"));
            pkFlow->setDestinationId(callInfo->sourceId);
            pkFlow->setType(STARTFLOW);
            pkFlow->setReserve(callInfo->usedBandwith);
            pkFlow->setFlowId(callInfo->flowId);
            sprintf(pkname, "FlowOn-%d-to-%d-#%lud-Sid-%d", myAddress, pkFlow->getDestAddr(), callIdentifier,
                    this->getIndex());
            pkFlow->setName(pkname);
            CallEvents.insert(std::make_pair(simTime() + delayAux, callInfo));
            send(pkFlow, "out");
        }
        else
            callInfo->state = PASSIVE;
        scheduleAt(simTime() + callDuration->doubleValue(), pk);
    }
    else if (pk->getType() == RELEASE || pk->getType() == RELEASEDELAYED) {
        // Handle incoming packet
        for (auto it = CallEvents.begin(); it != CallEvents.end(); ++it) {
            if (it->second->callId == pk->getCallId()) {
                CallEvents.erase(it);
                break;
            }
        }

        // acumular el ancho de banda enviado
        auto it = activeCalls.find(pk->getCallId());
        if (it == activeCalls.end())
            throw cRuntimeError("Call id is not registered");

        auto itAccSend = sendBytes.find(it->second->dest);
        auto itAccRec = receivedBytes.find(it->second->dest);
        if (itAccSend == sendBytes.end())
            sendBytes[it->second->dest] = it->second->acumulateSend;
        else
            itAccSend->second += it->second->acumulateSend;
        if (itAccRec == receivedBytes.end())
            receivedBytes[it->second->dest] = it->second->acumulateRec;
        else
            itAccRec->second += it->second->acumulateRec;
        delete it->second;
        activeCalls.erase(it);

        if (pk->isSelfMessage())
            send(pk, "out");
        else {
            delete pk;
            if (hasGUI()) {
                getParentModule()->getDisplayString().setTagArg("i", 1, "green");
                getParentModule()->bubble("Arrived!");
            }
        }
    }
    else if (pk->getType() == BREAK) {

    }
    else // flow control packets, can be used to compute the statistics
    {
        auto it = activeCalls.find(pk->getCallId());
        if (it == activeCalls.end())
            throw cRuntimeError("Call id is not registered");
        CallInfo * callInfo = it->second;
        if (pk->getType() == ENDFLOW) {
            callInfo->acumulateRec += (callInfo->recBandwith * SIMTIME_DBL(simTime() - callInfo->startOnRec));
        }
        else if (pk->getType() == STARTFLOW) {
            callInfo->recBandwith = (uint64_t) pk->getReserve();
            callInfo->startOnRec = simTime();
        }

        delete msg;
    }
    rescheduleEvent();
}

