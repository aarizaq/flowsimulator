//
// Copyright (C)Author: alfonso ariza quintana, Universidad de Malaga
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//

#ifndef CALLAPP_H_
#define CALLAPP_H_

#include <vector>
#include <omnetpp.h>
#include "DijkstraFuzzy.h"
#include "FlowDataTypes.h"

using namespace omnetpp;

/**
 * Generates traffic for the network.
 */

class CallApp : public cSimpleModule, public cListener
{
protected:
    enum RoutingType {
        HOPBYHOP,
        SOURCEROUTING,
        DISJOINT
     };
    RoutingType rType;
private:
    // configuration
    int myAddress;
    std::vector<int> destAddresses;
    cPar *callArrival = nullptr;
    cPar *callReserve = nullptr;
    cPar *callDuration = nullptr;

    bool generateFlow;
    cPar *TimeOn = nullptr;
    cPar *TimeOff = nullptr;
    cPar *usedBandwith = nullptr;
    cPar *flowArrival = nullptr;
    cPar *flowDuration = nullptr;
    cPar *flowUsedBandwith = nullptr;

    // state
    static uint64_t callIdentifier;
    uint64_t flowIdentifier = 0;
    cMessage *generateCall = nullptr;
    cMessage *nextEvent = nullptr;
    cMessage *nextFlow = nullptr;

    long callCounter = 0;
    long callReceived = 0;

    enum State
    {
        WAITCONFIRMATION = 0, ON, OFF, ACTIVE, PASSIVE //this call doesn't create flows
    };

    struct FlowData
    {
        uint64_t flowId;
        uint64_t usedBandwith = 0;
        uint64_t reservedBandwith = 0;
        uint64_t recBandwith = 0;
        State state = OFF;
        State stateRec = OFF;
        simtime_t nextEvent;
        simtime_t startOn;
        simtime_t startOnRec;
    };

    struct FlowStat
    {
        uint64_t used = 0;
        uint64_t total = 0;
        simtime_t startOn;
        simtime_t startOnRec;
    };

    struct CallInfo
    {
        int dest = -1;
        int sourceId = 0;
        uint64_t callId;
        uint64_t flowId = 0;
        State state = WAITCONFIRMATION;
        State stateRec = OFF;
        uint64_t usedBandwith = 0;
        uint64_t reservedBandwith = 0;
        uint64_t recBandwith = 0;
        std::vector<FlowData> flowData; // multi flow calls
        long double acumulateSend = 0;
        long double acumulateRec = 0;
        simtime_t startOn;
        simtime_t startOnRec;
    };

    struct FlowEvent
    {
        int dest = -1;
        int destId = 0;
        uint64_t flowId = 0;
        uint64_t usedBandwith = 0;
    };

    std::map<FlowIdentification,FlowStat> flowStatistics;

    typedef std::map<int, unsigned long int> SequenceTable;
    std::multimap<simtime_t, CallInfo*> CallEvents;
    std::multimap<simtime_t, FlowEvent*> FlowEvents;
    DijkstraFuzzy *dijFuzzy = nullptr;
    SequenceTable sequenceTable;

    std::map<int, long double> receivedBytes;
    std::map<int, long double> sendBytes;
    std::map<uint64_t, CallInfo*> activeCalls;

    bool check = false;

    static bool residual;
private:
    bool trace = false;
    void bytesTraceSend(const CallInfo *callInfo);
    void bytesTraceRec(const CallInfo *callInfo);
public:
    CallApp();
    virtual ~CallApp();

protected:
    using cIListener::finish;
    virtual void checkAlg();
    virtual void readTopo();
    virtual void rescheduleEvent();
    virtual void procNextEvent();
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;
    virtual void receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj, cObject *details) override;


};

#endif /* CALLAPP_H_ */
