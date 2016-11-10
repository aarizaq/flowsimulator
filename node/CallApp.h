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
#include "Packet_m.h"
#include "DijktraKShortest.h"

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
        SOURCEROUTINGNORMAL,
        DISJOINT,
        BACKUPROUTE,
        BACKUPROUTEKSH,
        SW,
        WS,
        SWFUZZY,
        WSFUZZY
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

    // identifiers
    static uint64_t callIdentifier;
    uint64_t flowIdentifier = 0;

    // event messages
    cMessage *generateCall = nullptr;
    cMessage *nextEvent = nullptr;
    cMessage *nextFlow = nullptr;

    long callCounter = 0; // number of call generated
    long callEstabilized  = 0; // number of call stabilized
    long callReceived = 0;
    long callRejected = 0;


    enum State
    {
        WAITCONFIRMATION = 0, ON, OFF, ACTIVE, PASSIVE, BACKUP, //this call doesn't create flows
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
        uint64_t callIdBk = 0;
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
        simtime_t startOn;
    };

    std::map<FlowIdentification,FlowStat> flowStatistics;

    typedef std::map<int, unsigned long int> SequenceTable;
    std::multimap<simtime_t, CallInfo*> CallEvents;
    std::multimap<simtime_t, FlowEvent*> FlowEvents;
    DijkstraFuzzy *dijFuzzy = nullptr;
    Dijkstra *dijkstra = nullptr;
    DijkstraKshortest *dijkstraks = nullptr;
    SequenceTable sequenceTable;

    std::map<int, long double> receivedBytes;
    std::map<int, long double> sendBytes;
    std::map<uint64_t, CallInfo*> activeCalls;
    std::map<uint64_t, CallInfo*> backupCalls;
    std::list<Packet *> listPendingRelease;

    bool check = false;

    std::vector<double> percentajesValues;
    std::vector<double> sanctionValues;

    static bool residual;
private:
    bool trace = false;
    void bytesTraceSend(const CallInfo *callInfo);
    void bytesTraceRec(const CallInfo *callInfo);
    void storeCallStatistics(const CallInfo *callInfo);
    static simsignal_t actualizationSignal;
public:
    CallApp();
    virtual ~CallApp();

protected:
    using cIListener::finish;

    virtual void checkAlg();
    virtual void readTopo();
    virtual void rescheduleEvent();
    virtual void procNextEvent();
    virtual void newCall();
    virtual void newFlow();
    virtual void newReserve(Packet *);
    virtual void newAccepted(Packet *);
    virtual void release(Packet *);
    virtual void procFlowPk(Packet *);
    virtual void procActualize(Actualize *);
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;
    virtual void receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj, cObject *details) override;


};

#endif /* CALLAPP_H_ */
