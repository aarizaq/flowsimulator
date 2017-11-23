//
// Copyright (C)Author: alfonso ariza quintana, Universidad de Malaga
// Copyright (C) 2017 Alfonso Ariza
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
#include <time.h>
#include "IRouting.h"

using namespace omnetpp;

/**
 * Generates traffic for the network.
 */

class CallApp : public cSimpleModule
{
protected:
    SimulationMode simulationMode = FLOWMODE;
private:
    // configuration
    int myAddress;
    std::vector<int> destAddresses;
    cPar *callArrival = nullptr;
    cPar *callReserve = nullptr;
    cPar *callDuration = nullptr;
    cPar *packetSize = nullptr;

    bool generateFlow;
    cPar *TimeOn = nullptr;
    cPar *TimeOff = nullptr;
    cPar *usedBandwith = nullptr;
    cPar *flowArrival = nullptr;
    cPar *flowDuration = nullptr;
    cPar *flowUsedBandwith = nullptr;
    cPar *flowPacketSize = nullptr;
    //
    time_t initTime;

    //double maxCapacity = 0;
    //bool useAlpha = false;



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

    long totalPkSent = 0;
    long totalPkRec = 0;


    enum State
    {
        WAITCONFIRMATION = 0, ON, OFF, ACTIVE, PASSIVE, BACKUP, //this call doesn't create flows
    };

    struct FlowData
    {
        uint64_t flowId = -1;
        uint64_t usedBandwith = 0;
        uint64_t reservedBandwith = 0;
        uint64_t recBandwith = 0;
        State state = OFF;
        State stateRec = OFF;
        simtime_t nextEvent;
        simtime_t startOn;
        simtime_t startOnRec;
        simtime_t interArrivalTime;
        uint64_t paketSize = 0;
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
        uint64_t callId = -1;
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
        simtime_t interArrivalTime;
        uint64_t paketSize = 0;
    };

    struct FlowEvent
    {
        int dest = -1;
        int destId = 0;
        uint64_t flowId = 0;
        uint64_t usedBandwith = 0;
        simtime_t startOn;
        simtime_t interArrivalTime;
        uint64_t paketSize;
    };

    std::map<FlowIdentification,FlowStat> flowStatistics;

    typedef std::map<int, unsigned long int> SequenceTable;
    std::multimap<simtime_t, CallInfo*> CallEvents;
    std::multimap<simtime_t, FlowEvent*> FlowEvents;
    std::multimap<simtime_t, CallInfo*> CallPacketsEvents;
    std::multimap<simtime_t, FlowEvent*> FlowPacketsEvents;

    SequenceTable sequenceTable;

    IRouting * routingModule = nullptr;


    std::map<int, long double> receivedBytes;
    std::map<int, long double> sendBytes;
    std::map<uint64_t, CallInfo*> activeCalls;
    std::map<uint64_t, CallInfo*> backupCalls;
    std::list<Packet *> listPendingRelease;

    //bool check = false;

    std::vector<double> percentajesValues;
    std::vector<double> sanctionValues;

    static bool residual;
private:
    bool trace = false;
    void bytesTraceSend(const CallInfo *callInfo);
    void bytesTraceRec(const CallInfo *callInfo);
    void storeCallStatistics(const CallInfo *callInfo);
    static simsignal_t actualizationSignal;
    static simsignal_t rcvdPk;

    virtual void   newCallFlow(CallInfo *callInfo, const uint64_t  &bw);
    virtual double startCallFlow(CallInfo *callInfo, Packet *pkFlow);
    virtual double endCallFlow(CallInfo *callInfo, Packet *pkFlow);
    virtual double startCallFlow(CallInfo *callInfo, Packet *pkFlow, FlowData & elem);
    virtual double endCallFlow(CallInfo *callInfo, Packet *pkFlow, FlowData & elem);
    virtual void newCallPacket(CallInfo *callInfo);

    virtual void  endFlow(FlowEvent *flowEvent);

public:
    CallApp();
    virtual ~CallApp();

protected:
    virtual void rescheduleEvent();
    virtual void procNextEvent();
    virtual void newCall();
    virtual void newFlow();
    virtual void newReserve(Packet *);
    virtual void newAccepted(Packet *);
    virtual void release(Packet *);
    virtual void procFlowPk(Packet *);
    virtual void getNodesAddress(const char *destAddressesPar, std::vector<int> &listAddres);
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;
};

#endif /* CALLAPP_H_ */
