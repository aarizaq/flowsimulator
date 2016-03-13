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

using namespace omnetpp;

/**
 * Generates traffic for the network.
 */

class CallApp : public cSimpleModule
{
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
    cPar *usedBandwith;

    // state
    static uint64_t callIdentifier;
    cMessage *generateCall = nullptr;
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
        simtime_t nextEvent;
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
        uint64_t usedBandwith = 0;
        uint64_t reservedBandwith = 0;
        uint64_t recBandwith = 0;
        std::vector<FlowData> flowData; // multi flow calls
        long double acumulateSend = 0;
        long double acumulateRec = 0;
        simtime_t startOn;
        simtime_t startOnRec;
    };

    typedef std::map<int, unsigned long int> SequenceTable;
    std::map<simtime_t, CallInfo*> CallEvents;
    cMessage *nextEvent = nullptr;
    DijkstraFuzzy *dijFuzzy = nullptr;
    SequenceTable sequenceTable;

    std::map<int, long double> receivedBytes;
    std::map<int, long double> sendBytes;
    std::map<uint64_t, CallInfo*> activeCalls;

public:
    CallApp();
    virtual ~CallApp();

protected:
    virtual void readTopo();
    virtual void rescheduleEvent();
    virtual void initialize();
    virtual void handleMessage(cMessage *msg);
};

#endif /* CALLAPP_H_ */
