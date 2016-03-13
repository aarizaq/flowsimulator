#ifndef FLOWROUTING_H_
#define FLOWROUTING_H_

#include <map>
#include <vector>
#include <set>
#include <omnetpp.h>
#include "Packet_m.h"
#include "DijktraKShortest.h"

using namespace omnetpp;

/**
 * Demonstrates call and flow simulations using static routing, utilizing the cTopology class.
 */
class FlowRouting : public cSimpleModule, public cListener
{
private:
    int myAddress;
    enum LinkState
    {
        UP, DOWN
    };
    enum CallState
    {
        UNKNOWN = -1, CALLUP, END
    };

    struct NeighborsPorts
    {
        int port;
        LinkState state;
        simtime_t failureTime;
    };

    std::map<int, int> sourceIdGate;

    typedef std::map<int, int> RoutingTable; // destaddr -> gateindex
    typedef std::map<int, NeighborsPorts> NeighborsTable; // destaddr -> gateindex
    typedef std::vector<uint64_t> Ocupation;
    typedef std::map<int, uint64_t> SequenceTable;

    struct FlowInfo
    {
        int src;
        uint64_t flowId;
        uint64_t callId;
        uint64_t used;
        int port;
        bool operator <(const FlowInfo& b) const
        {
            if (src == b.src) {
                if (callId == b.callId)
                    return flowId < b.flowId;
                else
                    return callId < b.callId;
            }
            else
                return src < b.src;
        }
        bool operator >(const FlowInfo& b) const
        {
            if (src == b.src) {
                if (callId == b.callId)
                    return flowId > b.flowId;
                else
                    return callId > b.callId;
            }
            else
                return src > b.src;
        }
        bool operator ==(const FlowInfo& b) const
        {
            return (src == b.src) && (flowId == b.flowId) && (callId == b.callId);
        }
    };

    typedef std::vector<FlowInfo> FlowInfoVector;


    struct CallInfo
    {
        int node1; // address origin of the call
        int node2; // address of the destination of the call
        uint64_t reserve;
        // direct flows source --> destination
        // inverse flows destination --> source
        int port1; // forwarding direct flows-receiving inverse flows
        int port2; // forwarding inverse flows-receiving direct flows
        int applicationId = -1;
        CallState state = UNKNOWN;
        FlowInfoVector inputFlows;
        FlowInfoVector outputFlows;
    };

    typedef std::map<uint64_t, CallInfo> CallInfoMap;
    CallInfoMap callInfomap;
    SequenceTable sequenceTable;
    uint64_t seqnum = 0;

    RoutingTable rtable;
    Ocupation ocupation;
    Ocupation nominalbw;
    Ocupation flowOcupation;
    Ocupation lastInfoOcupation;
    Ocupation lastInfoNominal;
    std::vector<LinkState> portStatus;
    FlowInfoVector pendingFlows;

    NeighborsTable neighbors;

    simsignal_t dropSignal;
    simsignal_t outputIfSignal;
    int localOutSize = 0;

    cMessage *actualizeTimer;

    DijkstraKshortest * dijkstra = nullptr;

    simtime_t lastTimeActualize;

    simsignal_t eventSignal;

    bool inmediateNotificationLink = false;

    ~FlowRouting();
    virtual void receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj, cObject *) override;

    virtual bool actualize(Actualize * = nullptr);
    virtual void processLinkEvents(cObject *msg);
    virtual void procReserve(Packet *msg, int&, int&);
    virtual void checkPendingList();
    virtual void procActualize(Actualize *pkt);
    virtual void procBroadcast(Base *pkt);
protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
};

#endif
