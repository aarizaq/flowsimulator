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

    enum LinkState
    {
        UP, DOWN
    };
    enum CallState
    {
        UNKNOWN = -1, CALLUP, END
    };
// Structures used
    struct NeighborsPorts // allows identify the port using the address.
    {
        int port;
        LinkState state;
        simtime_t failureTime;
    };

// Information flow
    struct FlowInfo
    {
        int src;
        uint64_t flowId;
        uint64_t callId;
        uint64_t used;
        int port;
        int portInput;
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

    // vector of flows used by a call
    typedef std::vector<FlowInfo> FlowInfoVector;

    // call information
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

    // structure that contains the information related to a port, bandwidth, occupation, status ...
    struct PortData
    {
        uint64_t occupation;
        uint64_t nominalbw;
        uint64_t flowOcupation;
        uint64_t lastInfoOcupation;
        uint64_t lastInfoNominal;
        LinkState portStatus = UP;
        bool overload = false;
        std::set<FlowInfo> inputFlows; // flows not assigned to a call
        std::set<FlowInfo> outputtFlows; // flows not assigned to a call
    };



    //
    typedef std::map<int, int> RoutingTable; // destaddr -> gateindex
    typedef std::map<int, NeighborsPorts> NeighborsTable; // destaddr -> gateindex
    typedef std::map<int, uint64_t> SequenceTable;
    typedef std::map<uint64_t, CallInfo> CallInfoMap;

    // variables and containers
    int myAddress;

    std::map<int, int> sourceIdGate;
    CallInfoMap callInfomap;
    SequenceTable sequenceTable;
    uint64_t seqnum = 0;

    RoutingTable rtable;


    std::vector<PortData> portData;

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
    virtual bool preProcPacket(Packet *);
    virtual bool procStartFlow(Packet *, const int&, const int&);
    virtual bool procEndFlow(Packet *);
    virtual void postProc(Packet *, const int&, const int&, const int &);
protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
};

#endif
