#ifndef FLOWROUTING_H_
#define FLOWROUTING_H_

#include <map>
#include <vector>
#include <set>
#include <omnetpp.h>
#include "Packet_m.h"
#include "DijktraKShortest.h"
#include "FlowDataTypes.h"

#include "BaseFlowDistribution.h"

using namespace omnetpp;

/**
 * Demonstrates call and flow simulations using static routing, utilizing the cTopology class.
 */
class FlowRouting : public cSimpleModule, public cListener
{
private:
    SimulationMode simulationMode = FLOWMODE;

    // configuration parameters
    FlowAdmisionModes flowAdmisionMode = DISCARD; // if true and the bandwidth is busy the flow is lost, if false, the flows share in proportional the bandwidth
    double reserveCall = 0; // quantity that determine if flows not assigned to call must be accepted or not, 0 implies that they must accept it of possible, not reservation, 1 only accept call flows
    double reserveFlows = 0; //similar to previous, but with the bandwidth used in the port

    long callRequest = 0;
    long callLost = 0;

    //
    typedef std::map<int, int> RoutingTable; // destaddr -> gateindex
    typedef std::map<int, NeighborsPorts> NeighborsTable; // destaddr -> gateindex
    typedef std::map<int, uint64_t> SequenceTable;
    typedef std::map<uint64_t, CallInfo> CallInfoMap;
    typedef std::map<FlowIdentification,FlowInfo> FlowInfoMap;
    typedef std::vector<PortData> PortDataArray;

    // variables and containers
    int myAddress;

    std::map<int, int> sourceIdGate;
    std::map<int, int> inverseSourceIdGate;
    CallInfoMap callInfomap;
    SequenceTable sequenceTable;
    uint64_t seqnum = 0;

    RoutingTable rtable;

    PortDataArray portDataArray;

    FlowInfoVector pendingFlows;
    FlowInfoVector delayedFlows;
    FlowInfoMap inputFlows; // flows not assigned to a call
    FlowInfoMap outputFlows; // flows not assigned to a call

    NeighborsTable neighbors;

    simsignal_t dropSignal;
    simsignal_t outputIfSignal;
    int localOutSize = 0;

    cMessage *actualizeTimer = nullptr;
    //cMessage *computeBwTimer = nullptr;

    static simsignal_t actualizationSignal;

    DijkstraKshortest * dijkstra = nullptr;

    simtime_t lastTimeActualize;

    static simsignal_t eventSignal;

    bool inmediateNotificationLink = false;

    BaseFlowDistribution * flowDist = nullptr;

    simtime_t computationInterval = 5;
    ~FlowRouting();
    virtual void receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj, cObject *) override;

    virtual bool actualize(Actualize * = nullptr);
    virtual bool actualizePercentaje();
    virtual void processLinkEvents(cObject *msg);
    virtual void getListFlowsToModifyStartFlow(const int &, std::vector<FlowInfo *> &, std::vector<FlowInfo *> &);
    virtual bool procReserve(Packet *msg, int&, int&);
    virtual bool getForwarPortFreeFlow(Packet *msg, int&);
    virtual void checkPendingList();
    virtual bool flodAdmision(const uint64_t &reserve, FlowInfo *flowInfoInputPtr, FlowInfo *flowInfoOutputPtr, const int & portForward, const int & portInput, PacketCode codeStart);
    virtual bool sendChangeFlow(FlowInfo &, const int &);
    virtual void procActualize(Actualize *pkt);
    virtual void procBroadcast(Base *pkt);
    virtual bool preProcPacket(Packet *);
    virtual bool procStartFlow(Packet *, const int&, const int&);
    virtual bool procFlowChange(Packet *, const int&, const int&);
    virtual bool procEndFlow(Packet *);
    virtual bool procEndFlowStoreAndForward(Packet *);
    virtual bool procEndFlowLost(Packet *);

    virtual void postProc(Packet *, const int&, const int&, const int &);
    virtual void computeUsedBw();
    virtual void recordOccupation(PortData &port, const ChangeBw &val);
protected:
    using cIListener::finish;
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;
};

#endif
