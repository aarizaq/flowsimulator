/*
 * DataTypes.h
 *
 *  Created on: Mar 30, 2016
 *      Author: alfonso
 */

#ifndef FLOWDATATYPES_H_
#define FLOWDATATYPES_H_

enum LinkState
{
    UP, DOWN
};
enum CallState
{
    UNKNOWN = -1, CALLUP, END
};

enum FlowAdmisionModes
{
    DISCARD, // discard the flow if not enough bandwith
    FINITEQUEUE, // the bandwidth is shared between the nodes
    INFINITEQUEUE
};


// Structures used
struct NeighborsPorts // allows identify the port using the address.
{
    int port;
    LinkState state;
    simtime_t failureTime;
};

// Information flow
class FlowIdentification
{
    int _src;
    uint64_t _flowId;
    uint64_t _callId;
public:
    int & src()
    {
        return _src;
    }
    uint64_t & flowId()
    {
        return _flowId;
    }
    uint64_t & callId()
    {
        return _callId;
    }

    FlowIdentification & operator =(const FlowIdentification& b)
    {
        _src = b._src;
        _flowId = b._flowId;
        _callId = b._callId;
        return *this;
    }

    bool operator <(const FlowIdentification& b) const
    {
        if (_src == b._src) {
            if (_callId == b._callId)
                return _flowId < b._flowId;
            else
                return _callId < b._callId;
        }
        else
            return _src < b._src;
    }
    bool operator >(const FlowIdentification& b) const
    {
        if (_src == b._src) {
            if (_callId == b._callId)
                return _flowId > b._flowId;
            else
                return _callId > b._callId;
        }
        else
            return _src > b._src;
    }
    bool operator ==(const FlowIdentification& b) const
    {
        return (_src == b._src) && (_flowId == b._flowId) && (_callId == b._callId);
    }
};

struct FlowInfo
{
    FlowIdentification identify;
    uint64_t used;
    int port;
    int portInput;
    int dest;
    int destId;
    std::vector<int> sourceRouting;
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
    int applicationId1 = -1;
    int applicationId2 = -1;
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
};




#endif /* DATATYPES_H_ */
