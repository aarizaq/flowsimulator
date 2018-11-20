//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#ifndef NODE_ROUTINGMODULE_H_
#define NODE_ROUTINGMODULE_H_

#include <omnetpp/csimplemodule.h>

#include "Dijkstra.h"
#include "DijkstraFuzzy.h"
#include "DijktraKShortestFuzzy.h"
#include "Packet_m.h"
#include "DijktraKShortest.h"
#include "IForwarding.h"
#include "IRouting.h"

using namespace omnetpp;

class RoutingModule: public cSimpleModule, public IRouting, protected cListener {
public:
    RoutingModule();
    virtual ~RoutingModule();
    virtual void getPairRoutes(const int &, std::vector<int> &, std::vector<int> &, const bool & =false) override;
    virtual void setRoutingType(const IRouting::RoutingType & a) override;
    virtual IRouting::RoutingType getRoutingType() override;
    virtual void receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj, cObject *details) override;
    virtual void receiveSignal(cComponent *source, simsignal_t signalID, bool change, cObject *details) override;

protected:
    int myAddress;
    double maxCapacity = 0;
    bool useAlpha = false;

    cMessage *nextAct;
    IForwarding *forwarding;

    static simsignal_t actualizationSignal;
    static simsignal_t actualizationPortsSignal;
    static simsignal_t changeRoutingTableSignal;

    std::vector<int> destination;

    bool actualizeForwarding = false;

    DijkstraFuzzy *dijFuzzy = nullptr;
    Dijkstra *dijkstra = nullptr;
    DijkstraKshortest *dijkstraks = nullptr;
    DijkstraKshortestFuzzy *dijkstraksFuzzy = nullptr;

    bool residual = false;
    bool normalized = true;

    std::vector<double> percentajesValues;
    std::vector<double> sanctionValues;

    RoutingType rType;
    virtual void readTopo();
    virtual int numInitStages() const override {return 2;}
    virtual void initialize(int) override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void procActualize(Actualize *pkt);

};

#endif /* NODE_ROUTINGMODULE_H_ */
