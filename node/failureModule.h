//
// Copyright (C)Author: alfonso ariza quintana, Universidad de Malaga
//
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

#ifndef FAILUREMODULE_H_
#define FAILUREMODULE_H_
#include <omnetpp.h>
#include <map>
#include <vector>
#include "FailureEvent.h"

using namespace omnetpp;

#ifndef LinkId
typedef std::pair<int, int> LinkId;
#endif

class failureModule : public cSimpleModule
{
    enum CONFIGURATION_TYPE
    {
        FAILURE, RECOVERY
    };

    typedef std::map<simtime_t, Event> EventList;
    EventList eventList;
    cMessage *timer;
    simsignal_t eventSignal;

    cTopology *topo = nullptr;
    bool _hasKey(cXMLAttributeMap attributes, std::string key)
    {
        return attributes.find(key) != attributes.end();
    }
    simtime_t rdNormal(cXMLAttributeMap attributes);
    simtime_t rdUniform(cXMLAttributeMap attributes);
    simtime_t rdExponential(cXMLAttributeMap attributes);
    simtime_t rdConstant(double value);
    simtime_t rdConstant(cXMLAttributeMap attributes);
    simtime_t procDistribution(cXMLAttributeMap attributes);

    virtual void create(LinkId, cXMLElement *, int = 1);
    virtual std::pair<int, int> getNode(cXMLAttributeMap attributes);

    simtime_t createEvent(LinkId nodeId, cXMLAttributeMap attributes, const simtime_t &base, const CONFIGURATION_TYPE &);

public:
    virtual void initialize(int) override;
    virtual int numInitStages() const override
    {
        return 2;
    }
    virtual void handleMessage(cMessage *) override;
    failureModule();
    virtual ~failureModule();
    virtual void parser(cXMLElement *rootelement);
};

#endif /* FAILUREMODULE_H_ */
