/*
 * FailureEvent.h
 *
 *  Created on: Mar 3, 2016
 *      Author: alfonso
 */

#ifndef FAILUREEVENT_H_
#define FAILUREEVENT_H_
#include <omnetpp.h>
using namespace omnetpp;

typedef std::pair<int, int> LinkId;

enum EVENT_TYPE
{
    NODE_FAILURE_EV, LINK_FAILURE_EV, NODE_RECOVERY_EV, LINK_RECOVERY_EV

};

struct Event : public cObject
{
    EVENT_TYPE type;
    LinkId linkId;
    bool operator ==(const Event& o) const
    {
        return type == o.type && linkId == o.linkId;
    }
};

#endif /* FAILUREEVENT_H_ */
