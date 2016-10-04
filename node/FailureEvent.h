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
    std::string info() const override
    {
        std::stringstream out;

        out << "Type: ";
        if (type == NODE_FAILURE_EV)
            out << "Node Failure  ";
        else  if (type == LINK_FAILURE_EV)
            out << "Link Failure  ";
        else if (type == NODE_RECOVERY_EV)
            out << "Node recovery  ";
        else  if (type == LINK_RECOVERY_EV)
            out << "Link recovery  ";

        out << " Node origin : " << linkId.first;
        out << " Node end : " << linkId.second;
        return out.str();
    }
};

#endif /* FAILUREEVENT_H_ */
