
#include "Filters.h"

Register_ResultFilter("messageAge", MessageAgeFilter);

void MessageAgeFilter::receiveSignal(cResultFilter *prev, simtime_t_cref t, cObject *object, cObject *details)
{
    if (auto msg = dynamic_cast<cMessage *>(object))
        fire(this, t, t - msg->getCreationTime(), details);
}
