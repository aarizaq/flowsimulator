#ifndef FILTERS_H_
#define FILTERS_H_

#include <omnetpp.h>
using namespace omnetpp;

class MessageAgeFilter : public cObjectResultFilter
{
  public:
    virtual void receiveSignal(cResultFilter *prev, simtime_t_cref t, cObject *object, cObject *details) override;
};
#endif
