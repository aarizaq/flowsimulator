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

#ifndef BASEFLOWDISTRIBUTION_H_
#define BASEFLOWDISTRIBUTION_H_

#include <omnetpp.h>
#include "FlowDataTypes.h"

using namespace omnetpp;

class BaseFlowDistribution : public cObject
{
public:
    virtual bool startShare(std::vector<FlowInfo *> &, std::vector<FlowInfo *> &, const uint64_t &) = 0;
    virtual bool endShare(std::vector<FlowInfo *> &, std::vector<FlowInfo *> &, const uint64_t &) = 0;
};

#endif /* BASEFLOWDISTIRBUTION_H_ */
