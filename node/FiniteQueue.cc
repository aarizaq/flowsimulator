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

#include "FiniteQueue.h"

Register_Class(FiniteQueue);

FiniteQueue::FiniteQueue()
{
    // TODO Auto-generated constructor stub

}

FiniteQueue::~FiniteQueue()
{
    // TODO Auto-generated destructor stub
}

bool FiniteQueue::startShare(std::vector<FlowInfo *> &listOut, std::vector<FlowInfo *> &listIn, const uint64_t &bw)
{
    uint64_t request = 0;
    for (auto elem : listIn) {
        request += elem->used;
    }
    if (request < bw)
        return false; // nothing to do
    double p = (double) bw / (double) request;
    if (p > 1)
        p = 1.0;
    for (auto elem : listIn) {
        auto it = listOut.begin();
        while (it != listOut.end()) {
            if ((*it)->identify == elem->identify)
                break;
            ++it;
        }
        if (it != listOut.end())
            (*it)->used = elem->used * p;
    }
    return true;
}

bool FiniteQueue::endShare(std::vector<FlowInfo *> &listOut, std::vector<FlowInfo *> &listIn, const uint64_t &bw)
{
    uint64_t request = 0;
    for (auto elem : listIn) {
        request += elem->used;
    }

    double p = (double) bw / (double) request;
    if ( bw >= request)
        p = 1.0;
    for (auto elem : listIn) {
        auto it = listOut.begin();
        while (it != listOut.end()) {
            if ((*it)->identify == elem->identify)
                break;
            ++it;
        }
        (*it)->used = elem->used * p;
    }
    return  bw >= request;
}
