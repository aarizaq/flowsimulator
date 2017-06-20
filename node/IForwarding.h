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

#ifndef NODE_IFORWARDING_H_
#define NODE_IFORWARDING_H_
#include "FlowDataTypes.h"
class IForwarding {
public:
    virtual unsigned int getNumPorts() = 0;
    virtual void getPorts(std::vector<PortData> &) = 0;
    virtual PortData getPort(const int &) = 0;
    virtual int getPortNeighbor(const int &) = 0;
    virtual int getNeighborConnectPort(const int &) const = 0;
    virtual void getRoutingTable(std::map<int, int> &) = 0;
    virtual int getRouting(const int&) = 0;
    virtual int getAddress() = 0;
    virtual void setRoute(const int &,const int &) = 0;
};

#endif /* NODE_IFORWARDING_H_ */
