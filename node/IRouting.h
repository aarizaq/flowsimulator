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

#ifndef NODE_IROUTING_H_
#define NODE_IROUTING_H_

class IRouting {
public:
    enum RoutingType {
        HOPBYHOP,
        SOURCEROUTING,
        SOURCEROUTINGNORMAL,
        DISJOINT,
        BACKUPROUTE,
        BACKUPROUTEKSH,
        SW,
        WS,
        SWFUZZY,
        WSFUZZY
     };
    virtual void setRoutingType(const RoutingType &) = 0;
    virtual RoutingType getRoutingType() = 0;
    virtual void getRoute(int, std::vector<int> &, std::vector<int> &) = 0;
};

#endif /* NODE_IROUTING_H_ */
