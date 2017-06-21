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

package node;

//
// TODO auto-generated module
//
simple RoutingModule like IRouting
{
    parameters:
    // Routing type
        string RoutingType @enum("HopByHop","SourceRouting","Disjoint","BackupRouting","SourceRoutingFuzzy","Sw","Ws","SwFuzzy","WsFuzzy") = default("HopByHop");
        bool useHyperbolic = default(false); // use lineal or hyperbolic in SourceRouting or SourceRoutingFuzzy type               
        bool instValue = default(false); // use instantaneous value instead of the mean value
        bool fuzzyCorregido = default(false); 
                
        string levelsValues = default(""); // number of level in the asignation of cost, "30 60 90" changes in the cost at 30% residual, 60 % residual and 90% residual    
        string santionValues = default("1 5 10");// only used if levelsValues has values, in this case the size of this must be equal in both 
        
        bool useAlpha = default(false); // use alpha to correct he diferences in the link nominal bandwidth
    gates:
        input in;
        output out;
}
