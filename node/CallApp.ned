//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// Copyright (C) 1992-2008 Andras Varga
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//

package node;

//
// Application model to generate traffic for the network.
//
simple CallApp like IApp
{
    parameters:
        int address;  // local node address
        string destAddresses = default("");  // destination addresses
        int sourceId = default(0);
        int destinationId = default(0);
        string RoutingType @enum("HopByHop","SourceRouting","Disjoint","BackupRouting") = default("HopByHop");
        volatile double callArrival @unit(s) = default(exponential(20s)); // time between generating packets
        volatile double callDuration @unit(s) = default(exponential(60s)); // time between generating packets
        volatile double callReserve @unit(bps) = default(0bps);  //
        
        bool generateFlow = default(true); // the source can generate calls
        // paramters of flows assined to calls          
        volatile double TimeOn @unit(s) = default(exponential(1s)); // time between generating packets
        volatile double TimeOff @unit(s) = default(exponential(1s)); // time between generating packets, can be 0 in in this case the flow can change the bandwith used
        volatile double usedBandwith @unit(bps);  //
        
        // independent flows, flows that aren't associated to calls
        bool independentFlows = default(false);
        volatile double flowArrival @unit(s) = default(exponential(1s));
        volatile double flowDuration @unit(s) = default(exponential(1s));
        volatile double flowUsedBandwith @unit(bps);  //
        
         
         
        @display("i=block/browser");
    gates:
        input in;
        output out;
}


