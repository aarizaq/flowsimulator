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
        bool disjointRoute = default(false);
        int sourceId = default(0);
        int destinationId = default(0);
        bool packetMode = default(false); // if true the source generates packets instead flow events
        
        volatile double callArrival @unit(s) = default(exponential(20s)); // time between generating packets
        volatile double callDuration @unit(s) = default(exponential(60s)); // time between generating packets
        volatile double callReserve @unit(bps) = default(0bps);  //
        
        bool generateFlow = default(true); // the source can generate calls
        // paramters of flows assined to calls          
        volatile double TimeOn @unit(s) = default(exponential(1s)); // time between generating packets
        volatile double TimeOff @unit(s) = default(exponential(1s)); // time between generating packets, can be 0 in in this case the flow can change the bandwith used
        volatile double usedBandwith @unit(bps);  //
        volatile int packetSize @unit(b) = default(1000b);  // The inter arrival packet is computed in base to usedBandwith

        // independent flows, flows that aren't associated to calls
        bool independentFlows = default(false);
        volatile double flowArrival @unit(s) = default(exponential(1s));
        volatile double flowDuration @unit(s) = default(exponential(1s));
        volatile double flowUsedBandwith @unit(bps);  //
        
        volatile int flowPacketSize @unit(b) = default(1000b);  // The inter arrival packet is computed in base to flowUsedBandwith              
        
        @display("i=block/browser");
        
        @signal[rcvdPk](type=cPacket);        
        @statistic[endToEndDelay](title="end-to-end delay"; source="messageAge(rcvdPk)"; unit=s; record=histogram,vector; interpolationmode=none);
    gates:
        input in;
        output out;
}


