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
// Models a router.
//
simple FlowForwarding like IForwarding
{
    parameters:
        volatile double actualizeState @unit(s)= default(60s);
        volatile double breakRelease  @unit(s)= default(0s);
        bool actualizeWithSignals = default(false);
        bool packetMode = default(false); // if true the source generates packets instead flow events
        double minimumTimeActualize @unit(s) = default(60s); // minimum time between two actualization messages
        string flowClass @enum("","Discard","StoreAndForward","FiniteQueue") = default(""); // como se comporta el router cuando est� lleno el canal, como tira los datos
        int storeAndForwardSize = default(-1);
        bool actPercentage = default(false);
        double percentage = default(0.1);
        volatile double breakDelay @unit(s) = default(0s);
         
        @display("i=block/switch");
        @signal[drop](type="long");
        @signal[outputIf](type="long");
        @signal[actualizationSignal](type=Packet);
        @signal[actualizationPortsSignal](type="bool");
        @signal[changeRoutingTableSignal];
        @statistic[drop](title="dropped packet byte length";unit=bytes;record=vector?,count,sum;interpolationmode=none);
        @statistic[outputIf](title="output interface for each routed packet";record=histogram;interpolationmode=none);
    gates:
        input in[];
        output out[];
        input localIn[];
        output localOut[];
        
        output toRouting;
        input fromRouting;
        
}


