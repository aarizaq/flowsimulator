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
simple FlowRouting
{
    parameters:
        volatile double actualizeState @unit(s)= default(60s+uniform(-0.1s,0.1s));
        volatile double breakRelease  @unit(s)= default(0);
        double minimumTimeActualize @unit(s) = default(10s); // minimum time between two actualization messages
        string flowClass = default("");
        @display("i=block/switch");
        @signal[drop](type="long");
        @signal[outputIf](type="long");
        @statistic[drop](title="dropped packet byte length";unit=bytes;record=vector?,count,sum;interpolationmode=none);
        @statistic[outputIf](title="output interface for each routed packet";record=histogram;interpolationmode=none);
    gates:
        input in[];
        output out[];
        input localIn[];
        output localOut[];
}


