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
// A "Node" consists of a Routing plus an App module which represents the
// aggregate traffic of the subnets connected to that router.
//
module Node
{
    parameters:
        int address;
        string appType;
        int numSources = default(1);
        @node();
        @display("i=misc/node_vs,gold");
    gates:
        inout port[];
    submodules:
        app[numSources]: <appType> like IApp {
            parameters:
                address = address;
                @display("p=140,60,row");
        }
        routing: <default("RoutingModule")> like IRouting {
            parameters:
                 @display("p=80,130");
        }
        
        forwarding: <default("FlowForwarding")> like IForwarding {
            parameters:
                @display("p=140,130");
            gates:
                in[sizeof(port)];
                out[sizeof(port)];
        }
        queue[sizeof(port)]: L2Queue {
            parameters:
                @display("p=80,200,row");
        }
    connections:
        for i=0..numSources-1 {
            forwarding.localOut++ --> app[i].in;
            forwarding.localIn++ <-- app[i].out;
        }
        for i=0..sizeof(port)-1 {
            forwarding.out[i] --> queue[i].in;
            forwarding.in[i] <-- queue[i].out;
            queue[i].line <--> port[i];
        }
        forwarding.toRouting --> routing.in;
        forwarding.fromRouting <-- routing.out;
}


