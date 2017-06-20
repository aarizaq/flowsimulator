//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// Copyright (C) 1992-2008 Andras Varga
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//

package networks;

import node.Node;
import ned.DatarateChannel;

//
// A ten-node network topology
//
network Net3
{
    types:
        channel C extends DatarateChannel
        {
            delay = uniform(0.1ms, 1ms);
            datarate = 1Gbps;
        }
    submodules:
        rte[3]: Node {
            address = index;
        }
    connections:
        rte[0].port++ <--> C <--> rte[1].port++;
        rte[1].port++ <--> C <--> rte[2].port++;
}

