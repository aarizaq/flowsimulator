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
network NetNfsnet
{
    types:
        channel C extends DatarateChannel
        {
            delay = default(uniform(0.1ms, 1ms));
            datarate = default(10Mbps);
        }
    submodules:

        Seattle: Node {
            address = 1;
            @display("p=30,100");
        }

        SFrancisco: Node {
            address = 2;
        }

        LVegas: Node {
            address = 3;
        }

        LAngeles: Node {
            address = 4;
            @display("p=31,200");
        }

        Denver: Node {
            address = 5;
            @display("p=72,132");
        }

        Minneapolis: Node {
            address = 6;
            @display("p=87,84");
        }

        Dallas: Node {
            address = 7;
            @display("p=122,156");
        }

        Chicago: Node {
            address = 8;
            @display("p=122,84");
        }

        Washinton: Node {
            address = 9;
            @display("p=153,100");
        }

        Boston: Node {
            address = 10;
            @display("p=203,65");
        }

        NY: Node {
            address = 11;
            @display("p=188,100");
        }

        Miami: Node {
            address = 12;
            @display("p=181,156");
        }

    connections:
        Seattle.port++ <--> C <--> SFrancisco.port++;
        Seattle.port++ <--> C <--> Denver.port++;
        Seattle.port++ <--> C <--> Minneapolis.port++;

        SFrancisco.port++ <--> C <--> LVegas.port++;
        SFrancisco.port++ <--> C <--> LAngeles.port++;

        LVegas.port++ <--> C <--> LAngeles.port++;
        LVegas.port++ <--> C <--> Denver.port++;
        LVegas.port++ <--> C <--> Dallas.port++;

        Denver.port++ <--> C <--> Minneapolis.port++;
        Denver.port++ <--> C <--> Dallas.port++;

        Minneapolis.port++ <--> C <--> Dallas.port++;
        Minneapolis.port++ <--> C <--> Chicago.port++;

        Dallas.port++ <--> C <--> Washinton.port++;
        Dallas.port++ <--> C <--> Miami.port++;

        Chicago.port++ <--> C <--> Washinton.port++;
        Chicago.port++ <--> C <--> Boston.port++;



        Washinton.port++ <--> C <--> Boston.port++;
        Washinton.port++ <--> C <--> NY.port++;
        Washinton.port++ <--> C <--> Miami.port++;

        Boston.port++ <--> C <--> NY.port++;
        NY.port++ <--> C <--> Miami.port++;
}

