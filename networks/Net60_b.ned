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
import node.FailureModule;

//
// A network topology commonly known as the "NTT backbone".
//
network Net60_b
{
    types:
        channel C extends DatarateChannel
        {
            parameters:
                delay = default(0.1ms);
                datarate = default(1Gbps);
        }
        channel C2 extends DatarateChannel
        {
            parameters:
                delay = default(0.1ms);
                datarate = default(0.5Gbps);
        }
        
    submodules:
        rte[57]: Node {
            parameters:
                address = index;
        }
        failure: FailureModule {
            @display("p=19,17;is=s");
        }

    connections allowunconnected:
        rte[0].port++ <--> C <--> rte[1].port++;
        rte[1].port++ <--> C <--> rte[2].port++;
        rte[1].port++ <--> C <--> rte[4].port++;
        rte[1].port++ <--> C2 <--> rte[7].port++;
        rte[3].port++ <--> C2 <--> rte[4].port++;
        rte[4].port++ <--> C2 <--> rte[5].port++;
        rte[4].port++ <--> C <--> rte[7].port++;
        rte[5].port++ <--> C <--> rte[6].port++;
        rte[5].port++ <--> C <--> rte[10].port++;
        rte[6].port++ <--> C <--> rte[7].port++;
        rte[6].port++ <--> C <--> rte[9].port++;
        rte[7].port++ <--> C2 <--> rte[8].port++;
        rte[7].port++ <--> C2 <--> rte[12].port++;
        rte[9].port++ <--> C <--> rte[11].port++;
        rte[10].port++ <--> C <--> rte[11].port++;
        rte[10].port++ <--> C <--> rte[13].port++;
        rte[11].port++ <--> C2 <--> rte[12].port++;
        rte[11].port++ <--> C2 <--> rte[14].port++;
        rte[12].port++ <--> C <--> rte[15].port++;
        rte[13].port++ <--> C <--> rte[18].port++;
        rte[14].port++ <--> C2 <--> rte[15].port++;
        rte[14].port++ <--> C2 <--> rte[22].port++;
        rte[15].port++ <--> C2 <--> rte[16].port++;
        rte[16].port++ <--> C <--> rte[17].port++;
        rte[16].port++ <--> C <--> rte[23].port++;
        rte[18].port++ <--> C2 <--> rte[19].port++;
        rte[18].port++ <--> C2 <--> rte[21].port++;
        rte[19].port++ <--> C <--> rte[20].port++;
        rte[19].port++ <--> C <--> rte[26].port++;
        rte[20].port++ <--> C <--> rte[21].port++;
        rte[20].port++ <--> C2 <--> rte[24].port++;
        rte[20].port++ <--> C <--> rte[25].port++;
        rte[21].port++ <--> C <--> rte[22].port++;
        rte[21].port++ <--> C2 <--> rte[23].port++;
        rte[21].port++ <--> C2 <--> rte[24].port++;
        rte[23].port++ <--> C2 <--> rte[24].port++;
        rte[24].port++ <--> C <--> rte[28].port++;
        rte[25].port++ <--> C <--> rte[27].port++;
        rte[26].port++ <--> C <--> rte[32].port++;
        rte[27].port++ <--> C <--> rte[29].port++;
        rte[27].port++ <--> C <--> rte[30].port++;
        rte[27].port++ <--> C2 <--> rte[32].port++;
        rte[28].port++ <--> C <--> rte[29].port++;
        rte[28].port++ <--> C2 <--> rte[31].port++;
        rte[29].port++ <--> C <--> rte[31].port++;
        rte[30].port++ <--> C <--> rte[36].port++;
        rte[31].port++ <--> C <--> rte[33].port++;
        rte[32].port++ <--> C2 <--> rte[34].port++;
        rte[33].port++ <--> C2 <--> rte[37].port++;
        rte[34].port++ <--> C <--> rte[35].port++;
        rte[34].port++ <--> C <--> rte[40].port++;
        rte[35].port++ <--> C <--> rte[36].port++;
        rte[35].port++ <--> C <--> rte[39].port++;
        rte[36].port++ <--> C2 <--> rte[37].port++;
        rte[37].port++ <--> C2 <--> rte[38].port++;
        rte[38].port++ <--> C <--> rte[39].port++;
        rte[38].port++ <--> C <--> rte[42].port++;
        rte[39].port++ <--> C <--> rte[40].port++;
        rte[39].port++ <--> C2 <--> rte[41].port++;
        rte[40].port++ <--> C <--> rte[43].port++;
        rte[40].port++ <--> C <--> rte[44].port++;
        rte[40].port++ <--> C <--> rte[52].port++;
        rte[41].port++ <--> C2 <--> rte[42].port++;
        rte[41].port++ <--> C2 <--> rte[43].port++;
        rte[42].port++ <--> C2 <--> rte[43].port++;
        rte[43].port++ <--> C <--> rte[46].port++;
        rte[44].port++ <--> C <--> rte[45].port++;
        rte[44].port++ <--> C <--> rte[47].port++;
        rte[45].port++ <--> C <--> rte[46].port++;
        rte[46].port++ <--> C <--> rte[49].port++;
        rte[47].port++ <--> C2 <--> rte[48].port++;
        rte[48].port++ <--> C <--> rte[50].port++;
        rte[48].port++ <--> C2 <--> rte[51].port++;
        rte[49].port++ <--> C <--> rte[50].port++;
        rte[50].port++ <--> C <--> rte[53].port++;
        rte[51].port++ <--> C2 <--> rte[52].port++;
        rte[51].port++ <--> C2 <--> rte[53].port++;
        rte[52].port++ <--> C <--> rte[53].port++;
        rte[52].port++ <--> C <--> rte[54].port++;
        rte[53].port++ <--> C <--> rte[56].port++;
        rte[54].port++ <--> C2 <--> rte[55].port++;
        rte[55].port++ <--> C <--> rte[56].port++;
}

