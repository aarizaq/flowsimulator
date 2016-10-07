//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// Copyright (C) 1992-2008 Andras Varga
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//


//
// Represents a packet in the network.
//


struct LinkData
{
    int node;
    double residual;
    double time;
    double nominal;
    double mean;
    double min;
    double max;
    double actual;
}

enum PacketCode
{
    RESERVE = 1; // start a call
    RELEASE = 2; // end a call
    ACEPTED = 3; //
    REJECTED = 4;        
    STARTFLOW = 5;
    ENDFLOW = 6;
    FLOWCHANGE = 7;
    ACTUALIZE = 8;
    BREAK = 8;
    RESTORE = 10;
    RELEASEDELAYED = 11; // end a call
    CROUTEFLOWSTART = 12; // Change in the routing tables
    CROUTEFLOWEND = 13;
    RESERVEBK = 14;
    RELEASEBREAK = 15;
}

message PendingRelease
{
    uint64_t callId;
}

message RegisterMsg
{
    int sourceId;
}

packet Base
{
    int srcAddr;
    int destAddr;
    int hopCount;
    int sourceId;
    int destinationId;
    uint64_t sequence;
    int type @enum(PacketCode);
}

packet Packet extends Base
{
    int route[];
    double cost;
    int hopOrigin; // used in the change routing.
    int seqChange; // used in the change routing.
    uint64_t reserve;
    uint64_t flowId;
    uint64_t callId;
    uint64_t callIdBk;
    bool principal = true;
}
 
packet Actualize extends Packet
{
    LinkData linkData[];
    int type @enum(PacketCode);
   
}
