//
// Copyright (C)Author: alfonso ariza quintana, Universidad de Malaga
// Copyright (C) 2017 Alfonso Ariza
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//

#ifdef _MSC_VER
#pragma warning(disable:4786)
#endif

#include <iostream>
#include <fstream>
#include "CallApp.h"
#include <cinttypes>
#include "DijkstraFuzzy.h"
#include "DijktraKShortest.h"
#include "DijktraKShortestFuzzy.h"

uint64_t CallApp::callIdentifier = 1;
bool CallApp::residual = false;
simsignal_t CallApp::actualizationSignal = registerSignal("actualizationSignal");
simsignal_t CallApp::rcvdPk = registerSignal("rcvdPk");

//#define ONLYONECALL

Define_Module(CallApp);

void CallApp::checkDijktra()
{
#if 0
    DijkstraFuzzy dijFuzzy;
    std::ofstream myfile;
    DijkstraKshortestFuzzy dijKFuzzy(50);

    dijFuzzy.setRoot(myAddress);
    dijFuzzy.setHasFindDisjoint(true);
    dijFuzzy.setAlpha(0.6);

    dijKFuzzy.setRoot(myAddress);
    dijKFuzzy.setAlpha(0.6);

    struct PairPaths {
        std::vector<NodeId> path1;
        std::vector<NodeId> path2;
        DijkstraFuzzy::FuzzyCost cost1;
    };

    std::map<NodeId, PairPaths> kRoutes;
    std::map<NodeId, PairPaths> pRoutes;

#if 0
    dijFuzzy.addLink(1, 2, 1, 2, 4);
    dijFuzzy.addLink(1, 5, 6, 13, 15);
    dijFuzzy.addLink(1, 6, 11, 14, 14);
    dijFuzzy.addLink(2, 3, 0, 2, 4);
    dijFuzzy.addLink(2, 4, 0, 2, 6);
    dijFuzzy.addLink(3, 4, 3, 4, 8);
    dijFuzzy.addLink(3, 5, 2, 3, 3);
    dijFuzzy.addLink(3, 7, 5, 7, 11);
    dijFuzzy.addLink(5, 6, 1, 5, 8);
    dijFuzzy.addLink(5, 7, 1, 3, 6);
    dijFuzzy.addLink(6, 7, 4, 6, 6);
    dijFuzzy.addLink(6, 8, 0, 1, 3);
    dijFuzzy.addLink(7, 9, 9, 10, 12);
    dijFuzzy.addLink(7, 12, 7, 12, 15);
    dijFuzzy.addLink(8, 9, 0, 1, 2);
    dijFuzzy.addLink(8, 10, 3, 5, 6);
    dijFuzzy.addLink(9, 10, 0, 2, 3);
    dijFuzzy.addLink(9, 11, 2, 3, 3);
    dijFuzzy.addLink(9, 12, 7, 8, 8);
    dijFuzzy.addLink(10, 11, 0, 2, 4);
    dijFuzzy.addLink(11, 12, 9, 10, 13);

    dijFuzzy.run();

    dijKFuzzy.addLink(1, 2, 1, 2, 4);
    dijKFuzzy.addLink(1, 5, 6, 13, 15);
    dijKFuzzy.addLink(1, 6, 11, 14, 14);
    dijKFuzzy.addLink(2, 3, 0, 2, 4);
    dijKFuzzy.addLink(2, 4, 0, 2, 6);
    dijKFuzzy.addLink(3, 4, 3, 4, 8);
    dijKFuzzy.addLink(3, 5, 2, 3, 3);
    dijKFuzzy.addLink(3, 7, 5, 7, 11);
    dijKFuzzy.addLink(5, 6, 1, 5, 8);
    dijKFuzzy.addLink(5, 7, 1, 3, 6);
    dijKFuzzy.addLink(6, 7, 4, 6, 6);
    dijKFuzzy.addLink(6, 8, 0, 1, 3);
    dijKFuzzy.addLink(7, 9, 9, 10, 12);
    dijKFuzzy.addLink(7, 12, 7, 12, 15);
    dijKFuzzy.addLink(8, 9, 0, 1, 2);
    dijKFuzzy.addLink(8, 10, 3, 5, 6);
    dijKFuzzy.addLink(9, 10, 0, 2, 3);
    dijKFuzzy.addLink(9, 11, 2, 3, 3);
    dijKFuzzy.addLink(9, 12, 7, 8, 8);
    dijKFuzzy.addLink(10, 11, 0, 2, 4);
    dijKFuzzy.addLink(11, 12, 9, 10, 13);

    dijKFuzzy.run();

    std::string nameK= "rutasK.txt";
    if (myAddress == 1)
        myfile.open (nameK);
    else
        myfile.open (nameK, std::ios_base::out | std::ios_base::app);
    for (int i = 1; i <=12; i ++) {
         if (i == myAddress) continue;
         int numR = dijKFuzzy.getNumRoutes(i);
         DijkstraKshortestFuzzy::FuzzyCost cost1;
         DijkstraKshortestFuzzy::FuzzyCost cost2;
         std::vector<NodeId> path1;
         std::vector<NodeId> path2;

         myfile << "Solution from " + std::to_string(myAddress);
         myfile << " to " + std::to_string(i) << " for Alpha "<< dijFuzzy.getAlpha() << "\n";
         for (int k = 0; k < numR; k++) {
             std::vector<NodeId> pathNode1;
             std::vector<NodeId> pathNode2;
             DijkstraKshortestFuzzy::FuzzyCost costAux1;
             DijkstraKshortestFuzzy::FuzzyCost costAux2;

             costAux1 = dijKFuzzy.getRouteCost(i, pathNode1, k);

             if (path1.empty()) {
                 if (costAux1 == DijkstraKshortestFuzzy::maximumCost)
                     throw cRuntimeError("");
                 path1 = pathNode1;
                 cost1 = costAux1;
             }
             for (int l = k+1; l < numR; l++) {
                 std::vector<NodeId> pathNode;
                 costAux2 = dijKFuzzy.getRouteCost(i, pathNode2, l);
                 if (pathNode2.empty())
                     continue;
                 if (dijKFuzzy.commonLinks(pathNode1, pathNode2) != 0)
                     continue;
                 if (path2.empty()) {
                     path2 = pathNode2;
                     cost2 = costAux2;
                     continue;
                 }
                 if (costAux1 + costAux2 < cost1 + cost2) {
                     path1 = pathNode1;
                     cost1 = costAux1;
                     path2 = pathNode2;
                     cost2 = costAux2;
                 }
             }
         }
         DijkstraKshortestFuzzy::FuzzyCost cost3 = cost1+cost2;
         myfile << "PairShortestPathSolution from " + std::to_string(myAddress);
         myfile << " to " + std::to_string(i) << " for Alpha "<< dijFuzzy.getAlpha() <<  " weight= (" << cost3.cost1 <<"," << cost3.cost2 << "," << cost3.cost3 << "); \n";
         myfile << "solution1=Solution " ;
         myfile << "weight= (" << cost1.cost1 <<"," << cost1.cost2 << "," << cost1.cost3 << "); nodes=[";
         for (auto elem : path1) {
             myfile <<" "<< std::to_string(elem);
             if (elem != path1.back())
                 myfile <<",";
         }
         myfile <<"] \n";
         myfile << "solution2=Solution " ;
         myfile << "weight= (" << cost2.cost1 <<"," << cost2.cost2 << "," << cost2.cost3 << "); nodes=[";
         for (auto elem : path2) {
             myfile <<" "<< std::to_string(elem);
             if (elem != path2.back())
                 myfile <<",";
         }
         myfile <<"] \n";
         PairPaths pair;
         pair.path1 = path1;
         pair.path2 = path2;
         pair.cost1 = DijkstraFuzzy::FuzzyCost(cost3.cost1, cost3.cost2, cost3.cost3);
         kRoutes[i] = pair;
    }
    myfile.close();


    std::string name= "rutas.txt";
    if (myAddress == 1)
        myfile.open (name);
    else
        myfile.open (name, std::ios_base::out | std::ios_base::app);
/*
    PairShortestPathSolution from 1 to 2 for Alpha 0.6 (totallyDisjoint: true; weight(xMin=0)  19,60; weight: ( 9, 20, 26))
      solution1=Solution [weight(xMin=0)=   2,40; weight= ( 1,  2,  4); nodes= [1, 2]]
      solution2=Solution [weight(xMin=0)=  17,20; weight= ( 8, 18, 22); nodes= [1, 5, 3, 2]]
*/
    for (int i = 1; i <=12; i ++) {

        if (i == myAddress) continue;
        dijFuzzy.runDisjoint(i);
        DijkstraFuzzy::Route r1;
        DijkstraFuzzy::Route r2;
        DijkstraFuzzy::FuzzyCost cost3;
        if (dijFuzzy.checkDisjoint(i, r1, r2)) {
            // print routes
            DijkstraFuzzy::FuzzyCost cost1;
            DijkstraFuzzy::FuzzyCost cost2;

            dijFuzzy.getCostPath(r1, cost1);
            dijFuzzy.getCostPath(r2, cost2);
            cost3 = cost1+cost2;

            myfile << "PairShortestPathSolution from " + std::to_string(myAddress);
            myfile << " to " + std::to_string(i) << " for Alpha "<< dijFuzzy.getAlpha() <<  " weight= (" << cost3.cost1 <<"," << cost3.cost2 << "," << cost3.cost3 << "); \n";
            myfile << "solution1=Solution " ;
            myfile << "weight= (" << cost1.cost1 <<"," << cost1.cost2 << "," << cost1.cost3 << "); nodes=[";
            for (auto elem : r1) {
                myfile <<" "<< std::to_string(elem);
                if (elem != r1.back())
                    myfile <<",";
            }
            myfile <<"] \n";
            myfile << "solution2=Solution " ;
            myfile << "weight= (" << cost2.cost1 <<"," << cost2.cost2 << "," << cost2.cost3 << "); nodes=[";
            for (auto elem : r2) {
                myfile <<" "<< std::to_string(elem);
                if (elem != r2.back())
                    myfile <<",";
            }
            myfile <<"] \n";
        }
        else {
            throw cRuntimeError("ERROR");
        }
        PairPaths pair;
        pair.path1 = r1;
        pair.path2 = r2;
        pair.cost1 = cost3;
        pRoutes[i] = pair;
    }
    myfile.close();

    for (int i = 0; i < pRoutes.size(); i++) {
        if (pRoutes[i].cost1 != kRoutes[i].cost1)
            throw cRuntimeError("");
    }
#endif
    std::vector<std::string> nedTypes;
    nedTypes.push_back(getParentModule()->getNedTypeName());

    cTopology topo("topo");
    topo.extractByNedTypeName(nedTypes);


    dijKFuzzy.setFromTopo(&topo);
    dijFuzzy.setFromTopo(&topo);

    dijKFuzzy.setRoot(myAddress);
    dijKFuzzy.run();

    dijFuzzy.setRoot(myAddress);
    dijFuzzy.setHasFindDisjoint(true);
    dijFuzzy.run();

    pRoutes.clear();
    kRoutes.clear();

    for (int i = 0; i < topo.getNumNodes(); i ++) {
        NodeId add = topo.getNode(i)->getModule()->par("address");
        if (add == myAddress) continue;
        dijFuzzy.runDisjoint(add);
        DijkstraFuzzy::Route r1;
        DijkstraFuzzy::Route r2;
        DijkstraFuzzy::FuzzyCost cost3A;
        if (dijFuzzy.checkDisjoint(add, r1, r2)) {
            // print routes
            DijkstraFuzzy::FuzzyCost cost1;
            DijkstraFuzzy::FuzzyCost cost2;

            dijFuzzy.getCostPath(r1, cost1);
            dijFuzzy.getCostPath(r2, cost2);
            cost3A = cost1+cost2;
        }
        else {
            throw cRuntimeError("ERROR");
        }
        PairPaths pair;
        pair.path1 = r1;
        pair.path2 = r2;
        pair.cost1 = cost3A;
        pRoutes[add] = pair;

        int numR = dijKFuzzy.getNumRoutes(add);
        DijkstraKshortestFuzzy::FuzzyCost cost1(1e100, 1e100, 1e100);
        DijkstraKshortestFuzzy::FuzzyCost cost2(1e100, 1e100, 1e100);
        DijkstraKshortestFuzzy::FuzzyCost cost3;
        std::vector<NodeId> pathInitial;
        std::vector<NodeId> path1;
        std::vector<NodeId> path2;
        for (int k = 0; k < numR; k++) {
            std::vector<NodeId> pathNode1;
            std::vector<NodeId> pathNode2;
            DijkstraKshortestFuzzy::FuzzyCost costAux1;
            DijkstraKshortestFuzzy::FuzzyCost costAux2;
            costAux1 = dijKFuzzy.getRouteCost(add, pathNode1, k);
            if (pathInitial.empty())
                pathInitial = pathNode1;

            for (int l = k+1; l < numR; l++) {
                std::vector<NodeId> pathNode;
                costAux2 = dijKFuzzy.getRouteCost(add, pathNode2, l);
                if (pathNode2.empty())
                    continue;
                if (dijKFuzzy.commonLinks(pathNode1, pathNode2) != 0)
                    continue;
                if (costAux1 + costAux2 < cost1 + cost2) {
                    path1 = pathNode1;
                    cost1 = costAux1;
                    path2 = pathNode2;
                    cost2 = costAux2;
                }
            }
        }
        cost3 = cost1+cost2;
        pair.path1 = path1;
        pair.path2 = path2;
        if (pair.path1.empty())
            pair.path1 = pathInitial;
        pair.cost1 = DijkstraFuzzy::FuzzyCost(cost3.cost1, cost3.cost2, cost3.cost3);
        kRoutes[add] = pair;
        if (dijKFuzzy.commonLinks(pair.path1, pair.path2) != 0) {
            throw cRuntimeError("");
        }

    }

    std::string nameD= "diff.txt";
    if (myAddress == 0)
        myfile.open (nameD);
    else
        myfile.open (nameD, std::ios_base::out | std::ios_base::app);
    // check diferences
//    for (auto elem : pRoutes) {
//        auto it = kRoutes.find(elem.first);
//        if (it == kRoutes.end()) {
//            throw cRuntimeError("ERROR");
//        }
//        if (!(elem.second.path1 == it->second.path1 && elem.second.path1 == it->second.path1)){
//            // diferences
//            // throw cRuntimeError("Diferences ");
//            // print routes
//            DijkstraFuzzy::FuzzyCost cost1;
//            DijkstraFuzzy::FuzzyCost cost2;
//            DijkstraFuzzy::FuzzyCost cost3;
//            cost3 = cost1+cost2;
//            for (auto elem2 : elem.second.path1) {
//                myfile <<" "<< std::to_string(elem2);
//                if (elem2 != elem.second.path1.back())
//                    myfile <<",";
//            }
//            for (auto elem2: elem.second.path2) {
//                myfile <<" "<< std::to_string(elem2);
//                if (elem2 != elem.second.path2.back())
//                    myfile <<",";
//            }
//        }
//    }
//

    for (auto elem : pRoutes) {
        if (elem.first == myAddress)
            continue; // no routes to root node
        auto it = kRoutes.find(elem.first);
        PairPaths pair1, pair2;
        pair1 = elem.second;
        pair2 = it->second;

        if (pair1.path2.empty())
            throw cRuntimeError("");
        if (pair2.path2.empty())
            continue;

        if (pair1.cost1 != pair2.cost1) {
            myfile <<" Destination "<< std::to_string(elem.first);
            myfile <<"\n";
            myfile << "weight= (" << pair1.cost1.cost1 <<"," << pair1.cost1.cost2 << "," << pair1.cost1.cost3 << " \n";
            myfile << "Path 1 = (" ;
            for (auto elem2 : pair1.path1) {
                myfile <<" "<< std::to_string(elem2);
                if (elem2 != elem.second.path1.back())
                    myfile <<",";
            }
            myfile <<") \n";
            myfile << "Path 2 = (" ;
            for (auto elem2 : pair1.path2) {
                myfile <<" "<< std::to_string(elem2);
                if (elem2 != elem.second.path1.back())
                    myfile <<",";
            }
            myfile <<") \n";

            myfile << "weight2= (" << pair2.cost1.cost1 <<"," << pair2.cost1.cost2 << "," << pair2.cost1.cost3 << " \n";
            myfile << "Path 1 = (" ;
            for (auto elem2 : pair2.path1) {
                myfile <<" "<< std::to_string(elem2);
                if (elem2 != elem.second.path1.back())
                    myfile <<",";
            }
            myfile <<") \n";
            myfile << "Path 2 = (" ;
            for (auto elem2 : pair2.path2) {
                myfile <<" "<< std::to_string(elem2);
                if (elem2 != elem.second.path1.back())
                    myfile <<",";
            }
            myfile <<") \n";
            // throw cRuntimeError("");
        }
    }
    myfile.close();
#endif
}

CallApp::CallApp()
{
    callIdentifier = 1;
}

CallApp::~CallApp()
{
    if (generateCall)
        cancelAndDelete(generateCall);
    if (nextEvent)
        cancelAndDelete(nextEvent);
    CallPacketsEvents.clear();
    while (!activeCalls.empty()) {
        delete activeCalls.begin()->second;
        activeCalls.erase(activeCalls.begin());
    }
    CallEvents.clear();
}


void CallApp::rescheduleEvent()
{
    if (CallEvents.empty() && FlowEvents.empty() && CallPacketsEvents.empty() && FlowPacketsEvents.empty()) {
        if (nextEvent->isScheduled())
            cancelEvent(nextEvent);
        return;
    }

    simtime_t min1 = SimTime::getMaxTime();
    simtime_t min2 = SimTime::getMaxTime();
    simtime_t min3 = SimTime::getMaxTime();
    simtime_t min4 = SimTime::getMaxTime();

    if (!CallEvents.empty())
        min1 = CallEvents.begin()->first;
    if (!FlowEvents.empty())
        min2 = FlowEvents.begin()->first;
    if (!CallPacketsEvents.empty())
        min3 = CallPacketsEvents.begin()->first;
    if (!FlowPacketsEvents.empty())
        min4 = FlowPacketsEvents.begin()->first;

    simtime_t min = min1<min2?min1:min2;
    if (min > min3)
        min = min3;
    if (min>min4)
        min = min4;

    if (min<simTime())
        throw cRuntimeError("Planing error");

    if (!nextEvent->isScheduled()) {
        scheduleAt(min, nextEvent);
    }
    else if (min != nextEvent->getArrivalTime()) {
        cancelEvent(nextEvent);
        scheduleAt(min, nextEvent);
    }
}

void CallApp::bytesTraceRec(const CallInfo *callInfo)
{

    if (!trace) return;

    double brec = callInfo->recBandwith * SIMTIME_DBL(simTime() - callInfo->startOnRec);
    double tm = SIMTIME_DBL(simTime() - callInfo->startOnRec);

    char fileName[100];
    memset(fileName,0,sizeof(fileName));
    sprintf(fileName,"Rec-%i",myAddress);
    std::ofstream myfile;
    myfile.open(fileName, std::ios::out | std::ios::app);

    myfile << simTime().dbl() << " " << callInfo->startOnRec << " ";
    myfile << "Dest " << callInfo->dest  << " CalId :" << callInfo->callId <<"   bw: " << callInfo->usedBandwith << " t " << tm << " brec " << brec << "\n";
}

void CallApp::bytesTraceRec(const CallInfo *callInfo, const FlowData &flow)
{

    if (!trace) return;

    double brec = flow.recBandwith * SIMTIME_DBL(simTime() - flow.startOnRec);
    double tm = SIMTIME_DBL(simTime() - flow.startOnRec);

    char fileName[100];
    memset(fileName,0,sizeof(fileName));
    sprintf(fileName,"Rec-%i",myAddress);
    std::ofstream myfile;
    myfile.open(fileName, std::ios::out | std::ios::app);

    myfile << simTime().dbl() << " " << flow.startOnRec << " ";
    myfile << "Dest " << callInfo->dest  << " CalId :" << callInfo->callId <<"   bw: " << callInfo->usedBandwith << " t " << tm << " brec " << brec << "\n";
}

void CallApp::bytesTraceSend(const CallInfo *callInfo, const FlowData &flow)
{

    if (!trace) return;

    double bsend = flow.usedBandwith * SIMTIME_DBL(simTime() - flow.startOn);

    char fileName[100];
    memset(fileName,0,sizeof(fileName));
    sprintf(fileName,"Send-%i",myAddress);
    std::ofstream myfile;
    myfile.open(fileName, std::ios::out | std::ios::app);

    double tm = SIMTIME_DBL(simTime() - callInfo->startOn);

    myfile << simTime().dbl() << " " << callInfo->startOn << " ";
    myfile << "Dest " << callInfo->dest  << " CalId :" << flow.callId <<"   bw: " << flow.usedBandwith << " t " << tm << " bsend " << bsend << "\n";
}

void CallApp::bytesTraceSend(const CallInfo *callInfo)
{

    if (!trace) return;

    double bsend = callInfo->usedBandwith * SIMTIME_DBL(simTime() - callInfo->startOn);

    char fileName[100];
    memset(fileName,0,sizeof(fileName));
    sprintf(fileName,"Send-%i",myAddress);
    std::ofstream myfile;
    myfile.open(fileName, std::ios::out | std::ios::app);

    double tm = SIMTIME_DBL(simTime() - callInfo->startOn);

    myfile << simTime().dbl() << " " << callInfo->startOn << " ";
    myfile << "Dest " << callInfo->dest  << " CalId :" << callInfo->callId <<"   bw: " << callInfo->usedBandwith << " t " << tm << " bsend " << bsend << "\n";
}
// Event flow generate methods,
// Call associated methods.



void CallApp::newCallFlow(CallInfo *callInfo, const uint64_t  &bw)
{
    char pkname[100];
    callInfo->state = ON;
    callInfo->flowId++;
    callInfo->reservedBandwith = bw;
    callInfo->usedBandwith = (uint64_t) usedBandwith->doubleValue();
    callInfo->paketSize = packetSize->intValue();
    simtime_t delayAux = TimeOn->doubleValue();
    callInfo->startOn = simTime();
    CallEvents.insert(std::make_pair(simTime() + delayAux, callInfo));

    if (callInfo->dest == myAddress)
        throw cRuntimeError("Destination address erroneous");

    if (simulationMode == PACKETMODE) {
        // TODO: Compute interarrival time of the packet
        callInfo->interArrivalTime = (double) callInfo->paketSize/ (double) callInfo->usedBandwith;
        newCallPacket(callInfo);
        return;
    }
    if (callInfo->disjoint) {
        uint64_t  half = (callInfo->usedBandwith) >> 1;
        uint64_t  half2 = callInfo->usedBandwith - half;

        Packet *pkFlow = new Packet();
        pkFlow->setSourceId(par("sourceId").intValue());
        pkFlow->setDestinationId(callInfo->sourceId);
        pkFlow->setSrcAddr(myAddress);
        pkFlow->setDestAddr(callInfo->dest);
        pkFlow->setType(STARTFLOW);
        pkFlow->setReserve(half);
        pkFlow->setFlowId(callInfo->flowId);
        pkFlow->setCallId(callInfo->flowData[0].callId);
        pkFlow->setDisjoint(true);
        callInfo->flowData[0].state = ON;
        callInfo->flowData[1].state = ON;
        callInfo->flowData[0].startOn = simTime();
        callInfo->flowData[1].startOn = simTime();

        Packet *pkFlow2 = pkFlow->dup();
        pkFlow2->setCallId(callInfo->flowData[1].callId);
        pkFlow2->setReserve(half2);

        sprintf(pkname, "FlowOn-%d-to-%d-#%" PRIu64 "-Sid-%d-FlowId-%" PRIu64 "", myAddress,
                pkFlow->getDestAddr(), pkFlow->getCallId(),
                this->getIndex(), callInfo->flowId);
        pkFlow->setName(pkname);
        sprintf(pkname, "FlowOn-%d-to-%d-#%" PRIu64 "-Sid-%d-FlowId-%" PRIu64 "", myAddress,
                pkFlow->getDestAddr(), pkFlow->getCallId(),
                this->getIndex(), callInfo->flowId);
        pkFlow2->setName(pkname);

        send(pkFlow, "out");
        send(pkFlow2, "out");
    }
    else {
        Packet *pkFlow = new Packet();
        pkFlow->setSourceId(par("sourceId").intValue());
        pkFlow->setDestinationId(callInfo->sourceId);
        pkFlow->setSrcAddr(myAddress);
        pkFlow->setDestAddr(callInfo->dest);
        pkFlow->setType(STARTFLOW);
        pkFlow->setReserve(callInfo->usedBandwith);
        pkFlow->setFlowId(callInfo->flowId);
        pkFlow->setCallId(callInfo->callId);
        sprintf(pkname, "FlowOn-%d-to-%d-#%" PRIu64 "-Sid-%d-FlowId-%" PRIu64 "", myAddress,
                pkFlow->getDestAddr(), pkFlow->getCallId(),
                this->getIndex(), callInfo->flowId);
        pkFlow->setName(pkname);

        send(pkFlow, "out");
    }
}


double CallApp::startCallFlow(CallInfo *callInfo, Packet *pkFlow, Packet *pkFlow2)
{
    char pkname[100];
    callInfo->state = ON;
    callInfo->flowId++;
    pkFlow->setFlowId(callInfo->flowId);
    callInfo->usedBandwith = (uint64_t) usedBandwith->doubleValue();
    callInfo->paketSize = packetSize->intValue();
    pkFlow->setType(STARTFLOW);
    callInfo->startOn = simTime();


    sprintf(pkname, "FlowOn-%d-to-%d-CallId#%" PRIu64 "-FlowId#%" PRIu64 "-Sid-%d", myAddress, pkFlow->getDestAddr(),
            pkFlow->getCallId(), pkFlow->getFlowId(), this->getIndex());
    pkFlow->setName(pkname);

    if (pkFlow2 != nullptr) {
        pkFlow2->setType(STARTFLOW);
        pkFlow2->setFlowId(callInfo->flowId);
        sprintf(pkname, "FlowOn-%d-to-%d-CallId#%" PRIu64 "-FlowId#%" PRIu64 "-Sid-%d", myAddress, pkFlow2->getDestAddr(),
                pkFlow2->getCallId(), pkFlow2->getFlowId(), this->getIndex());
        pkFlow2->setName(pkname);

        uint64_t  half = (callInfo->usedBandwith) >> 1;
        uint64_t  half2 = callInfo->usedBandwith - half;
        if (half > half2) {
            pkFlow->setReserve(half);
            pkFlow2->setReserve(half2);
        }
        else {
            pkFlow->setReserve(half2);
            pkFlow2->setReserve(half);
        }
        if (!callInfo->disjoint)
            throw cRuntimeError("");
        callInfo->flowData[0].flowId = callInfo->flowId;
        callInfo->flowData[1].flowId = callInfo->flowId;
        callInfo->flowData[0].usedBandwith = pkFlow->getReserve();
        callInfo->flowData[1].usedBandwith = pkFlow2->getReserve();
        callInfo->flowData[0].state = ON;
        callInfo->flowData[1].state = ON;
        callInfo->flowData[0].startOn = simTime();
        callInfo->flowData[1].startOn = simTime();
    }
    else
        pkFlow->setReserve(callInfo->usedBandwith);

    return TimeOn->doubleValue();

}


double CallApp::endCallFlow(CallInfo *callInfo, Packet *pkFlow, Packet *pkFlow2)
{

    // delete pending packets
    // TODO: Multiflow support for packets.
    for (auto it = CallPacketsEvents.begin(); it != CallPacketsEvents.end();) {
        if (it->second == callInfo)
            CallPacketsEvents.erase(it++);
        else
            ++it;
    }

    char pkname[100];
    bytesTraceSend(callInfo);
    if (!callInfo->disjoint) {
        double bsend = callInfo->usedBandwith * SIMTIME_DBL(simTime() - callInfo->startOn);
        callInfo->acumulateSend += (bsend/1000.0);
        EV << "Call Id :" << callInfo->callId << "Flow Id :" << callInfo->flowId <<
                " Time in on :" << callInfo->startOn << "Bsend :" << bsend;
    }
    else {
        double bsend = callInfo->flowData[0].usedBandwith * SIMTIME_DBL(simTime() - callInfo->flowData[0].startOn);
        double bsend2 = callInfo->flowData[1].usedBandwith * SIMTIME_DBL(simTime() - callInfo->flowData[1].startOn);
        callInfo->flowData[0].state = OFF;
        callInfo->flowData[1].state = OFF;
        callInfo->acumulateSend += (bsend/1000.0);
        callInfo->acumulateSend += (bsend2/1000.0);
        EV << "Call Id :" << callInfo->callId << "Flow Id :" << callInfo->flowId <<
                   " Time in on :" << callInfo->startOn << "Bsend :" << bsend;
    }
    callInfo->state = OFF;
    pkFlow->setType(ENDFLOW);
    pkFlow->setFlowId(callInfo->flowId);

    sprintf(pkname, "FlowOff-%d-to-%d-CallId#%" PRIu64 "-FlowId#%" PRIu64 "-Sid-%d",
                                    myAddress, pkFlow->getDestAddr(),
                                    pkFlow->getCallId(),pkFlow->getFlowId(), this->getIndex());

    pkFlow->setName(pkname);
    if (pkFlow2 != nullptr) {
        pkFlow2->setType(ENDFLOW);
        sprintf(pkname, "FlowOff-%d-to-%d-CallId#%" PRIu64 "-FlowId#%" PRIu64 "-Sid-%d",
                                        myAddress, pkFlow2->getDestAddr(),
                                        pkFlow2->getCallId(),pkFlow2->getFlowId(), this->getIndex());
        pkFlow2->setName(pkname);
        pkFlow2->setFlowId(callInfo->flowId);
        uint64_t  half = (callInfo->usedBandwith) >> 1;
        uint64_t  half2 = callInfo->usedBandwith - half;
        if (half > half2) {
            pkFlow->setReserve(half);
            pkFlow2->setReserve(half2);
        }
        else {
            pkFlow->setReserve(half2);
            pkFlow2->setReserve(half);
        }
    }
    else
        pkFlow->setReserve(callInfo->usedBandwith);
    return TimeOff->doubleValue();
}

double CallApp::startCallFlow(CallInfo *callInfo, Packet *pkFlow, Packet *pkFlow2, FlowData & elem)
{
    char pkname[100];
    elem.state = ON;
    callInfo->flowId++;
    elem.flowId = callInfo->flowId;
    pkFlow->setFlowId(callInfo->flowId);
    elem.usedBandwith = (uint64_t) usedBandwith->doubleValue();
    elem.paketSize = packetSize->intValue();
    pkFlow->setReserve(elem.usedBandwith);
    pkFlow->setType(STARTFLOW);
    callInfo->startOn = simTime();
    sprintf(pkname, "FlowOn-%d-to-%d-CallId#%" PRIu64 " - FlowId#%" PRIu64 " -Sid-%d",
            myAddress, pkFlow->getDestAddr(),
            pkFlow->getCallId(),pkFlow->getFlowId(), this->getIndex());
    pkFlow->setName(pkname);

    if (pkFlow2 != nullptr) {
        pkFlow2->setType(STARTFLOW);
        pkFlow2->setFlowId(callInfo->callIdBk);
        sprintf(pkname, "FlowOn-%d-to-%d-CallId#%" PRIu64 " - FlowId#%" PRIu64 " -Sid-%d",
                myAddress, pkFlow2->getDestAddr(),
                pkFlow2->getCallId(), pkFlow2->getFlowId(), this->getIndex());
        pkFlow2->setName(pkname);

        if (callInfo->disjoint) {
            uint64_t  half = (elem.usedBandwith) >> 1;
            uint64_t  half2 = elem.usedBandwith - half;
            if (half > half2) {
                pkFlow->setReserve(half);
                pkFlow2->setReserve(half2);
            }
            else {
                pkFlow->setReserve(half2);
                pkFlow2->setReserve(half);
            }
        }
        else {
            throw cRuntimeError("pkFlow2 != nullptr and callInfo->disjoint");
        }
    }
    else
        pkFlow->setReserve(elem.usedBandwith);

    return TimeOn->doubleValue();

}

double CallApp::endCallFlow(CallInfo *callInfo, Packet *pkFlow, Packet *pkFlow2, FlowData & elem)
{
    char pkname[100];

    bytesTraceRec(callInfo);
    callInfo->acumulateSend += ((elem.usedBandwith * SIMTIME_DBL(simTime() - elem.startOn))/1000.0);
    pkFlow->setFlowId(elem.flowId);
    elem.state = OFF;
    pkFlow->setType(ENDFLOW);
    pkFlow->setReserve(elem.usedBandwith);
    sprintf(pkname, "FlowOff-%d-to-%d-CallId#%" PRIu64 "- FlowId#%" PRIu64 " -Sid-%d",
            myAddress, pkFlow->getDestAddr(),
            pkFlow->getCallId(),pkFlow->getFlowId(), this->getIndex());
    pkFlow->setName(pkname);
    if (pkFlow2 != nullptr) {
        pkFlow2->setType(ENDFLOW);
        pkFlow2->setFlowId(callInfo->callIdBk);
        sprintf(pkname, "pkFlow-%d-to-%d-CallId#%" PRIu64 " - FlowId#%" PRIu64 " -Sid-%d",
                myAddress, pkFlow2->getDestAddr(),
                pkFlow2->getCallId(), pkFlow2->getFlowId(), this->getIndex());
        pkFlow2->setName(pkname);

        uint64_t  half = (elem.usedBandwith) >> 1;
        uint64_t  half2 = elem.usedBandwith - half;
        if (half > half2) {
            pkFlow->setReserve(half);
            pkFlow2->setReserve(half2);
        }
        else {
            pkFlow->setReserve(half2);
            pkFlow2->setReserve(half);
        }
    }
    else
        pkFlow->setReserve(elem.usedBandwith);

    return TimeOff->doubleValue();
}

// Flow associated methods.

void CallApp::newFlow() {

    if (destAddresses.empty())
        return;
    char pkname[100];

// generate the next flow
    int destAddress = destAddresses[intuniform(0, destAddresses.size() - 1)];
    Packet *pkFlow = new Packet();

    pkFlow->setReserve(flowUsedBandwith->doubleValue());
    pkFlow->setDestAddr(destAddress);
    pkFlow->setCallId(0);
    pkFlow->setSourceId(par("sourceId").intValue());
    pkFlow->setDestinationId(par("destinationId").intValue());
    pkFlow->setSrcAddr(myAddress);
    pkFlow->setFlowId(flowIdentifier++);
    pkFlow->setType(STARTFLOW);

    sprintf(pkname, "NewFlow-%d-to-%d-Call Id #%" PRIu64 "- Flow Id #%" PRIu64 " -Did-%ld",
            myAddress, destAddress, pkFlow->getCallId(), pkFlow->getFlowId(),
            par("sourceId").intValue());
    pkFlow->setName(pkname);
    if (pkFlow->getDestAddr() == myAddress)
        throw cRuntimeError("Destination address erroneous");
    send(pkFlow, "out");

    if (hasGUI())
        getParentModule()->bubble("Generating flow..");

    FlowEvent *event = new FlowEvent;
    event->dest = destAddress;
    event->destId = pkFlow->getDestinationId();
    event->flowId = pkFlow->getFlowId();
    event->usedBandwith = pkFlow->getReserve();
    event->paketSize = packetSize->intValue();
    event->startOn = simTime();
    FlowEvents.insert(std::make_pair(simTime() + flowDuration->doubleValue(), event));
}


void  CallApp::endFlow(FlowEvent *flowEvent) {
    char pkname[100];

    Packet *pkFlow = new Packet();
    pkFlow->setReserve(flowEvent->usedBandwith);
    pkFlow->setDestAddr(flowEvent->dest);
    pkFlow->setCallId(0);
    pkFlow->setSourceId(par("sourceId").intValue());
    pkFlow->setDestinationId(flowEvent->destId);
    pkFlow->setSrcAddr(myAddress);
//callInfo->acumulateSend += (callInfo->usedBandwith
//        * callInfo->startOn.dbl());

    pkFlow->setType(ENDFLOW);
    pkFlow->setFlowId(flowEvent->flowId);
    sprintf(pkname, "FlowOff-%d-to-%d-CallId#%" PRIu64 "- FlowId#%" PRIu64 " -Sid-%d",
            myAddress, pkFlow->getDestAddr(),
            pkFlow->getCallId(),pkFlow->getFlowId(), this->getIndex());
    pkFlow->setName(pkname);
    if (pkFlow->getDestAddr() == myAddress)
        throw cRuntimeError("Destination address erroneous");
    send(pkFlow, "out");
    if (simulationMode == FLOWMODE) {
        double bsend = flowEvent->usedBandwith * SIMTIME_DBL(simTime() - flowEvent->startOn);
        auto itStat = sendBytes.find(flowEvent->dest);
        if (itStat == sendBytes.end())
            sendBytes[flowEvent->dest] = (bsend/1000.0);
        else
            itStat->second += (bsend/1000.0);
    }
}

// packet generaration
void CallApp::newCallPacket(CallInfo *callInfo)
{
    char pkname[100];

    if (callInfo->interArrivalTime == SimTime::ZERO)
        return;

    // TODO: Divide the flow if the source is disjoint

    Packet *pkFlow = new Packet();
    pkFlow->setSourceId(par("sourceId").intValue());
    pkFlow->setDestinationId(callInfo->sourceId);
    pkFlow->setSrcAddr(myAddress);
    pkFlow->setDestAddr(callInfo->dest);
    pkFlow->setType(DATATYPE);
    pkFlow->setReserve(callInfo->usedBandwith);
    pkFlow->setFlowId(callInfo->flowId);
    pkFlow->setCallId(callInfo->callId);
    pkFlow->setBitLength(callInfo->paketSize);

    sprintf(pkname, "Data-%d-to-%d-#%" PRIu64 "-Sid-%d-FlowId-%" PRIu64, myAddress, pkFlow->getDestAddr(), pkFlow->getCallId(),
            this->getIndex(), callInfo->flowId);
    pkFlow->setName(pkname);
    // check if it is the latest packet of the flow.
    for (auto elem : CallEvents) {
        if (elem.second == callInfo) {
            if (elem.second->state != ON)
                throw cRuntimeError("Call state Error");


            if (simTime() + callInfo->interArrivalTime < elem.first)
                CallPacketsEvents.insert(std::make_pair(simTime() + callInfo->interArrivalTime, callInfo));
            else if (simTime() > elem.first)
                throw cRuntimeError("CallEvents Scheduler error compute time off");
            else
                pkFlow->setLast(true);                 // last
            break;
        }
    }

    if (pkFlow->getDestAddr() == myAddress)
        throw cRuntimeError("Destination address erroneous");

    auto itStat = sendBytes.find(pkFlow->getDestAddr());
    if (itStat == sendBytes.end())
        sendBytes[pkFlow->getDestAddr()] = pkFlow->getBitLength();
    else
        itStat->second += pkFlow->getBitLength();
    totalPkSent++;
    send(pkFlow, "out");
}


void CallApp::procNextEvent()
{
    char pkname[100];
    memset(pkname,0,sizeof(pkname));
    double delayAux;
    while (!CallEvents.empty() && CallEvents.begin()->first <= simTime()) {
        auto it = CallEvents.begin();
        CallInfo *callInfo = it->second;
        CallEvents.erase(it);

        Packet *pkFlow = new Packet();
        Packet *pkFlow2 = nullptr;

        pkFlow->setDestAddr(callInfo->dest);
        pkFlow->setCallId(callInfo->callId);
        pkFlow->setSourceId(par("sourceId").intValue());
        pkFlow->setDestinationId(callInfo->sourceId);
        pkFlow->setSrcAddr(myAddress);


        if (callInfo->disjoint)
        {
            pkFlow->setCallId(callInfo->flowData[0].callId);
            pkFlow2 = pkFlow->dup();
            pkFlow2->setCallId(callInfo->flowData[1].callId);
        }


        if (callInfo->flowData.empty() || callInfo->disjoint) {
            if (callInfo->state == ON) {
                callInfo->interArrivalTime = SimTime::ZERO;
                delayAux = endCallFlow(callInfo, pkFlow, pkFlow2);
            }
            else if (callInfo->state == OFF) {
                double timePackets = (double) callInfo->paketSize/ (double) callInfo->usedBandwith;
                callInfo->interArrivalTime = SimTime(timePackets);
                delayAux = startCallFlow(callInfo, pkFlow, pkFlow2);
            }
            if (pkFlow->getDestAddr() == myAddress)
                throw cRuntimeError("Destination address erroneous");
            if (simulationMode == FLOWMODE) {
                send(pkFlow, "out");
                if (pkFlow2)
                    send(pkFlow2, "out");
            }
            else {
                delete pkFlow;
                if (pkFlow2) {
                    delete pkFlow2;
                    pkFlow2 = nullptr;
                }
            }
            CallEvents.insert(std::make_pair(simTime() + delayAux, callInfo));
            // If simulation is Packet mode and state is in on, generate next
            if (simulationMode == PACKETMODE && callInfo->state == ON) {
                newCallPacket(callInfo);
            }
        }
        else {
            // multi flow system
            throw cRuntimeError("Disabled");
            for (auto &elem : callInfo->flowData) {
                if (elem.nextEvent > simTime())
                    continue;
                if (elem.state == ON) {
                    delayAux = endCallFlow(callInfo, pkFlow, pkFlow2, elem);
                }
                else if (elem.state == OFF) {
                    delayAux = startCallFlow(callInfo, pkFlow, pkFlow2, elem);
                    if (simulationMode == PACKETMODE) {
                        // TODO: Compute interarrival time of the packet
                        callInfo->interArrivalTime = (double) callInfo->paketSize/ (double) callInfo->usedBandwith;
                        newCallPacket(callInfo);
                    }
                }
                elem.nextEvent = simTime() + delayAux;
                if (pkFlow->getDestAddr() == myAddress)
                    throw cRuntimeError("Destination address erroneous");
                if (simulationMode == FLOWMODE)
                    send(pkFlow, "out");
                else
                    delete pkFlow;
                CallEvents.insert(std::make_pair(simTime() + delayAux, callInfo));
            }
        }
    }

    while (!FlowEvents.empty() && FlowEvents.begin()->first <= simTime()) {
        auto it = FlowEvents.begin();
        FlowEvent *flowEvent = it->second;
        FlowEvents.erase(it);
        endFlow(flowEvent);
        delete flowEvent;
    }


    while (!CallPacketsEvents.empty() && CallPacketsEvents.begin()->first <= simTime()) {
        auto it = CallPacketsEvents.begin();

        CallInfo *callEvent = it->second;
        CallPacketsEvents.erase(it);
        newCallPacket(callEvent);
    }

    while (!FlowPacketsEvents.empty() && FlowPacketsEvents.begin()->first <= simTime()) {
        auto it = FlowPacketsEvents.begin();
        //FlowEvent *flowEvent = it->second;
        FlowPacketsEvents.erase(it);
        throw cRuntimeError("implementation pending");
        // procFlowPacketEvent(flowEvent);
    }
}



void CallApp::newCall() {
    if (destAddresses.empty())
        return;

    char pkname[100];
// Sending packet
    callCounter++;

    int destAddress = destAddresses[intuniform(0, destAddresses.size() - 1)];

    sprintf(pkname, "CallReserve-%d-to-%d-Call id#%" PRIu64 "-Did-%ld", myAddress,
            destAddress, callIdentifier, par("sourceId").intValue());
    EV << "generating packet " << pkname << endl;

    Packet *pk = new Packet(pkname);
    pk->setType(RESERVE);
    pk->setReserve(callReserve->doubleValue());
    pk->setSrcAddr(myAddress);
    pk->setDestAddr(destAddress);
    pk->setCallId(callIdentifier);
    callIdentifier++;
    if (callIdentifier == 0) // 0 is reserved for flows not assigned to a call.
        callIdentifier++;
    pk->setSourceId(par("sourceId").intValue());
    pk->setDestinationId(par("destinationId").intValue());

    DijkstraFuzzy::Route r1, r2, min;

    if (par("disjointRoute").boolValue() || routingModule->getRoutingType() == IRouting::BACKUPROUTE || routingModule->getRoutingType() == IRouting::BACKUPROUTEKSH || routingModule->getRoutingType() == IRouting::DISJOINT ) {
        if (par("disjointRoute").boolValue())
            routingModule->getPairRoutes(destAddress,r1,r2, true);
        else
            routingModule->getPairRoutes(destAddress,r1,r2);
        // TODO: backup mode Se deben enviar dos paquetes, uno por cada ruta
        // new call id for backup route
        if (par("disjointRoute").boolValue() && r1.empty() && r2.empty())
            throw cRuntimeError("only a route");
        if (par("disjointRoute").boolValue() || routingModule->getRoutingType() == IRouting::DISJOINT)
            pk->setDisjoint(true);
        Packet *pk2 = pk->dup();
        pk2->setCallId(callIdentifier);
        callIdentifier++;
        pk->setDisjoint(true);
        pk2->setDisjoint(true);
        pk2->setCallIdBk(pk->getCallId());
        pk->setCallIdBk(pk2->getCallId());
        pk2->setRouteArraySize(r1.size());
        for (unsigned int i = 0; i < r1.size(); i++) {
            pk2->setRoute(i, r1[i]);
        }
        pk->setRouteArraySize(r2.size());
        for (unsigned int i = 0; i < r2.size(); i++) {
            pk->setRoute(i, r2[i]);
        }
        pk2->setType(RESERVEBK);
        // send before the best
        send(pk, "out");
        send(pk2, "out");
    }
    else {
        routingModule->getPairRoutes(destAddress,r1,r2);
        pk->setRouteArraySize(r1.size());
        for (unsigned int i = 0; i < r1.size(); i++) {
            pk->setRoute(i, r1[i]);
        }
        send(pk, "out");
    }
// TODO : recall timer,

}

void CallApp::newReserve(Packet *pk)
{

    CallInfo *callInfo = nullptr;
    auto itAux = activeCalls.find(pk->getCallId());

    if (itAux != activeCalls.end())
        throw cRuntimeError("Call id presents in the system");

    auto itAux2 = backupCalls.find(pk->getCallId());
    if (itAux2 != backupCalls.end())
        throw cRuntimeError("Call id presents in the system");

    bool activeCall = true;
    if (itAux == activeCalls.end() &&  pk->getCallIdBk() == 0) {
        callInfo = new CallInfo();
        callInfo->dest = pk->getSrcAddr();
        callInfo->sourceId = pk->getSourceId();
        callInfo->callId = pk->getCallId();
        callInfo->callIdBk = pk->getCallIdBk();
        if (callInfo->dest == myAddress)
            throw cRuntimeError("Address destination Error");
        activeCalls.insert(std::make_pair(callInfo->callId, callInfo));
        callReceived++;
    }
    else if (itAux == activeCalls.end() &&  pk->getCallIdBk() != 0) {
        // check if the other call exist
        auto itAux2 = activeCalls.find(pk->getCallIdBk());
        if (itAux2 != activeCalls.end()) { // store this like backup
            // sanity check
            auto itAuxSanity = backupCalls.find(pk->getCallIdBk());
            if (itAuxSanity != backupCalls.end())
                throw cRuntimeError("Call id presents in the system, bk list");
            CallInfo * callInfo = itAux2->second;
            callInfo->callIdBk = pk->getCallId();
            if (callInfo->dest == myAddress)
                throw cRuntimeError("Address destination Error");
            if (pk->getDisjoint())
                callInfo->disjoint = true; // both routes active.
            backupCalls.insert(std::make_pair(pk->getCallId(), callInfo));
            activeCall = false;
            if (pk->getDisjoint()) {
                // prepare disjoint
                callInfo->disjoint = true;
                callInfo->flowData.resize(2);
                callInfo->flowData[0].callId = pk->getCallIdBk();
                callInfo->flowData[1].callId = pk->getCallId();
            }
        }
        else {
            // there isn't any available, create a new information
            callInfo = new CallInfo();
            callInfo->dest = pk->getSrcAddr();
            callInfo->sourceId = pk->getSourceId();
            callInfo->callId = pk->getCallId();
            callInfo->callIdBk = 0; //
            if (callInfo->dest == myAddress)
                throw cRuntimeError("Address destination Error");
            activeCalls.insert(std::make_pair(callInfo->callId, callInfo));
            callReceived++;
        }
    }

    // check in the list of calls

    if (activeCall) {
        if (generateFlow) {
            callInfo->state = OFF;
            callInfo->reservedBandwith = pk->getReserve();
            callInfo->usedBandwith = (uint64_t) usedBandwith->doubleValue();
            CallEvents.insert(std::make_pair(simTime() + TimeOff->doubleValue(), callInfo));
        }
        else
            callInfo->state = PASSIVE;
    }
    // return acceptance packet
    char pkname[100];
    pk->setType(ACEPTED);
    pk->setDestAddr(pk->getSrcAddr());
    pk->setSrcAddr(myAddress);
    sprintf(pkname, "PkAccepted-%d-to-%d-#%" PRIu64 "-Sid-%d", myAddress, pk->getDestAddr(), pk->getCallId(), this->getIndex());
    pk->setName(pkname);
    pk->setDestinationId(pk->getSourceId());
    pk->setSourceId(par("sourceId").intValue());
    if (pk->getDestAddr() == myAddress)
        throw cRuntimeError("Destination address erroneous");
    send(pk, "out");
}

void CallApp::newAccepted(Packet *pk) {

    // prepare the release

    CallInfo *callInfo = nullptr;

    auto itAux = activeCalls.find(pk->getCallId());

    if (itAux != activeCalls.end())
        throw cRuntimeError("Call id presents in the system");

    auto itAux2 = backupCalls.find(pk->getCallId());
    if (itAux2 != backupCalls.end())
        throw cRuntimeError("Call id presents in the system");


    bool activeCall = true;
    // check if exist the backup
    if (pk->getCallIdBk() != 0) {
        if (pk->getDisjoint()) {
            activeCall = false; // must both paths acitive
        }
        auto itAux3 = activeCalls.find(pk->getCallIdBk());
        if (itAux3 != activeCalls.end()) {
            callInfo = itAux3->second;
            callInfo->callIdBk = pk->getCallId();
            if (callInfo->dest == myAddress)
                throw cRuntimeError("Address destination Error");
            backupCalls.insert(std::make_pair(callInfo->callIdBk, callInfo));
            activeCall = false;
            if (pk->getDisjoint()) {
                activeCall = true;
                callInfo->disjoint = true;
                callInfo->flowData.resize(2);
                callInfo->flowData[0].callId = pk->getCallIdBk();
                callInfo->flowData[1].callId = pk->getCallId();
            }
        }
        else {// check alternative list
            auto itAux3 = backupCalls.find(pk->getCallIdBk());
            if (itAux3 != backupCalls.end()) {
                callInfo = itAux3->second;
                if (pk->getDisjoint()) {
                    activeCall = true;
                    callInfo->disjoint = true;
                    callInfo->flowData.resize(2);
                    callInfo->flowData[0].callId = pk->getCallIdBk();
                    callInfo->flowData[1].callId = pk->getCallId();
                }
            }
            else {
                callInfo = new CallInfo;
                callInfo->dest = pk->getSrcAddr();
                callInfo->callId = pk->getCallId();
                callInfo->sourceId = pk->getSourceId();
            }
            if (callInfo->dest == myAddress)
                throw cRuntimeError("Address destination Error");
            activeCalls.insert(std::make_pair(callInfo->callId, callInfo));
        }
    }
    else {
        callInfo = new CallInfo;
        callInfo->dest = pk->getSrcAddr();
        callInfo->callId = pk->getCallId();
        callInfo->sourceId = pk->getSourceId();
        if (callInfo->dest == myAddress)
            throw cRuntimeError("Address destination Error");
        activeCalls.insert(std::make_pair(callInfo->callId, callInfo));
    }

    if (activeCall) {
        if (generateFlow) {
            newCallFlow(callInfo, pk->getReserve());
        }
        else
            callInfo->state = PASSIVE;
        // prepare release event
        callEstabilized++;
        char pkname[100];
        pk->setType(RELEASE);
        pk->setDestAddr(pk->getSrcAddr());
        pk->setSrcAddr(myAddress);
        pk->setDestinationId(pk->getSourceId());
        pk->setSourceId(par("sourceId").intValue());

        sprintf(pkname, "Pkrelease-%d-to-%d-#%" PRIu64 "-Sid-%d", myAddress,
                pk->getDestAddr(), pk->getCallId(), this->getIndex());
        pk->setName(pkname);
        scheduleAt(simTime() + callDuration->doubleValue(), pk);
        listPendingRelease.push_back(pk);
    }
    else
    {
        delete pk;
    }
}

void CallApp::storeCallStatistics(const CallInfo *callInfo) {

    if (callInfo->state == ON) {
        bytesTraceSend (callInfo);
        double bsend = callInfo->usedBandwith
                * SIMTIME_DBL(simTime() - callInfo->startOn);
        const_cast<CallInfo *>(callInfo)->acumulateSend += ((bsend) / 1000.0);
    }

    if (callInfo->stateRec == ON) {
        bytesTraceRec (callInfo);
        double brec = callInfo->recBandwith
                * SIMTIME_DBL(simTime() - callInfo->startOnRec);
        const_cast<CallInfo *>(callInfo)->acumulateRec += (brec / 1000.0);
    }

// record the statistics
    if (simulationMode == FLOWMODE) {
        auto itAccSend = sendBytes.find(callInfo->dest);
        auto itAccRec = receivedBytes.find(callInfo->dest);
        if (itAccSend == sendBytes.end())
            sendBytes[callInfo->dest] = callInfo->acumulateSend;
        else
            itAccSend->second += callInfo->acumulateSend;
        if (itAccRec == receivedBytes.end())
            receivedBytes[callInfo->dest] = callInfo->acumulateRec;
        else
            itAccRec->second += callInfo->acumulateRec;
    }
}

void CallApp::storeCallStatistics(const CallInfo *callInfo, const FlowData & flow) {

    if (flow.state == ON) {
        bytesTraceSend (callInfo);
        double bsend = flow.usedBandwith
                * SIMTIME_DBL(simTime() - flow.startOn);
        const_cast<CallInfo *>(callInfo)->acumulateSend += ((bsend) / 1000.0);
    }

    if (flow.stateRec == ON) {
        bytesTraceRec (callInfo, flow);
        double brec = flow.recBandwith
                * SIMTIME_DBL(simTime() - flow.startOnRec);
        const_cast<CallInfo *>(callInfo)->acumulateRec += (brec / 1000.0);
    }

// record the statistics
    if (simulationMode == FLOWMODE) {
        auto itAccSend = sendBytes.find(callInfo->dest);
        auto itAccRec = receivedBytes.find(callInfo->dest);
        if (itAccSend == sendBytes.end())
            sendBytes[callInfo->dest] = callInfo->acumulateSend;
        else
            itAccSend->second += callInfo->acumulateSend;
        if (itAccRec == receivedBytes.end())
            receivedBytes[callInfo->dest] = callInfo->acumulateRec;
        else
            itAccRec->second += callInfo->acumulateRec;
    }
}

void CallApp::release(Packet *pk) {

    // Handle incoming packet

    auto it = activeCalls.find(pk->getCallId());
    auto it2 = backupCalls.find(pk->getCallId());


    if (pk->isSelfMessage()) { // local release
        // search in the list
        auto itPendingRelease = std::find(listPendingRelease.begin(),listPendingRelease.end(),pk);
        if (itPendingRelease == listPendingRelease.end()) {
            throw cRuntimeError("Packet not found in the pending release");
        }
        else
            listPendingRelease.erase(itPendingRelease);
    }

    if (it == activeCalls.end() && it2 == backupCalls.end()) {
        if (!pk->isSelfMessage()) // si se ha liberador por rotura debería haber llegado a la otra parte el relese con lo cual no debe mandar el mensaje de release otra vez
            throw cRuntimeError("Call Id not found in any list");
        else {
            if (pk->getCallIdBk() == 0) {
                delete pk;
                return;
            }
            auto it = activeCalls.find(pk->getCallIdBk());
            if (it == activeCalls.end()) {
                callRejected++;
                delete pk;
                return;
            }
            else {
             // El backup pasó a principal, se debe eliminar ahora el backup
                throw cRuntimeError("Call Id not found in any list");
                pk->setCallId(pk->getCallIdBk());
                pk->setCallIdBk(0);
            }
        }
    }

    CallInfo *callInfo = nullptr;
    if (it != activeCalls.end())
        callInfo = it->second;
    if (callInfo == nullptr && it2 != backupCalls.end())
        callInfo = it2->second;
    if (callInfo == nullptr)
        throw cRuntimeError("Call Id not found in any list");

    if (callInfo != nullptr) {
        if (pk->isSelfMessage() && callInfo->callIdBk != 0)
        { // local release with backup route, release backup route.
            // release backup route

            Packet * pkt = pk->dup();
            if (pk->getCallId() == callInfo->callIdBk)
                pkt->setCallId(callInfo->callId);
            else if (pk->getCallId() == callInfo->callId)
                pkt->setCallId(callInfo->callIdBk);
            else
                throw cRuntimeError("");


            if (pkt->getDestAddr() == myAddress)
                throw cRuntimeError("Destination address erroneous");
            send(pkt, "out");

            if (callInfo->disjoint) {
                for (auto &elem : callInfo->flowData) {
                    if (elem.stateRec == ON) {
                        bytesTraceRec(callInfo);
                        callInfo->acumulateRec += ((elem.recBandwith
                                * SIMTIME_DBL(simTime() - elem.startOnRec))/1000);
                        // send off in the o
                        elem.stateRec = OFF;
                    }
                    if (elem.state == OFF) {
                        for (auto &elem : callInfo->flowData) {
                            bytesTraceSend(callInfo, elem);
                            callInfo->acumulateSend += ((elem.usedBandwith * SIMTIME_DBL(simTime() - elem.startOn)) / 1000);
                            // send off in the o
                        }
                    }
                }
                callInfo->disjoint = false;
                callInfo->flowData.clear();
                callInfo->state = OFF;
            }

            for (auto it = CallEvents.begin(); it != CallEvents.end();) {
                if (it->second->callIdBk == pk->getCallId())
                    CallEvents.erase(it++);
                else
                    ++it;
            }

            auto itAux = backupCalls.find(callInfo->callIdBk);
            if (itAux != backupCalls.end()) {
                backupCalls.erase(itAux);
                callInfo->callIdBk = 0;
            }
            else {
                // the release has arrive with the backup
                auto itAux = backupCalls.find(callInfo->callId);
                if (itAux != backupCalls.end()) {
                    backupCalls.erase(itAux);
                    callInfo->callId = callInfo->callIdBk;
                    callInfo->callIdBk = 0;
                    auto it = activeCalls.find(callInfo->callId);
                    if (it == activeCalls.end()) {
                        throw cRuntimeError("Call id not found");
                    }
                }
            }
        }

        for (auto it = CallEvents.begin(); it != CallEvents.end();) {
            if (it->second->callId == pk->getCallId())
                CallEvents.erase(it++);
            else
                ++it;
        }

        if (callInfo->callIdBk != 0) // change to backup route
        {
            // if arrive here, is a remote release,
            for (auto & elem : listPendingRelease) {
                if (elem->getCallId() == callInfo->callId) {
                    elem->setCallId(callInfo->callIdBk);
                    break;
                }
            }

            auto itAux =  backupCalls.find(callInfo->callIdBk);

            if (itAux != backupCalls.end()) {
                backupCalls.erase(itAux);
            }
            else
                throw cRuntimeError("Check bk table");


            // if active flows send end
            if (callInfo->disjoint) { // remote release
                for (auto &elem : callInfo->flowData) {
                    if (elem.state == ON) {
                        bytesTraceSend(callInfo, elem);
                        callInfo->acumulateSend += ((elem.usedBandwith * SIMTIME_DBL(simTime() - elem.startOn)) / 1000);
                        // send off in the o
                        elem.state = OFF;
                        if (simulationMode != PACKETMODE && elem.callId != pk->getCallId()) {
                            Packet *pkFlow = new Packet();
                            pkFlow->setDestAddr(callInfo->dest);
                            pkFlow->setCallId(elem.callId);
                            pkFlow->setSourceId(par("sourceId").intValue());
                            pkFlow->setDestinationId(callInfo->sourceId);
                            pkFlow->setType(ENDFLOW);
                            pkFlow->setFlowId(elem.flowId);
                            pkFlow->setReserve(elem.usedBandwith);
                            char pkname[60];
                            sprintf(pkname,
                                    "FlowOff-%d-to-%d-CallId#%" PRIu64 "-FlowId#%" PRIu64 "Sid-%d",
                                    myAddress, pkFlow->getDestAddr(),
                                    pkFlow->getCallId(), pkFlow->getFlowId(),
                                    this->getIndex());
                            pkFlow->setName(pkname);
                            send(pkFlow, "out");
                        }
                    }
                }
                callInfo->disjoint = false;
                callInfo->flowData.clear();
                if (callInfo->state == ON) {
                    CallEvents.insert(std::make_pair(simTime() + TimeOff->doubleValue(), callInfo));
                    callInfo->state = OFF;
                }
            }

            if (callInfo->state == ON) {
                bytesTraceSend(callInfo);
                callInfo->acumulateSend += ((callInfo->usedBandwith
                        * SIMTIME_DBL(simTime() - callInfo->startOn))/1000);
                // send off in the o
                callInfo->state = OFF;
                if (simulationMode != PACKETMODE) {
                    Packet *pkFlow = new Packet();
                    pkFlow->setDestAddr(callInfo->dest);
                    pkFlow->setCallId(callInfo->callId);
                    pkFlow->setSourceId(par("sourceId").intValue());
                    pkFlow->setDestinationId(callInfo->sourceId);
                    pkFlow->setType(ENDFLOW);
                    pkFlow->setFlowId(callInfo->flowId);
                    pkFlow->setReserve(callInfo->usedBandwith);
                    char pkname[60];
                    sprintf(pkname,"FlowOff-%d-to-%d-CallId#%" PRIu64 "-FlowId#%" PRIu64 "Sid-%d", myAddress, pkFlow->getDestAddr(),  pkFlow->getCallId(), pkFlow->getFlowId(), this->getIndex());
                    pkFlow->setName(pkname);
                    send(pkFlow, "out");
                }
                CallEvents.insert(std::make_pair(simTime() + TimeOff->doubleValue(), callInfo));
            }
            // if active flows send end
            if (callInfo->stateRec == ON) {
                bytesTraceRec(callInfo);
                callInfo->acumulateRec += ((callInfo->recBandwith
                        * SIMTIME_DBL(simTime() - callInfo->startOnRec))/1000);
                // send off in the o
                callInfo->stateRec = OFF;
            }
            // change all to backup

            if (callInfo->callIdBk == pk->getCallId()) {
                backupCalls.erase(callInfo->callIdBk);
                activeCalls.erase(callInfo->callIdBk);
                callInfo->callIdBk = 0;
            }
            else if (callInfo->callId == pk->getCallId()) {
                backupCalls.erase(callInfo->callId);
                activeCalls.erase(callInfo->callId);
                uint64_t callidBk = callInfo->callIdBk;
                callInfo->callIdBk = 0;
                callInfo->callId = callidBk;
                if (callInfo->dest == myAddress)
                    throw cRuntimeError("Address destination Error");
                activeCalls.insert(std::make_pair(callInfo->callId,callInfo));
            }

        }
        else {
            // Accumulate the flows in curse
            storeCallStatistics(callInfo);
            for (auto itAux = CallPacketsEvents.begin(); itAux != CallPacketsEvents.end();) {
                if (itAux->second == callInfo)
                    CallPacketsEvents.erase(itAux++);
                else
                    ++itAux;
            }
            auto itAux = activeCalls.find(callInfo->callId);
            if (itAux == activeCalls.end())
                throw cRuntimeError("Call id not found");
            delete callInfo;
            activeCalls.erase(itAux);
        }
    }
    else if (it2 != backupCalls.end())
    {
        // erase backup
        // Acummulate
        for (auto &elem : it2->second->flowData) {
            if (elem.callId != it2->first) continue;
            if (elem.stateRec == ON) {
                bytesTraceRec(callInfo);
                callInfo->acumulateRec += ((elem.recBandwith
                        * SIMTIME_DBL(simTime() - elem.startOnRec))/1000);
                // send off in the o
                elem.stateRec = OFF;
            }
            if (elem.state == OFF) {
                for (auto &elem : callInfo->flowData) {
                    bytesTraceSend(callInfo, elem);
                    callInfo->acumulateSend += ((elem.usedBandwith * SIMTIME_DBL(simTime() - elem.startOn)) / 1000);
                    // send off in the o
                    }
            }
        }
        if (it2->second->flowData[0].state)

        for (auto &elem : it2->second->flowData) {
            if (elem.stateRec == ON) {
                bytesTraceRec(callInfo);
                callInfo->acumulateRec += ((elem.recBandwith
                        * SIMTIME_DBL(simTime() - elem.startOnRec))/1000);
                // send off in the o
                elem.stateRec = OFF;
            }
            if (elem.state == OFF) {
                for (auto &elem : callInfo->flowData) {
                    bytesTraceSend(callInfo, elem);
                    callInfo->acumulateSend += ((elem.usedBandwith * SIMTIME_DBL(simTime() - elem.startOn)) / 1000);
                    // send off in the o
                    }
            }
        }

        it2->second->callIdBk = 0;
        it2->second->disjoint = false;
        backupCalls.erase(it2);
        // Nothing more to do
    }

// if self message send the release message to the other node
    if (pk->isSelfMessage()) {
        if (pk->getDestAddr() == myAddress)
            throw cRuntimeError("Destination address erroneous");
        send(pk, "out");
    }
    else {
        // relese receive nothing more to do
        delete pk;
        if (hasGUI()) {
            getParentModule()->getDisplayString().setTagArg("i", 1, "green");
            getParentModule()->bubble("Arrived!");
        }
    }
}

void CallApp::procFlowPk(Packet *pk) {
    FlowIdentification flowId;
    flowId.callId() = pk->getCallId();
    flowId.flowId() = pk->getFlowId();
    flowId.src() = pk->getSrcAddr();
    flowId.srcId() = pk->getSourceId();


    // auto it = flowStatistics.find(flowId);
//  if (it == flowStatistics.end())
//      throw cRuntimeError("Call id is not registered");

    if (pk->getType() == ENDFLOW) {
        emit(rcvdPk,pk);
        if (flowId.callId() > 0) {
            CallInfo * callInfo = nullptr;
            auto itAux = activeCalls.find(flowId.callId());
            auto itAuxBk = backupCalls.find(flowId.callId());
            if (itAux != activeCalls.end()) // check if in backup
                callInfo = itAux->second;
            else if (itAuxBk != backupCalls.end())
                callInfo = itAuxBk->second;
            else
                throw cRuntimeError("Call id not found but flow received");
            if (callInfo->disjoint) { // use multiflow
                bool found = false;

                for (auto &elem : callInfo->flowData) {
                    if (elem.callId == flowId.callId()) {
                        if (elem.flowIdRec == flowId.flowId()) {
                            found = true;
                            elem.stateRec = OFF;
                            double brec = elem.recBandwith
                                    * SIMTIME_DBL(simTime() - elem.startOnRec);
                            callInfo->acumulateRec += (brec/1000.0);

                            EV << "Rec Call Id :" << callInfo->callId << "Flow Id :"
                                    << elem.flowId << " Time in on :"
                                    << (simTime() - elem.startOnRec) << "Brec :" << brec;
                        }
                        else if (elem.stateRec == OFF) { // The reserve never arrive
                            found = true;
                        }
                        break;
                    }
                }
                if (!found) { // Flow lost?
                    // check status if one of both are OFF do nothing
                    throw cRuntimeError("Flow is not registered");
                }
                if (callInfo->flowData[0].stateRec == OFF && callInfo->flowData[1].stateRec == OFF)
                    callInfo->stateRec = OFF;
            }
            else {
                bytesTraceRec(callInfo);
                double brec = callInfo->recBandwith
                        * SIMTIME_DBL(simTime() - callInfo->startOnRec);
                callInfo->acumulateRec += (brec/1000.0);
                callInfo->stateRec = OFF;
                EV << "Rec Call Id :" << callInfo->callId << "Flow Id :"
                        << callInfo->flowId << " Time in on :"
                        << (simTime() - callInfo->startOnRec) << "Brec :" << brec;
            }
        }
        else {
            auto it = flowStatistics.find(flowId);
            if (it == flowStatistics.end())
                throw cRuntimeError("Flow is not registered");
            double brec = it->second.used * SIMTIME_DBL(simTime() - it->second.startOnRec);

            auto itStat = receivedBytes.find(flowId.src());
            if (itStat == receivedBytes.end())
                receivedBytes[flowId.src()] = (brec/1000.0);
            else
                itStat->second = (brec/1000.0);
            // erase flow
            flowStatistics.erase(it);
        }
    }
    else if (pk->getType() == STARTFLOW) {
        if (flowId.callId() > 0) {
            CallInfo * callInfo = nullptr;
            auto itAux = activeCalls.find(flowId.callId());
            auto itAuxBk = backupCalls.find(flowId.callId());
            if (itAux == activeCalls.end() && itAuxBk == backupCalls.end())
                throw cRuntimeError("Call Id not found in any list %i",
                        flowId.callId());
            if (itAux != activeCalls.end())
                callInfo = itAux->second;
            else if (itAuxBk != backupCalls.end())
                callInfo = itAuxBk->second;

            if (callInfo->disjoint) {
                auto itAux = activeCalls.find(callInfo->callId);
                auto itAuxBk = backupCalls.find(callInfo->callIdBk);
                if (itAux == activeCalls.end() || itAuxBk == backupCalls.end())
                    throw cRuntimeError("Disjoint call but only in a list.");
            }

            if (callInfo->disjoint) { // use multiflow
                for (auto &elem : callInfo->flowData) {
                    if (elem.callId == flowId.callId() && elem.flowIdRec == flowId.flowId())
                        throw cRuntimeError("Flow if already present in the flow list of the call");
                    if (elem.callId == flowId.callId() && elem.stateRec != OFF)
                        throw cRuntimeError("Flow is in not OFF state");
                    if (elem.callId == flowId.callId()) {
                        elem.flowIdRec = flowId.flowId();
                        elem.recBandwith = (uint64_t) pk->getReserve();
                        elem.stateRec = ON;
                        elem.startOnRec = simTime();
                        // search if present
                        break;
                    }
                }
            }
            else {
                if (itAux == activeCalls.end()) {
                    // check backup
                    // change to backup route
                    // if there is a active flow stop it and change to the other
                    /*if (callInfo->state == ON) {

                     for (auto it = CallEvents.begin(); it != CallEvents.end();) {
                     if (it->second->callId == callInfo->callId)
                     CallEvents.erase(it++);
                     else
                     ++it;
                     }
                     bytesTraceSend(callInfo);
                     callInfo->acumulateSend += ((callInfo->recBandwith
                     * SIMTIME_DBL(simTime() - callInfo->startOn))/1000);
                     // send off in the o
                     callInfo->state = OFF;
                     Packet *pkFlow = new Packet();
                     pkFlow->setDestAddr(callInfo->dest);
                     pkFlow->setCallId(callInfo->callId);
                     pkFlow->setSourceId(par("sourceId").intValue());
                     pkFlow->setDestinationId(callInfo->sourceId);
                     pkFlow->setType(ENDFLOW);
                     pkFlow->setFlowId(callInfo->flowId);
                     pkFlow->setReserve(callInfo->usedBandwith);
                     char pkname[60];
                     sprintf(pkname,"FlowOff-%d-to-%d-CallId#%" PRIu64 "-FlowId#%" PRIu64 "-Sid-%d", myAddress, pkFlow->getDestAddr(),  pkFlow->getCallId(), pkFlow->getFlowId(), this->getIndex());
                     pkFlow->setName(pkname);
                     if (pkFlow->getDestAddr() == myAddress)
                     throw cRuntimeError("Destination address erroneous");
                     send(pkFlow, "out");
                     CallEvents.insert(std::make_pair(simTime()+TimeOff->doubleValue(), callInfo));
                     }*/

                    uint64_t callId = callInfo->callId;
                    uint64_t calIdbk = callInfo->callIdBk;
                    callInfo->callId = calIdbk;
                    callInfo->callIdBk = callId;
                    backupCalls.erase(itAux);
                    itAux = activeCalls.find(callId);

                    if (itAux != activeCalls.end())
                        activeCalls.erase(itAux);

                    if (callInfo->dest == myAddress)
                        throw cRuntimeError("Address destination Error");

                    if (callId != 0)
                        backupCalls.insert(std::make_pair(callId, callInfo));
                    activeCalls.insert(std::make_pair(calIdbk, callInfo));

                    //
                    if (callInfo->state == ON) {
                        for (auto it = CallEvents.begin();
                                it != CallEvents.end();) {
                            if (it->second->callId == callId)
                                CallEvents.erase(it++);
                            else
                                ++it;
                        }
                        bytesTraceSend(callInfo);
                        callInfo->acumulateSend += ((callInfo->usedBandwith
                                * SIMTIME_DBL(simTime() - callInfo->startOn))
                                / 1000);
                        // send off in the o
                        callInfo->state = OFF;
                        Packet *pkFlow = new Packet();
                        pkFlow->setDestAddr(callInfo->dest);
                        pkFlow->setCallId(callId);
                        pkFlow->setSourceId(par("sourceId").intValue());
                        pkFlow->setDestinationId(callInfo->sourceId);
                        pkFlow->setType(ENDFLOW);
                        pkFlow->setFlowId(callInfo->flowId);
                        pkFlow->setReserve(callInfo->usedBandwith);
                        char pkname[60];

                        sprintf(pkname,
                                "FlowOff-%d-to-%d-CallId#%" PRIu64 "-FlowId#%" PRIu64 "-Sid-%d",
                                myAddress, pkFlow->getDestAddr(),
                                pkFlow->getCallId(), pkFlow->getFlowId(),
                                this->getIndex());
                        pkFlow->setName(pkname);
                        if (pkFlow->getDestAddr() == myAddress)
                            throw cRuntimeError(
                                    "Destination address erroneous");
                        send(pkFlow, "out");
                        CallEvents.insert(
                                std::make_pair(
                                        simTime() + TimeOff->doubleValue(),
                                        callInfo));
                    }
                }
                callInfo->recBandwith = (uint64_t) pk->getReserve();
                callInfo->startOnRec = simTime();
                callInfo->stateRec = ON;
                callInfo->flowIdRec = pk->getFlowId();
            }
        }
        else {
            // register the flow;
            auto it = flowStatistics.find(flowId);
            if (it != flowStatistics.end())
                throw cRuntimeError("FLow is already registered");

            FlowStat st;
            st.startOnRec = simTime();
            st.used = (uint64_t) pk->getReserve();
            flowStatistics.insert(std::make_pair(flowId,st));
        }
    }
    else if (pk->getType() == FLOWCHANGE) {
        if (flowId.callId() > 0) {
            auto itAux = activeCalls.find(flowId.callId());
            CallInfo * callInfo = itAux->second;
            if (callInfo->stateRec == ON) {
                bytesTraceRec(callInfo);
                callInfo->acumulateRec += ((callInfo->recBandwith
                        * SIMTIME_DBL(simTime() - callInfo->startOnRec))/1000);
            }
            callInfo->stateRec = ON;
            callInfo->recBandwith = (uint64_t) pk->getReserve();
            callInfo->startOnRec = simTime();
        }
        else {
            // register the flow;
            auto it = flowStatistics.find(flowId);
            if (it == flowStatistics.end()) {
                // create
                FlowStat st;
                st.startOnRec = simTime();
                st.used = (uint64_t) pk->getReserve();
                flowStatistics.insert(std::make_pair(flowId,st));
            }
            else {
                double brec = it->second.used * SIMTIME_DBL(simTime() - it->second.startOnRec);
                auto itStat = receivedBytes.find(flowId.src());
                if (itStat == receivedBytes.end())
                    receivedBytes[flowId.src()] = (brec/1000.0);
                else
                    itStat->second = (brec/1000.0);
                it->second.startOnRec = simTime();
                it->second.used = (uint64_t) pk->getReserve();
            }
        }

    }
    delete pk;
}

void CallApp::getNodesAddress(const char *destAddressesPar, std::vector<int> &listAddres)
{
    listAddres.clear();
    const char *token;
    if (strcmp(destAddressesPar,"any")==0) {
        cTopology topo("topo");
        std::vector<std::string> nedTypes;
        nedTypes.push_back(getParentModule()->getNedTypeName());
        topo.extractByNedTypeName(nedTypes);
        for (int i = 0; i < topo.getNumNodes(); i++) {
            cTopology::Node *node = topo.getNode(i);
            int destAddr = node->getModule()->par("address");
            if (destAddr != myAddress)
                listAddres.push_back(destAddr);
       }
    }
    else {
        cStringTokenizer tokenizer(destAddressesPar);
        while ((token = tokenizer.nextToken()) != NULL)
        {
            int destAddr = atoi(token);
            if (destAddr != myAddress)
                destAddresses.push_back(atoi(token));
        }
    }
}

void CallApp::initialize()
{
    myAddress = par("address");
    callReserve = &par("callReserve");
    callArrival = &par("callArrival");  // volatile parameter
    callDuration = &par("callDuration");  // volatile parameter

    generateFlow = par("generateFlow");
    TimeOn = &par("TimeOn");
    TimeOff = &par("TimeOff");
    usedBandwith = &par("usedBandwith");
    flowArrival = &par("flowArrival");
    flowDuration = &par("flowDuration");
    flowUsedBandwith = &par("flowUsedBandwith");

    packetSize = &par("packetSize");
    flowPacketSize = &par("flowPacketSize");

    std::vector<double> a;
    std::vector<uint32_t> b;

    callCounter = 0;
    if (par("packetMode"))
        simulationMode = PACKETMODE;

    WATCH(callCounter);
    WATCH(callReceived);
    WATCH(myAddress);
    WATCH_MAP(receivedBytes);
    WATCH_MAP(sendBytes);


    getNodesAddress(par("destAddresses"),destAddresses);

    generateCall = new cMessage("nextCall");
#ifndef ONLYONECALL
    if (!destAddresses.empty())
        scheduleAt(callArrival->doubleValue(), generateCall);
#else
    if (!destAddresses.empty())
        scheduleAt(simTime(), generateCall);
#endif


    routingModule = check_and_cast<IRouting *>(this->getParentModule()->getSubmodule("routing"));


    if (par("independentFlows").boolValue() && !destAddresses.empty()) {
        nextFlow = new cMessage();
        scheduleAt(flowArrival->doubleValue(), nextFlow);
    }

    nextEvent = new cMessage("NewEvent");
    RegisterMsg * msg = new RegisterMsg();
    msg->setSourceId(par("sourceId").intValue());
    send(msg, "out");

    initTime = time(nullptr);

    checkDijktra();

}

void CallApp::handleMessage(cMessage *msg)
{

    if (!msg->isPacket()) {
        if (msg == generateCall) {
            newCall();
#ifndef ONLYONECALL
            if (!destAddresses.empty())
                scheduleAt(simTime() + callArrival->doubleValue(), generateCall);
            if (hasGUI())
                getParentModule()->bubble("Generating call..");
#endif
        }
        else if (msg == nextFlow) {
            newFlow();
            if (!destAddresses.empty())
                scheduleAt(simTime() + flowArrival->doubleValue(), nextFlow);
        }
        else if (msg == nextEvent) {
            procNextEvent();
        }
        else
            delete msg;

        rescheduleEvent();
        return;

    }

    Base *pkaux = dynamic_cast<Base *>(msg);
    if (pkaux == nullptr) {
        delete msg;
        return;
    }

    if (pkaux->getType() == ACTUALIZE) {
        delete msg;
        return;
    }

    Packet *pk = dynamic_cast<Packet *>(msg);
    if (pk == nullptr)
        throw cRuntimeError("Packet unknown");

    if (pk->getType() == RESERVE || pk->getType() == RESERVEBK) {
       newReserve(pk);
    }
    else if (pk->getType() == ACEPTED) {
        newAccepted(pk);
    }
    else if (pk->getType() == RELEASE || pk->getType() == RELEASEDELAYED || pk->getType() == RELEASEBREAK) {
        release(pk);
    }
    else if (pk->getType() == BREAK) {

    }
    else if (pk->getType() == REJECTED) {
        callRejected++;
        auto it = activeCalls.find(pk->getCallId());
        if (it != activeCalls.end())
            throw cRuntimeError("This call must not be registered");
        delete pk;
    }
    else if (pk->getType() == DATATYPE) {
        totalPkRec++;
        auto itAccRec = receivedBytes.find(pk->getSrcAddr());
        if (itAccRec == receivedBytes.end())
            receivedBytes[pk->getSrcAddr()] = pk->getBitLength();
        else
            itAccRec->second += pk->getBitLength();

        emit(rcvdPk,pk);
        delete pk;
    }
    else // flow control packets, can be used to compute the statistics
    {
        procFlowPk(pk);
    }
    rescheduleEvent();
}

void CallApp::finish()
{
    // total data of flows and calls that has finished
    long double totalSend = 0;
    long double totalRec = 0;
    if (simulationMode == FLOWMODE) {
        for (auto elem : receivedBytes)
            totalRec += elem.second;
        for (auto elem : sendBytes)
            totalSend += elem.second;


        // register call in curse
        for (auto elem : activeCalls) {
            if (elem.second->disjoint) {
                // Disjoint calls, the data are in flowData vector
                for (auto &elem2 : elem.second->flowData) {
                    if (elem2.state == ON) {
                        elem.second->acumulateSend += ((elem2.usedBandwith * SIMTIME_DBL(simTime() - elem2.startOn))/1000);
                        elem2.state = OFF;
                    }
                    if (elem2.stateRec == ON) {
                        bytesTraceRec(elem.second, elem2);
                        elem.second->acumulateRec += ((elem2.recBandwith * SIMTIME_DBL(simTime() - elem2.startOnRec))/1000);
                        elem2.stateRec = OFF;
                    }
                }
                // The data has been recorded, set to OFF
                elem.second->state = OFF;
                elem.second->stateRec = OFF;
            }
            if (elem.second->state == ON) {
                bytesTraceSend(elem.second);
                elem.second->acumulateSend += ((elem.second->usedBandwith * SIMTIME_DBL(simTime() - elem.second->startOn))/1000);
            }
            if (elem.second->stateRec == ON) {
                bytesTraceRec(elem.second);
                elem.second->acumulateRec += ((elem.second->recBandwith * SIMTIME_DBL(simTime() - elem.second->startOnRec))/1000);
            }
            totalRec += elem.second->acumulateRec;
            totalSend += elem.second->acumulateSend;
        }

    }
    else {
        for (auto elem : receivedBytes)
            totalRec += (elem.second/1000);
        for (auto elem : sendBytes)
            totalSend += (elem.second/1000);
    }


    activeCalls.clear();

    // pending flow "end" in transmission
    while (!FlowEvents.empty()) {
        // flows that the node is sending
        auto it = FlowEvents.begin();
        FlowEvent *flowEvent = it->second;
        FlowEvents.erase(it);
        double bsend = flowEvent->usedBandwith * SIMTIME_DBL(simTime() - flowEvent->startOn);
        totalSend += (bsend/1000);
        delete flowEvent;
    }
    // pending flow "end" in receptions
    while (!flowStatistics.empty()) {
        auto it = flowStatistics.begin();
        double brec = it->second.used * SIMTIME_DBL(simTime() - it->second.startOnRec);
        totalRec += (brec/1000);
        flowStatistics.erase(it);
    }

    recordScalar("Total Send Kb",totalSend);
    recordScalar("Total Rec Kb",totalRec);

    recordScalar("Total Pk Sent",totalPkSent);
    recordScalar("Total Pk Rec",totalPkRec);

    recordScalar("Total calls generated",callCounter);
    recordScalar("Total calls established",callEstabilized);
    recordScalar("Total calls received",callReceived);

    recordScalar("Total calls callRejected",callRejected);

    recordScalar("simulation time",time(nullptr)-initTime);

}
