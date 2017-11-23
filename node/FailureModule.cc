//
// Copyright (C)Author: alfonso ariza quintana, Universidad de Malaga
// Copyright (C) 2017 Alfonso Ariza
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 


#include <stdlib.h>     /* atoi */
#include "FailureModule.h"


Define_Module(FailureModule);


simsignal_t FailureModule::eventSignal = registerSignal("EventSignal");

std::ostream& operator<<(std::ostream& os, const Event& e)
{
    os << e.info();
    return os;
};


std::string FailureModule::str() const
{
    std::stringstream out;

    if (eventList.empty())
        out << "Event list empty";
    for (auto elem : eventList) {
        out << "Event time : " << elem.first;
        out << "  Event - " << elem.second.info();
        out << endl;
    }
    return out.str();
}

simtime_t FailureModule::rdUniform(cXMLAttributeMap &attributes)
{
    if (!_hasKey(attributes, "beginning"))
        throw "Undefined parameter for random distribution. Beginning must be defined for a normal distribution";
    if (!_hasKey(attributes, "end"))
        throw "Undefined parameter for random distribution. End must be defined for a normal distribution";
    double m_beginning = atof(attributes["beginning"].c_str());
    double m_end = atof(attributes["end"].c_str());
    return omnetpp::uniform(getEnvir()->getRNG(0), m_beginning, m_end);
}

simtime_t FailureModule::rdExponential(cXMLAttributeMap &attributes)
{

    if (!_hasKey(attributes, "mean"))
        throw cRuntimeError(
                "Undefined parameter for random distribution. Mean must be defined for an exponential distribution");
    double m_mean = atof(attributes["mean"].c_str());

    double m_min = 0.0;
    double m_max = SimTime::getMaxTime().dbl();
    double m_bMinLimit = _hasKey(attributes, "min");
    double m_bMaxLimit = _hasKey(attributes, "max");
    if (m_bMinLimit)
        m_min = atof(attributes["min"].c_str());
    if (m_bMaxLimit)
        m_max = atof(attributes["max"].c_str());
    double val;
    do {
        val = omnetpp::exponential(getEnvir()->getRNG(0), m_mean);
    } while (val < m_min || val > m_max);
    return val;

}

simtime_t FailureModule::rdNormal(cXMLAttributeMap &attributes)
{

    if (!_hasKey(attributes, "mean"))
        throw cRuntimeError(
                "Undefined parameter for random distribution. Mean must be defined for an normal distribution");
    double m_mean = atof(attributes["mean"].c_str());

    if (!_hasKey(attributes, "stddev"))
        throw cRuntimeError(
                "Undefined parameter for random distribution. stddev must be defined for an normal distribution");
    double m_stddev = atof(attributes["stddev"].c_str());
    return omnetpp::normal(getEnvir()->getRNG(0), m_mean, m_stddev);
}

simtime_t FailureModule::rdConstant(cXMLAttributeMap &attributes)
{
    if (!_hasKey(attributes, "value"))
        throw "No value specified";
    double m_value = atof(attributes["value"].c_str());
    return m_value;
}

simtime_t FailureModule::procDistribution(cXMLAttributeMap &attributes)
{
    std::string typeName = attributes["distribution"];
    std::string dt;
    if (typeName == "normal")
        return rdNormal(attributes);
    else if (typeName == "uniform")
        return rdUniform(attributes);
    else if (typeName == "exponential")
        return rdExponential(attributes);
    else if (typeName == "constant")
        return rdConstant(attributes);
    else {
        throw cRuntimeError("Distribucion no encontrada");
        return 0;
    }
}

FailureModule::FailureModule()
{
    timer = new cMessage("Failure-timer");
}

FailureModule::~FailureModule()
{
    // TODO Auto-generated destructor stub
    cancelAndDelete(timer);
}

LinkId FailureModule::getNode(cXMLAttributeMap attributes)
{
// enlace random

    if (!_hasKey(attributes, "node") && _hasKey(attributes, "link")) {
        // random
        int numNodes = topo->getNumNodes();
        int nodeindex = omnetpp::intuniform(getEnvir()->getRNG(0), 0, numNodes - 1);
        cTopology::Node * node = topo->getNode(nodeindex);
        int numlink = node->getNumOutLinks();
        int linkid = omnetpp::intuniform(getEnvir()->getRNG(0), 0, numlink - 1);
        int remoteNodeId = node->getLinkOut(linkid)->getRemoteNode()->getModule()->par("address");
        return std::make_pair(topo->getNode(nodeindex)->getModule()->par("address").longValue(), remoteNodeId);
    }
    // nodo completo
    else if (!_hasKey(attributes, "node")) {
        // random
        int numNodes = topo->getNumNodes();
        int nodeindex = omnetpp::intuniform(getEnvir()->getRNG(0), 0, numNodes - 1);
        return std::make_pair(topo->getNode(nodeindex)->getModule()->par("address").longValue(), -1);
    }
    // enlace de un nodo concreto, pero eleccion random
    else if (_hasKey(attributes, "link") || !_hasKey(attributes, "remotenode")) {
        int nodeId = atoi(attributes["node"].c_str());
        int nodeIndex = -1;
        int numNodes = topo->getNumNodes();
        for (int i = 0; i < numNodes; i++) {
            int addr = topo->getNode(i)->getModule()->par("address");
            if (nodeId == addr) {
                nodeIndex = i;
                break;

            }
        }
        cTopology::Node * node = topo->getNode(nodeIndex);
        int numlink = node->getNumOutLinks();
        int linkid = omnetpp::intuniform(getEnvir()->getRNG(0), 0, numlink - 1);
        cTopology::Node *remote = node->getLinkOut(linkid)->getRemoteNode();
        return std::make_pair(nodeId, remote->getModule()->par("address").longValue());
    }
    // enlace concreto entre 2 nodos
    else if (_hasKey(attributes, "remotenode")) {
        int nodeId = atoi(attributes["node"].c_str());
        int nodeIndex = -1;
        int numNodes = topo->getNumNodes();
        for (int i = 0; i < numNodes; i++) {
            int addr = topo->getNode(i)->getModule()->par("address");
            if (nodeId == addr) {
                nodeIndex = i;
                break;

            }
        }

        if (nodeIndex == -1)
            throw cRuntimeError("");
        int remoteNodeIndex = -1;
        int remoteNodeId = atoi(attributes["remotenode"].c_str());
        cTopology::Node * node = topo->getNode(nodeIndex);
        for (int i = 0; i < node->getNumOutLinks(); i++) {
            int addr = topo->getNode(i)->getModule()->par("address");
            if (remoteNodeId == addr) {
                remoteNodeIndex = i;
                break;

            }
        }

        if (remoteNodeIndex == -1)
            throw cRuntimeError("No existe enlace entre nodos");
        return std::make_pair(nodeId, remoteNodeId);
    }
    else
        throw cRuntimeError("Imposible determinar enlace/nodo");
    return std::make_pair(-1, -1);
}

simtime_t FailureModule::createEvent(const LinkId &nodeId, cXMLAttributeMap &attributes, const simtime_t &base, const CONFIGURATION_TYPE &evType)
{
    simtime_t start;
    simtime_t distTime;
    bool hasStart = false;
    bool hasDistribution = false;
    bool absolutetime = false;
    if (_hasKey(attributes, "start")) {
        start = atof(attributes["start"].c_str());
        hasStart = true;
    }
    if (_hasKey(attributes, "distribution")) {
        hasDistribution = true;
        distTime = procDistribution(attributes);
    }
    if (!hasStart && !hasDistribution)
        throw cRuntimeError("No tiene referencia temporal de inicio, ni start ni distribution");

    simtime_t timeEvent = start + distTime;
    if (!absolutetime)
        timeEvent += base;

    // introducir evento en la lista.
    Event event;
    event.linkId = nodeId;
    if (evType == FAILURE) {
        if (nodeId.second == -1)
            event.type = NODE_FAILURE_EV;
        else
            event.type = LINK_FAILURE_EV;
    }
    else {
        if (nodeId.second == -1)
            event.type = NODE_RECOVERY_EV;
        else
            event.type = LINK_RECOVERY_EV;
    }
    eventList.insert(std::make_pair(timeEvent, event));
    return timeEvent;
}

simtime_t FailureModule::create(const LinkId &nodeId, cXMLElement *element,const simtime_t &base)
{
    cXMLAttributeMap attributes = element->getAttributes();
    simtime_t baseValue = base;
    if (_hasKey(attributes, "type")) {
        CONFIGURATION_TYPE dt;
        if (attributes["type"] == "FAILURE") {
            dt = FAILURE;
        }
        else if (attributes["type"] == "RECOVERY") {
            dt = RECOVERY;
        }
        else
            throw cRuntimeError("Action type must be FAILURE or RECOVERY");
        baseValue = createEvent(nodeId, attributes, baseValue, dt);
    }
    else
        throw cRuntimeError("Action doesn't have type");
    return baseValue;
}

void FailureModule::parser(cXMLElement *rootelement)
{
    if (!rootelement->hasChildren())
        return;

    topo = new cTopology("topo");
    topo->extractByProperty("node");

    cXMLElementList elements = rootelement->getChildrenByTagName("eventsnode");
    for (auto & element : elements) {
        cXMLAttributeMap attributes = element->getAttributes();
        std::pair<int, int> nodeId = getNode(attributes);
        cXMLElementList actionsList = element->getChildrenByTagName("action");
        int repeat = 1;
        simtime_t basetime;
        if (_hasKey(attributes, "basetime"))
            basetime = atof(attributes["basetime"].c_str());

        if (_hasKey(attributes, "repeat"))
            repeat = atoi(attributes["repeat"].c_str());
        for (int i = 0; i < repeat; i++) {
            for (auto & action : actionsList) {
                 basetime = create(nodeId, action, basetime);
            }
        }
    }
    delete topo;

    // comprobar integridad de la lista de eventos
    if (eventList.empty())
        return;

    EventList temporalList = eventList;
    std::vector<std::pair<simtime_t, Event> > eraseEvents;

    while (!temporalList.empty()) {
        std::vector<std::pair<simtime_t, Event> > eventNodeList;
        Event event = temporalList.begin()->second;
        eventNodeList.push_back(std::make_pair(temporalList.begin()->first, event));
        temporalList.erase(temporalList.begin());

        for (auto it = temporalList.begin(); it != temporalList.end();) {
            if (it->second.linkId == event.linkId) {
                eventNodeList.push_back(std::make_pair(it->first, it->second));
                temporalList.erase(it++);
            }
            else
                ++it;
        }

        bool stateRecovery = true;
        // check the sequence
        for (auto &elem : eventNodeList) {
            if (stateRecovery && elem.second.type != NODE_FAILURE_EV && elem.second.type != LINK_FAILURE_EV) {
                // It should be a failure the first event, not a recovery
                //throw cRuntimeError("Event sequence erroneous, this event for link %i - %i should be a failure", event.linkId.first, event.linkId.second);
                // erase this event
                for (auto itAux = eventList.begin(); itAux != eventList.end();) {
                    if (itAux->first == elem.first && itAux->second == elem.second)
                        eventList.erase(itAux++);
                    else
                        ++itAux;
                }
            }
            else if (!stateRecovery && (elem.second.type == NODE_FAILURE_EV || elem.second.type == LINK_FAILURE_EV)) {
                // It should be a recovery,
                //throw cRuntimeError("Event sequence erroneous, this event should be a recovery", event.linkId.first, event.linkId.second);
                for (auto itAux = eventList.begin(); itAux != eventList.end();) {
                    if (itAux->first == elem.first && itAux->second == elem.second)
                        eventList.erase(itAux++);
                    else
                        ++itAux;
                }
            }
            stateRecovery = !stateRecovery;
        }
    }
}

void FailureModule::initialize(int stage)
{
    if (stage < numInitStages() - 1)
        return;
    eventSignal = registerSignal("EventSignal");
    configuration = par("config");
    parser(configuration);
    if (!eventList.empty())
        scheduleAt(eventList.begin()->first, timer);
}

void FailureModule::handleMessage(cMessage *msg)
{

    if (msg == timer) {
        while (eventList.begin()->first <= simTime()) {
            emit(eventSignal, &eventList.begin()->second);
            eventList.erase(eventList.begin());
        }
    }
    if (!eventList.empty())
        scheduleAt(eventList.begin()->first, timer);
}
