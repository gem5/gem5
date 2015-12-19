#include <iostream>
#include <vector>

#include "mem/ruby/network/Topology.hh"
#include "mem/ruby/network/Network.hh"
#include "mem/ruby/network/MessageBuffer.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/Router_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/RoutingUnit_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/OutputUnit_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/NetworkLink_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/NetworkInterface_d.hh"
#include "mem/ruby/slicc_interface/Message.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/GarnetNetwork_d.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/routing/aco/AntNetAgent.hh"

using namespace std;

AntNetAgent::AntNetAgent(RoutingUnit_d *routing_unit, int router)
{
    m_routing_unit = routing_unit;
    m_router = router;
    m_routing_table = new RoutingTable(this);
}

void
AntNetAgent::init()
{
    int num_routers = getNumRouters();

    vector<int> links = getLinks();

    double pheromoneValue = 1.0d / links.size();

    for(int router = 0; router < num_routers; router++)
    {
        if(router != m_router)
        {
            for(vector<int>::iterator it = links.begin(); it != links.end(); it++)
            {
                int neighbor = *it;

                if(neighbor != m_router)
                {
                    m_routing_table->addEntry(router, neighbor, pheromoneValue);
                    cout << "#" << m_router << ".routing_table.addEntry(router=#" << router << ", neighbor=#" << neighbor << ", pheromoneValue=" << pheromoneValue << ")" << endl;
                }
            }
        }
    }

//    TODO:add_periodic_event
    createAndSendForwardAntMessage();
}

void
AntNetAgent::createAndSendForwardAntMessage()
{
    int destination = m_routing_table->calculateRandomDestination(m_router);

    Tick current_time = m_routing_unit->getRouter()->clockEdge();

    AntMsg* message = new AntMsg(current_time, true, m_router, destination);
    memorize(message);

    int neighbor = m_routing_table->calculateNeighbor(destination, m_router);
    sendMessage(message, neighbor);
}

//TODO: to be called by NetworkInterface_d::wakeup
void
AntNetAgent::wakeup()
{
    GarnetNetwork_d *network = m_routing_unit->getRouter()->get_net_ptr();
    NetworkInterface_d *ni = network->get_nis()[m_router];

    Tick current_time = m_routing_unit->getRouter()->clockEdge();

    int vnet = 1; //TODO: to be set to 4, indicating a dedicated vnet for ant net message transmission

    MessageBuffer *m_requestToDir_ptr = ni->get_inNode_ptr()[vnet];

    if (m_requestToDir_ptr->isReady(m_routing_unit->getRouter()->clockEdge())) {
        const AntMsg* message = dynamic_cast<const AntMsg *>(m_requestToDir_ptr->peek());

        if (message == NULL) {
            fatal("AntNetAgent: rejected message at the head of m_requestToDir_ptr");
        }

        m_requestToDir_ptr->dequeue(current_time);

//        receiveAntMessage(message, -1); //TODO: parent
    }
}

//TODO: to be called by GEM5
void
AntNetAgent::receiveAntMessage(AntMsg* message, int parent)
{
    //TODO: perform actions on in_msg_ptr
    cout << "AntNetAgent::wakeup: " << *message << endl;

    if(message->isForward())
    {
        memorize(message);
        if(m_router != message->getDestinationRouter())
        {
            forwardAntMessage(message, parent);
        }
        else
        {
            createAndSendBackwardAntMessage(message);
        }
    }
    else
    {
        updateRoutingTable(message);
        if(m_router != message->getDestinationRouter())
        {
            backwardAntMessage(message);
        }
    }
}

void
AntNetAgent::forwardAntMessage(AntMsg* message, int parent)
{
    int neighbor = m_routing_table->calculateNeighbor(message->getDestinationRouter(), parent);
    sendMessage(message, neighbor);
}

void
AntNetAgent::createAndSendBackwardAntMessage(AntMsg* message)
{
    int temp = message->getSourceRouter();
    message->setSourceRouter(message->getDestinationRouter());
    message->setDestinationRouter(temp);

    message->setForward(false);

    int index = message->getMemory().size() - 2;
    sendMessage(message, message->getMemory()[index]);
}

void
AntNetAgent::backwardAntMessage(AntMsg* message)
{
    int index = -1;

    for(vector<int>::reverse_iterator it = message->getMemory().rbegin(); it != message->getMemory().rend(); it++)
    {
        int router = *it;
        if(m_router == router)
        {
            index = distance(message->getMemory().rend(), it);
            break;
        }
    }

    sendMessage(message, message->getMemory()[index - 1]);
}

void
AntNetAgent::updateRoutingTable(AntMsg* message)
{
    int index = -1;

    for(vector<int>::iterator it = message->getMemory().begin(); it != message->getMemory().end(); it++)
    {
        int router = *it;

        if(m_router == router)
        {
            index = distance(message->getMemory().begin(), it);
            break;
        }
    }

    int neighbor = message->getMemory()[index + 1];

    for(vector<int>::iterator it = message->getMemory().begin() + index + 1; it != message->getMemory().end(); it++)
    {
        int destination = *it;
        m_routing_table->update(destination, neighbor);
    }
}

void
AntNetAgent::memorize(AntMsg* message)
{
    message->getMemory().push_back(m_router);
}

void
AntNetAgent::sendMessage(AntMsg* message, int neighbor)
{
//    GarnetNetwork_d *network = m_routing_unit->getRouter()->get_net_ptr();
//    NetworkInterface_d *ni = network->get_nis()[m_router];
//
//    Tick current_time = m_routing_unit->getRouter()->clockEdge();
//
//    int vnet = 1; //TODO: to be set to 4, indicating a dedicated vnet for ant net message transmission
//
//TODO
//    ni->get_inNode_ptr()[vnet]->enqueue(message, current_time, m_routing_unit->getRouter()->cyclesToTicks(Cycles(1)));

    cout << "#" << m_router << ".sendMessage(source=#" << message->getSourceRouter() << ", destination=#" << message->getDestinationRouter() << ", neighbor=#" << neighbor << ")" << endl;
}

//TODO: to be called by GEM5
int
AntNetAgent::calculateNeighbor(int destination, int parent)
{
    return m_routing_table->calculateNeighbor(destination, parent);
}

int
AntNetAgent::getNumRouters()
{
    return m_routing_unit->getRouter()->get_net_ptr()->get_topology_ptr()->numSwitches();
}

int
AntNetAgent::getRouter()
{
    return m_router;
}

std::vector<int>&
AntNetAgent::getLinks()
{
    return m_routing_unit->get_neighbor_table();
}

RoutingTable *
AntNetAgent::getRoutingTable()
{
    return m_routing_table;
}