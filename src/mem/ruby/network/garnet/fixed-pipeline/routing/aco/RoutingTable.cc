#include <stdlib.h>
#include <algorithm>

#include "base/misc.hh"
#include "mem/ruby/network/garnet/NetworkHeader.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/routing/aco/AntNetAgent.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/routing/aco/RoutingTable.hh"

using namespace std;

const double REINFORCEMENT_FACTOR = 0.05d;

RoutingTable::RoutingTable(AntNetAgent *agent)
{
    m_agent = agent;

    srand(time(NULL));
}

void
RoutingTable::addEntry(int destination, int neighbor, double pheromone_value)
{
    Pheromone *pheromone = new Pheromone(neighbor, pheromone_value);

    if(m_pheromones.count(destination) == 0)
    {
        m_pheromones[destination] = vector<Pheromone*>();
    }

    m_pheromones[destination].push_back(pheromone);
}

int
RoutingTable::calculateRandomDestination(int source)
{
    int num_routers = m_agent->getNumRouters();

    int i;
    do
    {
        i = rand() % num_routers;
    }
    while (i == source);

    return i;
}

int
RoutingTable::calculateNeighbor(int destination, int parent)
{
    if(destination == m_agent->getRouter())
    {
        fatal("destination should not be equal to the current router");
        return 0;
    }

    if(count(m_agent->getLinks().begin(), m_agent->getLinks().end(), destination) > 0)
    {
        return destination;
    }

    vector<Pheromone*> pheromones_per_destination = m_pheromones[destination];

    double max_pheromone_value = 0.0d;
    int max_pheromone_neighbor = INFINITE_;

    for(vector<Pheromone*>::iterator it = pheromones_per_destination.begin(); it != pheromones_per_destination.end(); ++it)
    {
        Pheromone *pheromone = *it;

        int neighbor = pheromone->getNeighbor();
        double pheromone_value = pheromone->getValue();

        if(neighbor != parent && pheromone_value > max_pheromone_value)
        {
            max_pheromone_value = pheromone_value;
            max_pheromone_neighbor = neighbor;
        }
    }

    if(max_pheromone_neighbor == INFINITE_)
    {
        fatal("max_pheromone_neighbor should not be INFINITE_");
        return max_pheromone_neighbor;
    }

    return max_pheromone_neighbor;
}

void
RoutingTable::update(int destination, int neighbor)
{
    vector<Pheromone*> pheromones_per_destination = m_pheromones[destination];

    for(vector<Pheromone*>::iterator it = pheromones_per_destination.begin(); it != pheromones_per_destination.end(); it++)
    {
        Pheromone *pheromone = *it;

        if(pheromone->getNeighbor() == neighbor)
        {
            pheromone->setValue(pheromone->getValue() + REINFORCEMENT_FACTOR * (1 - pheromone->getValue()));
        }
        else
        {
            pheromone->setValue(pheromone->getValue() * (1 - REINFORCEMENT_FACTOR));
        }
    }
}

AntNetAgent *
RoutingTable::getAgent()
{
    return m_agent;
}

map<int, vector<Pheromone*>>&
RoutingTable::getPheromones()
{
    return m_pheromones;
}