#include "mem/ruby/network/garnet/fixed-pipeline/routing/aco/Pheromone.hh"

using namespace std;

Pheromone::Pheromone(int neighbor, double value)
{
    m_neighbor = neighbor;
    m_value = value;
}

int
Pheromone::getNeighbor()
{
    return m_neighbor;
}

void
Pheromone::setValue(double value)
{
    m_value = value;
}

double
Pheromone::getValue()
{
    return m_value;
}