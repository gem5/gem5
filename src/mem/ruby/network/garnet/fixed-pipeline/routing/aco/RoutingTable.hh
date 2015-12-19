#ifndef __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_ROUTING_ACO_ROUTING_TABLE_HH__
#define __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_ROUTING_ACO_ROUTING_TABLE_HH__

#include <iostream>
#include <vector>
#include <map>

#include "mem/ruby/common/TypeDefines.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/routing/aco/Pheromone.hh"

class AntNetAgent;

// Routing table.
class RoutingTable
{
    public:
        // Create a routing table.
        RoutingTable(AntNetAgent *agent);

        // Add a routing table entry.
        void addEntry(int destination, int neighbor, double pheromone_value);

        // Calculate a randomly chosen destination router for the specified source router.
        int calculateRandomDestination(int source);

        // Calculate the neighbor (next hop) router for the specified destination and parent routers.
        int calculateNeighbor(int destination, int parent);

        // Update the routing table by incrementing or evaporating pheromone values.
        void update(int destination, int neighbor);

        // Get the ant net agent.
        AntNetAgent *getAgent();

        // Get the map of pheromones.
        std::map<int, std::vector<Pheromone*>>& getPheromones();

    private:
        AntNetAgent *m_agent;
        std::map<int, std::vector<Pheromone*>> m_pheromones;

};

#endif // __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_ROUTING_ACO_ROUTING_TABLE_HH__