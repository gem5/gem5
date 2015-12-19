#ifndef __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_ROUTING_ACO_PHEROMONE_HH__
#define __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_ROUTING_ACO_PHEROMONE_HH__

#include <iostream>
#include <vector>

#include "mem/ruby/common/TypeDefines.hh"

// Pheromone.
class Pheromone
{
    public:
        // Create a pheromone.
        Pheromone(int neighbor, double value);

        // Get the neighbor router.
        int getNeighbor();

        // Set the pheromone's value.
        void setValue(double value);

        // Get the pheromone's value.
        double getValue();

    private:
        int m_neighbor;
        double m_value;

};


#endif // __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_ROUTING_ACO_PHEROMONE_HH__