#ifndef __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_ROUTING_ACO_ANT_NET_AGENT_HH__
#define __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_ROUTING_ACO_ANT_NET_AGENT_HH__

#include <iostream>
#include <vector>

#include "mem/ruby/common/TypeDefines.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/routing/aco/AntMsg.hh"
#include "mem/ruby/network/garnet/fixed-pipeline/routing/aco/RoutingTable.hh"

class RoutingUnit_d;

// Ant net agent.
class AntNetAgent
{
    public:
        // Create an ant net agent.
        AntNetAgent(RoutingUnit_d *routingUnit, int router);

        // Initialize.
        void init();

        // Create and send a forward ant message.
        void createAndSendForwardAntMessage();

        // Wakeup.
        void wakeup();

        // On receiving an ant message.
        void receiveAntMessage(AntMsg* message, int parent);

        // Send the specified forward ant message to the neighbor router.
        void forwardAntMessage(AntMsg* message, int parent);

        // Create and send a backward ant message.
        void createAndSendBackwardAntMessage(AntMsg* message);

        // Send the specified backward ant message to the neighbor router.
        void backwardAntMessage(AntMsg* message);

        // Update the routing table.
        void updateRoutingTable(AntMsg* message);

        // Build the memory of the specified forward ant message.
        void memorize(AntMsg* message);

        // Send the specified ant message to the neighbor router.
        void sendMessage(AntMsg* message, int neighbor);

        // Calculate the neighbor router for the specified destination router and parent router.
        int calculateNeighbor(int destination, int parent);

        // Get the routing unit.
        RoutingUnit_d *getRoutingUnit();

        // Get the number of routers.
        int getNumRouters();

        // Get the current router.
        int getRouter();

        // Get the links of the current router.
        std::vector<int>& getLinks();

        // Get the routing table.
        RoutingTable *getRoutingTable();

    private:
        RoutingUnit_d *m_routing_unit;
        int m_router;
        RoutingTable *m_routing_table;
};

#endif // __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_ROUTING_ACO_ANT_NET_AGENT_HH__