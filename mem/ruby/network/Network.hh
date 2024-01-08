/*
 * Copyright (c) 2017,2021 ARM Limited
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * The Network class is the base class for classes that implement the
 * interconnection network between components (processor/cache
 * components and memory/directory components).  The interconnection
 * network as described here is not a physical network, but a
 * programming concept used to implement all communication between
 * components.  Thus parts of this 'network' will model the on-chip
 * connections between cache controllers and directory controllers as
 * well as the links between chip and network switches.
 */

#ifndef __MEM_RUBY_NETWORK_NETWORK_HH__
#define __MEM_RUBY_NETWORK_NETWORK_HH__

#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "base/addr_range.hh"
#include "base/types.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "mem/ruby/common/MachineID.hh"
#include "mem/ruby/common/TypeDefines.hh"
#include "mem/ruby/network/Topology.hh"
#include "mem/ruby/network/dummy_port.hh"
#include "mem/ruby/protocol/LinkDirection.hh"
#include "mem/ruby/protocol/MessageSizeType.hh"
#include "params/RubyNetwork.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

namespace ruby
{

class NetDest;
class MessageBuffer;

class Network : public ClockedObject
{
  public:
    PARAMS(RubyNetwork);
    Network(const Params &p);

    virtual ~Network();

    static uint32_t getNumberOfVirtualNetworks() { return m_virtual_networks; }
    int getNumNodes() const { return m_nodes; }

    static uint32_t MessageSizeType_to_int(MessageSizeType size_type);

    // returns the queue requested for the given component
    void setToNetQueue(NodeID global_id, bool ordered, int netNumber,
                               std::string vnet_type, MessageBuffer *b);
    virtual void setFromNetQueue(NodeID global_id, bool ordered, int netNumber,
                                 std::string vnet_type, MessageBuffer *b);

    virtual void checkNetworkAllocation(NodeID local_id, bool ordered,
                                       int network_num, std::string vnet_type);

    virtual void makeExtOutLink(SwitchID src, NodeID dest, BasicLink* link,
                             std::vector<NetDest>& routing_table_entry) = 0;
    virtual void makeExtInLink(NodeID src, SwitchID dest, BasicLink* link,
                            std::vector<NetDest>& routing_table_entry) = 0;
    virtual void makeInternalLink(SwitchID src, SwitchID dest, BasicLink* link,
                                  std::vector<NetDest>& routing_table_entry,
                                  PortDirection src_outport,
                                  PortDirection dst_inport) = 0;

    virtual void collateStats() = 0;
    virtual void print(std::ostream& out) const = 0;

    /*
     * Virtual functions for functionally reading and writing packets in
     * the network. Each network needs to implement these for functional
     * accesses to work correctly.
     */
    virtual bool functionalRead(Packet *pkt)
    { fatal("Functional read not implemented.\n"); }
    virtual bool functionalRead(Packet *pkt, WriteMask& mask)
    { fatal("Masked functional read not implemented.\n"); }
    virtual uint32_t functionalWrite(Packet *pkt)
    { fatal("Functional write not implemented.\n"); }

    /**
     * Map an address to the correct NodeID
     *
     * This function traverses the global address map to find the
     * NodeID that corresponds to the given address and the type of
     * the destination. For example for a request to a directory this
     * function will return the NodeID of the right directory.
     *
     * @param the destination address
     * @param the type of the destination
     * @return the NodeID of the destination
     */
    NodeID addressToNodeID(Addr addr, MachineType mtype);

    Port &
    getPort(const std::string &, PortID idx=InvalidPortID) override
    {
        return RubyDummyPort::instance();
    }

    NodeID getLocalNodeID(NodeID global_id) const;

  protected:
    // Private copy constructor and assignment operator
    Network(const Network& obj);
    Network& operator=(const Network& obj);

    uint32_t m_nodes;
    static uint32_t m_virtual_networks;
    std::vector<std::string> m_vnet_type_names;
    Topology* m_topology_ptr;
    static uint32_t m_control_msg_size;
    static uint32_t m_data_msg_size;

    // vector of queues from the components
    std::vector<std::vector<MessageBuffer*> > m_toNetQueues;
    std::vector<std::vector<MessageBuffer*> > m_fromNetQueues;
    std::vector<bool> m_ordered;

  private:
    // Global address map
    struct AddrMapNode
    {
        NodeID id;
        AddrRangeList ranges;
    };
    std::unordered_multimap<MachineType, AddrMapNode> addrMap;

    // Global NodeID to local node map. If there are not multiple networks in
    // the same RubySystem, this is a one-to-one mapping of global to local.
    std::unordered_map<NodeID, NodeID> globalToLocalMap;
};

inline std::ostream&
operator<<(std::ostream& out, const Network& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_NETWORK_NETWORK_HH__
