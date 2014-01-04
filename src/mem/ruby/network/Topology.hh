/*
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
 * The topology here is configurable; it can be a hierachical (default
 * one) or a 2D torus or a 2D torus with half switches killed. I think
 * all input port has a one-input-one-output switch connected just to
 * control and bandwidth, since we don't control bandwidth on input
 * ports.  Basically, the class has a vector of nodes and edges. First
 * 2*m_nodes elements in the node vector are input and output
 * ports. Edges are represented in two vectors of src and dest
 * nodes. All edges have latency.
 */

#ifndef __MEM_RUBY_NETWORK_TOPOLOGY_HH__
#define __MEM_RUBY_NETWORK_TOPOLOGY_HH__

#include <iostream>
#include <string>
#include <vector>

#include "mem/protocol/LinkDirection.hh"
#include "mem/ruby/common/TypeDefines.hh"
#include "mem/ruby/network/BasicLink.hh"

class NetDest;
class Network;

typedef std::vector<std::vector<int> > Matrix;

struct LinkEntry 
{
    BasicLink *link;
    LinkDirection direction;
};

typedef std::map<std::pair<SwitchID, SwitchID>, LinkEntry> LinkMap;

class Topology
{
  public:
    Topology(uint32_t num_routers, std::vector<BasicExtLink *> ext_links,
             std::vector<BasicIntLink *> int_links);

    uint32_t numSwitches() const { return m_number_of_switches; }
    void createLinks(Network *net);
    void print(std::ostream& out) const { out << "[Topology]"; }

  protected:
    void addLink(SwitchID src, SwitchID dest, BasicLink* link,
                 LinkDirection dir);
    void makeLink(Network *net, SwitchID src, SwitchID dest,
                  const NetDest& routing_table_entry);

    NodeID m_nodes;
    uint32_t m_number_of_switches;

    std::vector<BasicExtLink*> m_ext_link_vector;
    std::vector<BasicIntLink*> m_int_link_vector;

    Matrix m_component_latencies;
    Matrix m_component_inter_switches;

    LinkMap m_link_map;
};

inline std::ostream&
operator<<(std::ostream& out, const Topology& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

#endif // __MEM_RUBY_NETWORK_TOPOLOGY_HH__
