
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
 * Topology.hh
 *
 * Description: The topology here is configurable; it can be a hierachical
 *              (default one) or a 2D torus or a 2D torus with half switches
 *              killed. I think all input port has a
 *              one-input-one-output switch connected just to control and
 *              bandwidth, since we don't control bandwidth on input ports.
 *              Basically, the class has a vector of nodes and edges. First
 *              2*m_nodes elements in the node vector are input and output
 *              ports. Edges are represented in two vectors of src and dest
 *              nodes. All edges have latency.
 *
 * $Id$
 *
 * */

#ifndef TOPOLOGY_H
#define TOPOLOGY_H

#include "mem/ruby/common/Global.hh"
#include "mem/gems_common/Vector.hh"
#include "mem/ruby/system/NodeID.hh"
#include "sim/sim_object.hh"
#include "params/Topology.hh"

class Network;
class NetDest;

typedef Vector < Vector <int> > Matrix;

class Topology : public SimObject {
public:
  // Constructors
    typedef TopologyParams Params;
    Topology(const Params *p);

  // Destructor
  virtual ~Topology() {}

  virtual void init();

  // Public Methods
  void makeTopology();
  int numSwitches() const { return m_number_of_switches; }
  void createLinks(bool isReconfiguration);

  const string getName() { return m_name; }
  void printStats(ostream& out) const {}
  void clearStats() {}
  void printConfig(ostream& out) const;
  void print(ostream& out) const { out << "[Topology]"; }

protected:
  // Private Methods
  SwitchID newSwitchID();
  void addLink(SwitchID src, SwitchID dest, int link_latency);
  void addLink(SwitchID src, SwitchID dest, int link_latency, int bw_multiplier);
  void addLink(SwitchID src, SwitchID dest, int link_latency, int bw_multiplier, int link_weight);
  void makeLink(SwitchID src, SwitchID dest, const NetDest& routing_table_entry, int link_latency, int weight, int bw_multiplier, bool isReconfiguration);

  //  void makeSwitchesPerChip(Vector< Vector < SwitchID > > &nodePairs, Vector<int> &latencies, Vector<int> &bw_multis, int numberOfChips);

  string getDesignStr();
  // Private copy constructor and assignment operator
  Topology(const Topology& obj);
  Topology& operator=(const Topology& obj);

  // Data Members (m_ prefix)
  string m_name;
  bool m_print_config;
  Network* m_network_ptr;
  string m_connections;
  NodeID m_nodes;
  int m_number_of_switches;

  Vector<SwitchID> m_links_src_vector;
  Vector<SwitchID> m_links_dest_vector;
  Vector<int> m_links_latency_vector;
  Vector<int> m_links_weight_vector;
  Vector<int> m_bw_multiplier_vector;

  Matrix m_component_latencies;
  Matrix m_component_inter_switches;
};

// Output operator declaration
ostream& operator<<(ostream& out, const Topology& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const Topology& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif
