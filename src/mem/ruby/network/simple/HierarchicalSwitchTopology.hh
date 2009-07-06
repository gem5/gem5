
#ifndef HIERARCHICALSWITCHTOPOLOGY_H
#define HIERARCHICALSWITCHTOPOLOGY_H

#include "mem/ruby/network/simple/Topology.hh"

class HierarchicalSwitchTopology : public Topology
{
public:
  HierarchicalSwitchTopology(const string & name);
  void init(const vector<string> & argv);

protected:
  void construct();
};

#endif
