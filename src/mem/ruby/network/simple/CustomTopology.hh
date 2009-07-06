
#ifndef CUSTOMTOPOLOGY_H
#define CUSTOMTOPOLOGY_H

#include "mem/ruby/network/simple/Topology.hh"

class CustomTopology : public Topology
{
public:
  CustomTopology(const string & name);
  void init(const vector<string> & argv);

protected:
  void construct();
};

#endif
