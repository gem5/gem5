
#ifndef TORUS2DTOPOLOGY_H
#define TORUS2DTOPOLOGY_H

#include "mem/ruby/network/simple/Topology.hh"

class Torus2DTopology : public Topology
{
public:
  Torus2DTopology(const string & name);
  void init(const vector<string> & argv);

protected:
  void construct();
};

#endif
