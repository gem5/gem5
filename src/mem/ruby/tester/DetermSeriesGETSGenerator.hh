
/*
    Copyright (C) 1999-2008 by Mark D. Hill and David A. Wood for the
    Wisconsin Multifacet Project.  Contact: gems@cs.wisc.edu
    http://www.cs.wisc.edu/gems/

    --------------------------------------------------------------------

    This file is part of the Ruby Multiprocessor Memory System Simulator, 
    a component of the Multifacet GEMS (General Execution-driven 
    Multiprocessor Simulator) software toolset originally developed at 
    the University of Wisconsin-Madison.

    Ruby was originally developed primarily by Milo Martin and Daniel
    Sorin with contributions from Ross Dickson, Carl Mauer, and Manoj
    Plakal.

    Substantial further development of Multifacet GEMS at the
    University of Wisconsin was performed by Alaa Alameldeen, Brad
    Beckmann, Jayaram Bobba, Ross Dickson, Dan Gibson, Pacia Harper,
    Derek Hower, Milo Martin, Michael Marty, Carl Mauer, Michelle Moravan,
    Kevin Moore, Andrew Phelps, Manoj Plakal, Daniel Sorin, Haris Volos, 
    Min Xu, and Luke Yen.
    --------------------------------------------------------------------

    If your use of this software contributes to a published paper, we
    request that you (1) cite our summary paper that appears on our
    website (http://www.cs.wisc.edu/gems/) and (2) e-mail a citation
    for your published paper to gems@cs.wisc.edu.

    If you redistribute derivatives of this software, we request that
    you notify us and either (1) ask people to register with us at our
    website (http://www.cs.wisc.edu/gems/) or (2) collect registration
    information and periodically send it to us.

    --------------------------------------------------------------------

    Multifacet GEMS is free software; you can redistribute it and/or
    modify it under the terms of version 2 of the GNU General Public
    License as published by the Free Software Foundation.

    Multifacet GEMS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with the Multifacet GEMS; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
    02111-1307, USA

    The GNU General Public License is contained in the file LICENSE.

### END HEADER ###
*/

/*
 * $Id$
 *
 * Description: 
 *
 */

// This Deterministic Generator generates a series of GETS requests for a given node.
// Sequentially goes through all nodes in the system
// This generator is used to tune the HW prefetcher
// The GETS requests are generated one at a time in round-robin fashion 0...1...2...etc.

#ifndef DETERMSERIESGETSGENERATOR_H
#define DETERMSERIESGETSGENERATOR_H

#include "mem/ruby/tester/Tester_Globals.hh"
#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/common/Global.hh"
#include "mem/protocol/DetermSeriesGETSGeneratorStatus.hh"
#include "mem/ruby/tester/SpecifiedGenerator.hh"

class DeterministicDriver;

class DetermSeriesGETSGenerator : public SpecifiedGenerator {
public:
  // Constructors
  DetermSeriesGETSGenerator(NodeID node, DeterministicDriver& driver);

  // Destructor
  ~DetermSeriesGETSGenerator();
  
  // Public Methods
  void wakeup();
  void performCallback(NodeID proc, Address address);

  void print(ostream& out) const;
private:
  // Private Methods
  int thinkTime() const;
  int waitTime() const;
  void initiateLoad();
  void pickAddress();

  // copy constructor and assignment operator
  DetermSeriesGETSGenerator(const DetermSeriesGETSGenerator& obj);
  DetermSeriesGETSGenerator& operator=(const DetermSeriesGETSGenerator& obj);

  // Data Members (m_ prefix)
  DetermSeriesGETSGeneratorStatus m_status;
  int m_counter;
  Address m_address;
  NodeID m_node;
  DeterministicDriver& m_driver;
  Time m_last_transition;
};

// Output operator declaration
ostream& operator<<(ostream& out, const DetermSeriesGETSGenerator& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline 
ostream& operator<<(ostream& out, const DetermSeriesGETSGenerator& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //DETERMSeriesGETSGENERATOR_H

