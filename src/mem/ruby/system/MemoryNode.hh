/*
 * Copyright (c) 1999 by Mark Hill and David Wood for the Wisconsin
 * Multifacet Project.  ALL RIGHTS RESERVED.
 *
 * ##HEADER##
 *
 * This software is furnished under a license and may be used and
 * copied only in accordance with the terms of such license and the
 * inclusion of the above copyright notice.  This software or any
 * other copies thereof or any derivative works may not be provided or
 * otherwise made available to any other persons.  Title to and
 * ownership of the software is retained by Mark Hill and David Wood.
 * Any use of this software must include the above copyright notice.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS".  THE LICENSOR MAKES NO
 * WARRANTIES ABOUT ITS CORRECTNESS OR PERFORMANCE.
 * */

/*
 * Description:
 * This structure records everything known about a single
 * memory request that is queued in the memory controller.
 * It is created when the memory request first arrives
 * at a memory controller and is deleted when the underlying
 * message is enqueued to be sent back to the directory.
 */

#ifndef MEMORYNODE_H
#define MEMORYNODE_H

#include "Global.hh"
#include "Message.hh"
#include "MemoryRequestType.hh"

class MemoryNode {

public:
  // Constructors

// old one:
  MemoryNode(const Time& time, int counter, const MsgPtr& msgptr, const physical_address_t addr, const bool is_mem_read) {
    m_time = time;
    m_msg_counter = counter;
    m_msgptr = msgptr;
    m_addr = addr;
    m_is_mem_read = is_mem_read;
    m_is_dirty_wb = !is_mem_read;
  }

// new one:
  MemoryNode(const Time& time, const MsgPtr& msgptr, const physical_address_t addr, const bool is_mem_read, const bool is_dirty_wb) {
    m_time = time;
    m_msg_counter = 0;
    m_msgptr = msgptr;
    m_addr = addr;
    m_is_mem_read = is_mem_read;
    m_is_dirty_wb = is_dirty_wb;
  }

  // Destructor
  ~MemoryNode() {};

  // Public Methods
  void print(ostream& out) const;

  // Data Members (m_ prefix)  (all public -- this is really more a struct)

  Time m_time;
  int m_msg_counter;
  MsgPtr m_msgptr;
  physical_address_t m_addr;
  bool m_is_mem_read;
  bool m_is_dirty_wb;
};

// Output operator declaration
ostream& operator<<(ostream& out, const MemoryNode& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const MemoryNode& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //MEMORYNODE_H
