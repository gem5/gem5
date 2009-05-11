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
 * MemoryNode.C
 *
 * Description: See MemoryNode.h
 *
 * $Id: MemoryNode.C 1.3 04/08/04 14:15:38-05:00 beckmann@c2-141.cs.wisc.edu $
 *
 */

#include "mem/ruby/system/MemoryNode.hh"

void MemoryNode::print(ostream& out) const
{
  out << "[";
  out << m_time << ", ";
  out << m_msg_counter << ", ";
  out << m_msgptr << "; ";
  out << "]";
}
