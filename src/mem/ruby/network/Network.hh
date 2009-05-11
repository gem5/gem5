
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
 * Network.h
 *
 * Description: The Network class is the base class for classes that
 * implement the interconnection network between components
 * (processor/cache components and memory/directory components).  The
 * interconnection network as described here is not a physical
 * network, but a programming concept used to implement all
 * communication between components.  Thus parts of this 'network'
 * will model the on-chip connections between cache controllers and
 * directory controllers as well as the links between chip and network
 * switches.
 *
 * $Id$
 * */

#ifndef NETWORK_H
#define NETWORK_H

#include "Global.hh"
#include "NodeID.hh"
#include "MessageSizeType.hh"

class NetDest;
class MessageBuffer;
class Throttle;

class Network {
public:
  // Constructors
  Network() {}

  // Destructor
  virtual ~Network() {}

  // Public Methods

  static Network* createNetwork(int nodes);

  // returns the queue requested for the given component
  virtual MessageBuffer* getToNetQueue(NodeID id, bool ordered, int netNumber) = 0;
  virtual MessageBuffer* getFromNetQueue(NodeID id, bool ordered, int netNumber) = 0;
  virtual const Vector<Throttle*>* getThrottles(NodeID id) const { return NULL; }

  virtual int getNumNodes() {return 1;}

  virtual void makeOutLink(SwitchID src, NodeID dest, const NetDest& routing_table_entry, int link_latency, int link_weight, int bw_multiplier, bool isReconfiguration) = 0;
  virtual void makeInLink(SwitchID src, NodeID dest, const NetDest& routing_table_entry, int link_latency, int bw_multiplier, bool isReconfiguration) = 0;
  virtual void makeInternalLink(SwitchID src, NodeID dest, const NetDest& routing_table_entry, int link_latency, int link_weight, int bw_multiplier, bool isReconfiguration) = 0;

  virtual void reset() = 0;

  virtual void printStats(ostream& out) const = 0;
  virtual void clearStats() = 0;
  virtual void printConfig(ostream& out) const = 0;
  virtual void print(ostream& out) const = 0;

private:

  // Private Methods
  // Private copy constructor and assignment operator
  Network(const Network& obj);
  Network& operator=(const Network& obj);

  // Data Members (m_ prefix)
};

// Output operator declaration
ostream& operator<<(ostream& out, const Network& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const Network& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

// Code to map network message size types to an integer number of bytes
const int CONTROL_MESSAGE_SIZE = 8;
const int DATA_MESSAGE_SIZE = (64+8);

extern inline
int MessageSizeType_to_int(MessageSizeType size_type)
{
  switch(size_type) {
  case MessageSizeType_Undefined:
    ERROR_MSG("Can't convert Undefined MessageSizeType to integer");
    break;
  case MessageSizeType_Control:
  case MessageSizeType_Request_Control:
  case MessageSizeType_Reissue_Control:
  case MessageSizeType_Response_Control:
  case MessageSizeType_Writeback_Control:
  case MessageSizeType_Forwarded_Control:
  case MessageSizeType_Invalidate_Control:
  case MessageSizeType_Unblock_Control:
  case MessageSizeType_Persistent_Control:
  case MessageSizeType_Completion_Control:
    return CONTROL_MESSAGE_SIZE;
    break;
  case MessageSizeType_Data:
  case MessageSizeType_Response_Data:
  case MessageSizeType_ResponseLocal_Data:
  case MessageSizeType_ResponseL2hit_Data:
  case MessageSizeType_Writeback_Data:
    return DATA_MESSAGE_SIZE;
    break;
  default:
    ERROR_MSG("Invalid range for type MessageSizeType");
    break;
  }
  return 0;
}

#endif //NETWORK_H
