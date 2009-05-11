
/*
 * AbstractMemOrCache.h
 *
 * Description:
 *
 *
 */

#ifndef ABSTRACT_MEM_OR_CACHE_H
#define ABSTRACT_MEM_OR_CACHE_H

#include "mem/ruby/common/Global.hh"
#include "mem/ruby/slicc_interface/AbstractChip.hh"
#include "mem/ruby/config/RubyConfig.hh"
#include "mem/ruby/common/Address.hh"

class AbstractMemOrCache {
public:

  virtual ~AbstractMemOrCache() {};
  virtual void setConsumer(Consumer* consumer_ptr) = 0;
  virtual Consumer* getConsumer() = 0;

  virtual void enqueue (const MsgPtr& message, int latency ) = 0;
  virtual void enqueueMemRef (MemoryNode& memRef) = 0;
  virtual void dequeue () = 0;
  virtual const Message* peek () = 0;
  virtual bool isReady () = 0;
  virtual MemoryNode peekNode () = 0;
  virtual bool areNSlotsAvailable (int n) = 0;
  virtual void printConfig (ostream& out) = 0;
  virtual void print (ostream& out) const = 0;
  virtual void setDebug (int debugFlag) = 0;

private:

};


#endif

