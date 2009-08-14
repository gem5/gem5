
#ifndef ABSTRACTCONTROLLER_H
#define ABSTRACTCONTROLLER_H

#include "mem/ruby/common/Consumer.hh"
#include "mem/protocol/MachineType.hh"
#include "mem/ruby/common/Address.hh"

class MessageBuffer;
class Network;

class AbstractController : public Consumer {
public:
  AbstractController() {}
  virtual void init(Network* net_ptr, const vector<string> & argv) = 0;

  // returns the number of controllers created of the specific subtype
  //  virtual int getNumberOfControllers() const = 0;
  virtual MessageBuffer* getMandatoryQueue() const = 0;
  virtual const int & getVersion() const = 0;
  virtual const string toString() const = 0;  // returns text version of controller type
  virtual const string getName() const = 0;   // return instance name
  virtual const MachineType getMachineType() const = 0;
  virtual void set_atomic(Address addr) = 0;
  virtual void clear_atomic() = 0;

  virtual void print(ostream & out) const = 0;
  virtual void printStats(ostream & out) const = 0;
  virtual void printConfig(ostream & out) const = 0;
  virtual void wakeup() = 0;
  //  virtual void dumpStats(ostream & out) = 0;
  virtual void clearStats() = 0;

};

#endif
