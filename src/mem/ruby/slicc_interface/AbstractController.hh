
#ifndef ABSTRACTCONTROLLER_H
#define ABSTRACTCONTROLLER_H

#include "sim/sim_object.hh"
#include "params/RubyController.hh"

#include "mem/ruby/common/Consumer.hh"
#include "mem/protocol/MachineType.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/network/Network.hh"

class MessageBuffer;
class Network;

class AbstractController : public SimObject, public Consumer {
public:
    typedef RubyControllerParams Params;
    AbstractController(const Params *p) : SimObject(p) {}

  // returns the number of controllers created of the specific subtype
  //  virtual int getNumberOfControllers() const = 0;
  virtual MessageBuffer* getMandatoryQueue() const = 0;
  virtual const int & getVersion() const = 0;
  virtual const string toString() const = 0;  // returns text version of controller type
  virtual const string getName() const = 0;   // return instance name
  virtual const MachineType getMachineType() const = 0;
  virtual void blockOnQueue(Address, MessageBuffer*) = 0;
  virtual void unblock(Address) = 0;
  virtual void initNetworkPtr(Network* net_ptr) = 0;

  virtual void print(ostream & out) const = 0;
  virtual void printStats(ostream & out) const = 0;
  virtual void printConfig(ostream & out) const = 0;
  virtual void wakeup() = 0;
  //  virtual void dumpStats(ostream & out) = 0;
  virtual void clearStats() = 0;

};

#endif
