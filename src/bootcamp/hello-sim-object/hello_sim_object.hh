#ifndef __BOOTCAMP_HELLO_SIM_OBJECT_HELLO_SIM_OBJECT_HH__
#define __BOOTCAMP_HELLO_SIM_OBJECT_HELLO_SIM_OBJECT_HH__

// Name of include guard should match the path of the header file.
//
// Before compiling SimObjects, gem5 creates those structures.
// The file below is auto-generated. It contains a struct with the
// parameter definitions. We include this so we can use
// EventFunctionWrapper.

#include "bootcamp/hello-sim-object/goodbye_sim_object.hh"
#include "params/HelloSimObject.hh"
#include "sim/eventq.hh"
#include "sim/sim_object.hh"

namespace gem5
{
    class HelloSimObject: public SimObject
    {
        public:
            HelloSimObject(const HelloSimObjectParams& params);
            virtual void startup() override;
            // ip are the params of the HelloSimObjectParams type
            // the constructor takes exactly one argument
        private:
            // an object function wrapper
            EventFunctionWrapper nextHelloEvent;
            // the below function returns and takes void
            void processNextHelloEvent(void);
            int remainingHellosToPrintByEvent;
            GoodByeSimObject* goodByeObject;
        };
}	// namespace gem5

#endif	// header endif
