#ifndef __BOOTCAMP_HELLO_SIM_OBJECT_GOOD_BYE_SIM_OBJECT_HH__
#define __BOOTCAMP_HELLO_SIM_OBJECT_GOOD_BYE_SIM_OBJECT_HH__

#include "params/GoodByeSimObject.hh"
#include "sim/eventq.hh"
#include "sim/sim_object.hh"

namespace gem5
{
    class GoodByeSimObject: public SimObject
        {
            public:
                GoodByeSimObject(const GoodByeSimObjectParams& params);
                void sayGoodBye();
                int useless_var;
            private:
                EventFunctionWrapper nextGoodByeEvent;
                void processNextGoodByeEvent(void);
        };
}
#endif
