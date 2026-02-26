#include "bootcamp/hello-sim-object/goodbye_sim_object.hh"

#include "base/trace.hh"
#include "debug/GoodByeExampleFlag.hh"

namespace gem5
{

GoodByeSimObject::GoodByeSimObject(const GoodByeSimObjectParams& params):
    SimObject(params),
    nextGoodByeEvent([this](){processNextGoodByeEvent();}, name()+\
"nextGoodByeEvent"),
    useless_var(params.useless_var)
{

};

void
GoodByeSimObject::processNextGoodByeEvent()
{
DPRINTF(GoodByeExampleFlag, "%s: GoodBye from GoodByeSimObject\n\
        ",__func__);
}

void
GoodByeSimObject::sayGoodBye()
{
    panic_if(nextGoodByeEvent.scheduled(),"GoodByeSimObject \
had already scheduled event\n");
    schedule(nextGoodByeEvent,curTick()+102);
}
}
