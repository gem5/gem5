
#include "cpu/thread_context.hh"
#include "cpu/quiesce_event.hh"

EndQuiesceEvent::EndQuiesceEvent(ThreadContext *_tc)
    : Event(&mainEventQueue), tc(_tc)
{
}

void
EndQuiesceEvent::process()
{
    tc->activate();
}

const char*
EndQuiesceEvent::description()
{
    return "End Quiesce Event.";
}
