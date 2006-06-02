
#include "cpu/exec_context.hh"
#include "cpu/quiesce_event.hh"

EndQuiesceEvent::EndQuiesceEvent(ExecContext *_xc)
    : Event(&mainEventQueue), xc(_xc)
{
}

void
EndQuiesceEvent::process()
{
    xc->activate();
}

const char*
EndQuiesceEvent::description()
{
    return "End Quiesce Event.";
}
