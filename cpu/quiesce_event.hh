#ifndef __CPU_QUIESCE_EVENT_HH__
#define __CPU_QUIESCE_EVENT_HH__

#include "sim/eventq.hh"

class ExecContext;

/** Event for timing out quiesce instruction */
struct EndQuiesceEvent : public Event
{
    /** A pointer to the execution context that is quiesced */
    ExecContext *xc;

    EndQuiesceEvent(ExecContext *_xc);

    /** Event process to occur at interrupt*/
    virtual void process();

    /** Event description */
    virtual const char *description();
};

#endif // __CPU_QUIESCE_EVENT_HH__
