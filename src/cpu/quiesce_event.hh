#ifndef __CPU_QUIESCE_EVENT_HH__
#define __CPU_QUIESCE_EVENT_HH__

#include "sim/eventq.hh"

class ThreadContext;

/** Event for timing out quiesce instruction */
struct EndQuiesceEvent : public Event
{
    /** A pointer to the thread context that is quiesced */
    ThreadContext *tc;

    EndQuiesceEvent(ThreadContext *_tc);

    /** Event process to occur at interrupt*/
    virtual void process();

    /** Event description */
    virtual const char *description();
};

#endif // __CPU_QUIESCE_EVENT_HH__
