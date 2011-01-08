/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Nathan Binkert
 */

#ifndef __SIM_SIM_EVENTS_HH__
#define __SIM_SIM_EVENTS_HH__

#include "sim/eventq.hh"

//
// Event to terminate simulation at a particular cycle/instruction
//
class SimLoopExitEvent : public Event
{
  protected:
    // string explaining why we're terminating
    std::string cause;
    int code;
    Tick repeat;

  public:
    SimLoopExitEvent(const std::string &_cause, int c, Tick repeat = 0);

    std::string getCause() { return cause; }
    int getCode() { return code; }

    void process();     // process event

    virtual const char *description() const;
};

class CountedDrainEvent : public Event
{
  private:
    // Count of how many objects have not yet drained
    int count;

  public:
    CountedDrainEvent();

    void process();

    void setCount(int _count) { count = _count; }

    int getCount() { return count; }
};

//
// Event class to terminate simulation after 'n' related events have
// occurred using a shared counter: used to terminate when *all*
// threads have reached a particular instruction count
//
class CountedExitEvent : public Event
{
  private:
    std::string cause;  // string explaining why we're terminating
    int &downCounter;   // decrement & terminate if zero

  public:
    CountedExitEvent(const std::string &_cause, int &_downCounter);

    void process();     // process event

    virtual const char *description() const;
};


#endif  // __SIM_SIM_EVENTS_HH__
