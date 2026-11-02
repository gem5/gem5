/*
 * Copyright 2026 Google LLC
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
 */

#ifndef __SYSTEMC_CORE_CLOCK_TICK_OPS_HH__
#define __SYSTEMC_CORE_CLOCK_TICK_OPS_HH__

/*
 * Free-function interface to sc_gem5::ClockTick, the scheduler event and
 * process behind each sc_clock edge, used by channel/sc_clock.cc. It needs
 * only forward declarations, so its callers do not have to see the
 * kernel's headers. ClockTick itself is defined in core/clock_tick.cc.
 */

namespace sc_core
{

class sc_clock;
class sc_time;

} // namespace sc_core

namespace sc_gem5
{

class ClockTick;

ClockTick *newClockTick(::sc_core::sc_clock *clock, bool to,
                        const ::sc_core::sc_time &period);
void destroyClockTick(ClockTick *tick);
void createClockTickProcess(ClockTick *tick);
void scheduleClockTick(ClockTick *tick, const ::sc_core::sc_time &delay);
void descheduleClockTick(ClockTick *tick);
bool isClockTickScheduled(ClockTick *tick);

} // namespace sc_gem5

#endif // __SYSTEMC_CORE_CLOCK_TICK_OPS_HH__
