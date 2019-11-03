/*
 * Copyright 2018 Google, Inc.
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
 * Authors: Gabe Black
 */

#include "base/types.hh"
#include "sim/core.hh"
#include "sim/eventq.hh"
#include "systemc/core/kernel.hh"
#include "systemc/core/sc_main_fiber.hh"
#include "systemc/core/scheduler.hh"
#include "systemc/ext/core/messages.hh"
#include "systemc/ext/core/sc_main.hh"
#include "systemc/ext/utils/sc_report_handler.hh"

namespace sc_core
{

namespace
{

sc_stop_mode _stop_mode = SC_STOP_FINISH_DELTA;

} // anonymous namespace

int
sc_argc()
{
    return ::sc_gem5::scMainFiber.argc();
}

const char *const *
sc_argv()
{
    return ::sc_gem5::scMainFiber.argv();
}

void
sc_start()
{
    Tick now = ::sc_gem5::scheduler.getCurTick();
    sc_start(sc_time::from_value(MaxTick - now), SC_EXIT_ON_STARVATION);
}

void
sc_pause()
{
    if (::sc_gem5::Kernel::status() == SC_RUNNING)
        ::sc_gem5::scheduler.schedulePause();
}

void
sc_start(const sc_time &time, sc_starvation_policy p)
{
    if (time.value() == 0) {
        ::sc_gem5::scheduler.oneCycle();
    } else {
        Tick now = ::sc_gem5::scheduler.getCurTick();
        if (MaxTick - now < time.value())
            SC_REPORT_ERROR(SC_ID_SIMULATION_TIME_OVERFLOW_, "");
        ::sc_gem5::scheduler.start(now + time.value(), p == SC_RUN_TO_TIME);
    }
}

void
sc_set_stop_mode(sc_stop_mode mode)
{
    if (sc_is_running()) {
        SC_REPORT_ERROR(SC_ID_STOP_MODE_AFTER_START_, "");
        return;
    }
    _stop_mode = mode;
}

sc_stop_mode
sc_get_stop_mode()
{
    return _stop_mode;
}

void
sc_stop()
{
    static bool stop_called = false;
    if (stop_called) {
        static bool stop_warned = false;
        if (!stop_warned)
            SC_REPORT_WARNING(SC_ID_SIMULATION_STOP_CALLED_TWICE_, "");
        stop_warned = true;
        return;
    }
    stop_called = true;

    if (::sc_gem5::Kernel::status() == SC_STOPPED)
        return;

    if ((sc_get_status() & SC_RUNNING)) {
        bool finish_delta = (_stop_mode == SC_STOP_FINISH_DELTA);
        ::sc_gem5::scheduler.scheduleStop(finish_delta);
    } else {
        ::sc_gem5::Kernel::stop();
    }
}

const sc_time &
sc_time_stamp()
{
    static sc_time tstamp(1.0, SC_SEC);
    tstamp = sc_time::from_value(::sc_gem5::scheduler.getCurTick());
    return tstamp;
}

sc_dt::uint64
sc_delta_count()
{
    return sc_gem5::scheduler.numCycles();
}

bool
sc_is_running()
{
    return sc_get_status() & (SC_RUNNING | SC_PAUSED);
}

bool
sc_pending_activity_at_current_time()
{
    return ::sc_gem5::scheduler.pendingCurr();
}

bool
sc_pending_activity_at_future_time()
{
    return ::sc_gem5::scheduler.pendingFuture();
}

bool
sc_pending_activity()
{
    return sc_pending_activity_at_current_time() ||
           sc_pending_activity_at_future_time();
}

sc_time
sc_time_to_pending_activity()
{
    return sc_time::from_value(::sc_gem5::scheduler.timeToPending());
}

sc_status
sc_get_status()
{
    return ::sc_gem5::kernel ? ::sc_gem5::kernel->status() : SC_ELABORATION;
}

std::ostream &
operator << (std::ostream &os, sc_status s)
{
    switch (s) {
      case SC_ELABORATION:
        os << "SC_ELABORATION";
        break;
      case SC_BEFORE_END_OF_ELABORATION:
        os << "SC_BEFORE_END_OF_ELABORATION";
        break;
      case SC_END_OF_ELABORATION:
        os << "SC_END_OF_ELABORATION";
        break;
      case SC_START_OF_SIMULATION:
        os << "SC_START_OF_SIMULATION";
        break;
      case SC_RUNNING:
        os << "SC_RUNNING";
        break;
      case SC_PAUSED:
        os << "SC_PAUSED";
        break;
      case SC_STOPPED:
        os << "SC_STOPPED";
        break;
      case SC_END_OF_SIMULATION:
        os << "SC_END_OF_SIMULATION";
        break;

        // Nonstandard
      case SC_END_OF_INITIALIZATION:
        os << "SC_END_OF_INITIALIZATION";
        break;
      case SC_END_OF_UPDATE:
        os << "SC_END_OF_UPDATE";
        break;
      case SC_BEFORE_TIMESTEP:
        os << "SC_BEFORE_TIMESTEP";
        break;

      default:
        if (s & SC_STATUS_ANY) {
            const char *prefix = "(";
            for (sc_status m = (sc_status)0x1;
                    m < SC_STATUS_ANY; m = (sc_status)(m << 1)) {
                if (m & s) {
                    os << prefix;
                    prefix = "|";
                    os << m;
                }
            }
            os << ")";
        } else {
            ccprintf(os, "%#x", s);
        }
    }

    return os;
}

} // namespace sc_core
