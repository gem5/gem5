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

#include <cstring>

#include "base/fiber.hh"
#include "base/logging.hh"
#include "base/types.hh"
#include "python/pybind11/pybind.hh"
#include "sim/eventq.hh"
#include "sim/init.hh"
#include "systemc/ext/core/sc_main.hh"
#include "systemc/ext/utils/sc_report_handler.hh"

// A default version of this function in case one isn't otherwise defined.
// This ensures everything will link properly whether or not the user defined
// a custom sc_main function. If they didn't but still try to call it, throw
// an error and die.
[[gnu::weak]] int
sc_main(int argc, char *argv[])
{
    // If python attempts to call sc_main but no sc_main was defined...
    fatal("sc_main called but not defined.\n");
}

namespace sc_core
{

namespace
{

bool scMainCalled = false;

int _argc = 0;
char **_argv = NULL;

class ScMainFiber : public Fiber
{
    void
    main()
    {
        ::sc_main(_argc, _argv);
    }
};

ScMainFiber scMainFiber;

// This wrapper adapts the python version of sc_main to the c++ version.
void
sc_main(pybind11::args args)
{
    panic_if(scMainCalled, "sc_main called more than once.");

    _argc = args.size();
    _argv = new char *[_argc];

    // Initialize all the _argvs to NULL so we can delete [] them
    // unconditionally.
    for (int idx = 0; idx < _argc; idx++)
        _argv[idx] = NULL;

    // Attempt to convert all the arguments to strings. If that fails, clean
    // up after ourselves. Also don't count this as a call to sc_main since
    // we never got to the c++ version of that function.
    try {
        for (int idx = 0; idx < _argc; idx++) {
            std::string arg = args[idx].cast<std::string>();
            _argv[idx] = new char[arg.length() + 1];
            strcpy(_argv[idx], arg.c_str());
        }
    } catch (...) {
        // If that didn't work for some reason (probably a conversion error)
        // blow away _argv and _argc and pass on the exception.
        for (int idx = 0; idx < _argc; idx++)
            delete [] _argv[idx];
        delete [] _argv;
        _argc = 0;
        throw;
    }

    // At this point we're going to call the c++ sc_main, so we can't try
    // again later.
    scMainCalled = true;

    scMainFiber.run();
}

// Make our sc_main wrapper available in the internal _m5 python module under
// the systemc submodule.
void
systemc_pybind(pybind11::module &m_internal)
{
    pybind11::module m = m_internal.def_submodule("systemc");
    m.def("sc_main", &sc_main);
}
EmbeddedPyBind embed_("systemc", &systemc_pybind);

sc_stop_mode _stop_mode = SC_STOP_FINISH_DELTA;
sc_status _status = SC_ELABORATION;

Tick _max_tick = MaxTick;
sc_starvation_policy _starvation = SC_EXIT_ON_STARVATION;

uint64_t _deltaCycles = 0;

} // anonymous namespace

int
sc_argc()
{
    return _argc;
}

const char *const *
sc_argv()
{
    return _argv;
}

void
sc_start()
{
    _max_tick = MaxTick;
    _starvation = SC_EXIT_ON_STARVATION;

    // Switch back gem5.
    Fiber::primaryFiber()->run();
}

void
sc_pause()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_start(const sc_time &time, sc_starvation_policy p)
{
    Tick now = curEventQueue() ? curEventQueue()->getCurTick() : 0;
    _max_tick = now + time.value();
    _starvation = p;

    // Switch back to gem5.
    Fiber::primaryFiber()->run();
}

void
sc_set_stop_mode(sc_stop_mode mode)
{
    if (sc_is_running()) {
        SC_REPORT_ERROR("attempt to set sc_stop mode "
                        "after start will be ignored", "");
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
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

const sc_time &
sc_time_stamp()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return *(sc_time *)nullptr;
}

sc_dt::uint64
sc_delta_count()
{
    return _deltaCycles;
}

bool
sc_is_running()
{
    return _status & (SC_RUNNING | SC_PAUSED);
}

bool
sc_pending_activity_at_current_time()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return false;
}

bool
sc_pending_activity_at_future_time()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return false;
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
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
    return sc_time();
}

sc_status
sc_get_status()
{
    return _status;
}

} // namespace sc_core
