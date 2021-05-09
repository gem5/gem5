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
 */

#include "systemc/core/sc_main_fiber.hh"

#include <cstring>
#include <string>

#include "base/compiler.hh"
#include "systemc/core/kernel.hh"
#include "systemc/core/scheduler.hh"
#include "systemc/ext/core/messages.hh"
#include "systemc/ext/core/sc_main.hh"
#include "systemc/ext/utils/sc_report_handler.hh"
#include "systemc/utils/report.hh"

// A weak symbol to detect if sc_main has been defined, and if so where it is.
GEM5_WEAK int sc_main(int argc, char *argv[]);

namespace sc_gem5
{

void
ScMainFiber::main()
{
    using namespace gem5;

    _called = true;

    if (::sc_main) {
        try {
            _resultInt = ::sc_main(_argc, _argv);
            if (_resultInt)
                _resultStr = "sc_main returned non-zero";
            else
                _resultStr = "sc_main finished";
            // Make sure no systemc events/notifications are scheduled
            // after sc_main returns.
        } catch (const ::sc_core::sc_report &r) {
            // There was an exception nobody caught.
            _resultStr = "uncaught sc_report";
            reportHandlerProc(
                    r, ::sc_core::sc_report_handler::get_catch_actions());
        } catch (...) {
            // There was some other type of exception we need to wrap.
            _resultStr = "uncaught exception";
            reportHandlerProc(reportifyException(),
                    ::sc_core::sc_report_handler::get_catch_actions());
        }
        scheduler.clear();
    } else {
        // If python tries to call sc_main but no sc_main was defined...
        fatal("sc_main called but not defined.\n");
    }
}

ScMainFiber scMainFiber;

} // namespace sc_gem5
