/*
 * Copyright (c) 2016, Dresden University of Technology (TU Dresden)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Christian Menard
 */

#include <iostream>
#include <systemc>

#include <sim/core.hh>
#include <sim/simulate.hh>

#include "report_handler.hh"

using namespace sc_core;

void
reportHandler(const sc_report &report, const sc_actions &actions)
{
    uint64_t systemc_time = report.get_time().value();
    uint64_t gem5_time = curTick();

    if (actions & SC_DO_NOTHING)
        return;

    if (actions & SC_DISPLAY || actions & SC_LOG)
    {
        std::ostream& stream = actions & SC_DISPLAY ? std::cout : std::cerr;

        stream << report.get_time();

        if (gem5_time < systemc_time) {
            stream << " (<) ";
        } else if (gem5_time > systemc_time) {
            stream << " (!) ";
        } else {
            stream << " (=) ";
        }

        stream << ": " << report.get_msg_type()
               << ' ' << report.get_msg() << '\n';
    }

    if (actions & SC_THROW) {
        std::cerr << "warning: the report handler ignored a SC_THROW action\n";
    } else if (actions & SC_INTERRUPT) {
        std::cerr << "warning: the report handler ignored a SC_INTERRUPT"
                  << "action\n";
    } else if (actions & SC_CACHE_REPORT) {
        std::cerr << "warning: the report handler ignored a SC_CACHE_REPORT"
                  << "action\n";
    }

    if (actions & SC_STOP)
        sc_stop();

    if (actions & SC_ABORT)
        abort();
}
