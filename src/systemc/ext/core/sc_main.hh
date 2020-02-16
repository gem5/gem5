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

#ifndef __SYSTEMC_EXT_CORE_SC_MAIN_HH__
#define __SYSTEMC_EXT_CORE_SC_MAIN_HH__

#include <iostream>

#include "../dt/int/sc_nbdefs.hh"
#include "sc_time.hh"

extern "C" int sc_main(int argc, char *argv[]);

namespace sc_core
{
    extern "C" int sc_argc();

    // The standard version of this function doesn't have these "const"
    // qualifiers, but the canonical SystemC implementation does.
    extern "C" const char *const *sc_argv();

    enum sc_starvation_policy
    {
        SC_RUN_TO_TIME,
        SC_EXIT_ON_STARVATION
    };

    void sc_start();
    void sc_start(const sc_time &, sc_starvation_policy p=SC_RUN_TO_TIME);
    static inline void
    sc_start(double d, sc_time_unit t, sc_starvation_policy p=SC_RUN_TO_TIME)
    {
        sc_start(sc_time(d, t), p);
    }

    void sc_pause();

    enum sc_stop_mode
    {
        SC_STOP_FINISH_DELTA,
        SC_STOP_IMMEDIATE,
    };

    void sc_set_stop_mode(sc_stop_mode mode);
    sc_stop_mode sc_get_stop_mode();

    void sc_stop();

    const sc_time &sc_time_stamp();
    sc_dt::uint64 sc_delta_count();
    bool sc_is_running();
    bool sc_pending_activity_at_current_time();
    bool sc_pending_activity_at_future_time();
    bool sc_pending_activity();
    sc_time sc_time_to_pending_activity();

    enum sc_status
    {
        SC_ELABORATION = 0x1,
        SC_BEFORE_END_OF_ELABORATION = 0x02,
        SC_END_OF_ELABORATION = 0x04,
        SC_START_OF_SIMULATION = 0x08,
        SC_RUNNING = 0x10,
        SC_PAUSED = 0x20,
        SC_STOPPED = 0x40,
        SC_END_OF_SIMULATION = 0x80,

        // Nonstandard
        SC_END_OF_INITIALIZATION = 0x100,
        SC_END_OF_UPDATE = 0x400,
        SC_BEFORE_TIMESTEP = 0x800,
        SC_STATUS_ANY = 0xdff
    };

    sc_status sc_get_status();

    std::ostream &operator << (std::ostream &os, sc_status s);
} // namespace sc_core

#endif  //__SYSTEMC_EXT_CORE_SC_MAIN_HH__
