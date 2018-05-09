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

#include "base/logging.hh"
#include "systemc/ext/core/sc_spawn.hh"

namespace sc_core
{

sc_spawn_options::sc_spawn_options()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}


void
sc_spawn_options::spawn_method()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_spawn_options::dont_initialize()
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_spawn_options::set_stack_size(int)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}


void
sc_spawn_options::set_sensitivity(const sc_event *)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_spawn_options::set_sensitivity(sc_port_base *)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_spawn_options::set_sensitivity(sc_export_base *)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_spawn_options::set_sensitivity(sc_interface *)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_spawn_options::set_sensitivity(sc_event_finder *)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}


void
sc_spawn_options::reset_signal_is(const sc_in<bool> &, bool)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_spawn_options::reset_signal_is(const sc_inout<bool> &, bool)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_spawn_options::reset_signal_is(const sc_out<bool> &, bool)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_spawn_options::reset_signal_is(const sc_signal_in_if<bool> &, bool)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}


void
sc_spawn_options::async_reset_signal_is(const sc_in<bool> &, bool)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_spawn_options::async_reset_signal_is(const sc_inout<bool> &, bool)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_spawn_options::async_reset_signal_is(const sc_out<bool> &, bool)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}

void
sc_spawn_options::async_reset_signal_is(const sc_signal_in_if<bool> &, bool)
{
    warn("%s not implemented.\n", __PRETTY_FUNCTION__);
}


void
sc_spawn_warn_unimpl(const char *func)
{
    warn("%s not implemented.\n", func);
}

} // namespace sc_core

namespace sc_unnamed
{

ImplementationDefined _1;
ImplementationDefined _2;
ImplementationDefined _3;
ImplementationDefined _4;
ImplementationDefined _5;
ImplementationDefined _6;
ImplementationDefined _7;
ImplementationDefined _8;
ImplementationDefined _9;

} // namespace sc_unnamed
