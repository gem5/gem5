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

#include <sstream>

#include "systemc/core/scheduler.hh"
#include "systemc/ext/channel/messages.hh"
#include "systemc/ext/channel/sc_signal.hh"
#include "systemc/ext/core/sc_main.hh"

namespace sc_gem5
{

ScSignalBase::ScSignalBase(const char *name) :
    sc_core::sc_prim_channel(name), _changeStamp(~0ULL),
    _gem5WriterPort(nullptr)
{}

ScSignalBase::~ScSignalBase() {}

const sc_core::sc_event &
ScSignalBase::defaultEvent() const
{
    return valueChangedEvent();
}

const sc_core::sc_event &
ScSignalBase::valueChangedEvent() const
{
    return _valueChangedEvent;
}

bool
ScSignalBase::event() const
{
    return _changeStamp == getChangeStamp();
}

void
ScSignalBase::_signalChange()
{
    _changeStamp = getChangeStamp();
    _valueChangedEvent.notify(sc_core::SC_ZERO_TIME);
}

void
ScSignalBaseBinary::_signalPosedge()
{
    _posStamp = getChangeStamp();
    _posedgeEvent.notify(sc_core::SC_ZERO_TIME);
}

void
ScSignalBaseBinary::_signalNegedge()
{
    _negStamp = getChangeStamp();
    _negedgeEvent.notify(sc_core::SC_ZERO_TIME);
}

namespace
{

void
reportSignalError(ScSignalBase *sig, sc_core::sc_object *first,
        sc_core::sc_object *second, bool delta_conflict=false)
{
    std::ostringstream ss;
    ss << "\n signal " << "`" << sig->name() << "' (" << sig->kind() << ")";
    ss << "\n first driver `" << first->name() << "' (" <<
        first->kind() << ")";
    ss << "\n second driver `" << second->name() << "' (" <<
        second->kind() << ")";
    if (delta_conflict) {
        ss << "\n conflicting write in delta cycle " <<
            sc_core::sc_delta_count();
    }
    SC_REPORT_ERROR(sc_core::SC_ID_MORE_THAN_ONE_SIGNAL_DRIVER_,
            ss.str().c_str());
}

} // anonymous namespace

WriteChecker<sc_core::SC_ONE_WRITER>::WriteChecker(ScSignalBase *_sig) :
    sig(_sig), firstPort(nullptr), proc(nullptr), writeStamp(~0ULL)
{}

void
WriteChecker<sc_core::SC_ONE_WRITER>::checkPort(sc_core::sc_port_base &port,
        std::string iface_type_name, std::string out_name)
{
    if (iface_type_name == out_name) {
        if (firstPort)
            reportSignalError(sig, firstPort, &port);
        firstPort = &port;
    }
}

void
WriteChecker<sc_core::SC_ONE_WRITER>::checkWriter()
{
    Process *p = scheduler.current();
    if (!p)
        return;
    uint64_t stamp = getChangeStamp();
    if (proc && proc != p)
        reportSignalError(sig, proc, p);
    proc = p;
    writeStamp = stamp;
}

WriteChecker<sc_core::SC_MANY_WRITERS>::WriteChecker(ScSignalBase *_sig) :
    sig(_sig), proc(nullptr), writeStamp(~0ULL)
{}

void
WriteChecker<sc_core::SC_MANY_WRITERS>::checkPort(sc_core::sc_port_base &port,
        std::string iface_type_name, std::string out_name)
{
    return;
}

void
WriteChecker<sc_core::SC_MANY_WRITERS>::checkWriter()
{
    Process *p = scheduler.current();
    if (!p)
        return;
    uint64_t stamp = getChangeStamp();
    if (writeStamp == stamp && proc && proc != p)
        reportSignalError(sig, proc, p, writeStamp == stamp);
    proc = p;
    writeStamp = stamp;
}

ScSignalBaseBinary::ScSignalBaseBinary(const char *_name) :
    ScSignalBase(_name), _posStamp(~0ULL), _negStamp(~0ULL)
{}

const sc_core::sc_event &
ScSignalBaseBinary::posedgeEvent() const
{
    return _posedgeEvent;
}

const sc_core::sc_event &
ScSignalBaseBinary::negedgeEvent() const
{
    return _negedgeEvent;
}

bool
ScSignalBaseBinary::posedge() const
{
    return _posStamp == getChangeStamp();
}

bool
ScSignalBaseBinary::negedge() const
{
    return _negStamp == getChangeStamp();
}

void
ScSignalBaseBinary::_signalReset(sc_gem5::Reset *r)
{
    r->update();
}

void
ScSignalBaseBinary::_signalReset()
{
    for (auto r: _resets)
        _signalReset(r);
}

} // namespace sc_gem5
