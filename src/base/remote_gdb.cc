/*
 * Copyright (c) 2018 ARM Limited
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright 2015, 2021 LabWare
 * Copyright 2014 Google, Inc.
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
 */

/*
 * Copyright (c) 1990, 1993 The Regents of the University of California
 * All rights reserved
 *
 * This software was developed by the Computer Systems Engineering group
 * at Lawrence Berkeley Laboratory under DARPA contract BG 91-66 and
 * contributed to Berkeley.
 *
 * All advertising materials mentioning features or use of this software
 * must display the following acknowledgement:
 *      This product includes software developed by the University of
 *      California, Lawrence Berkeley Laboratories.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed by the University of
 *      California, Berkeley and its contributors.
 * 4. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *      @(#)kgdb_stub.c 8.4 (Berkeley) 1/12/94
 */

/*-
 * Copyright (c) 2001 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Jason R. Thorpe.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed by the NetBSD
 *      Foundation, Inc. and its contributors.
 * 4. Neither the name of The NetBSD Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * $NetBSD: kgdb_stub.c,v 1.8 2001/07/07 22:58:00 wdk Exp $
 *
 * Taken from NetBSD
 *
 * "Stub" to allow remote cpu to debug over a serial line using gdb.
 */

#include "base/remote_gdb.hh"

#include <sys/signal.h>
#include <unistd.h>

#include <cassert>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <iterator>
#include <sstream>
#include <string>
#include <utility>

#include "base/cprintf.hh"
#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/socket.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "debug/GDBAll.hh"
#include "mem/port.hh"
#include "mem/port_proxy.hh"
#include "mem/se_translating_port_proxy.hh"
#include "mem/translating_port_proxy.hh"
#include "sim/full_system.hh"
#include "sim/process.hh"
#include "sim/system.hh"

namespace gem5
{

static const char GDBStart = '$';
static const char GDBEnd = '#';
static const char GDBGoodP = '+';
static const char GDBBadP = '-';

class HardBreakpoint : public PCEvent
{
  private:
    BaseRemoteGDB *gdb;

  public:
    int refcount;

  public:
    HardBreakpoint(BaseRemoteGDB *_gdb, PCEventScope *s, Addr pc)
        : PCEvent(s, "HardBreakpoint Event", pc),
          gdb(_gdb), refcount(0)
    {
        DPRINTF(GDBMisc, "creating hardware breakpoint at %#x\n", evpc);
    }

    const std::string name() const override { return gdb->name() + ".hwbkpt"; }

    void
    process(ThreadContext *tc) override
    {
        DPRINTF(GDBMisc, "handling hardware breakpoint at %#x\n", pc());

        if (tc == gdb->tc)
            gdb->trap(tc->contextId(), SIGTRAP);
    }
};

namespace {

// Exception to throw when the connection to the client is broken.
struct BadClient
{
    const char *warning;
    BadClient(const char *_warning=NULL) : warning(_warning)
    {}
};

// Exception to throw when an error needs to be reported to the client.
struct CmdError
{
    std::string error;
    CmdError(std::string _error) : error(_error)
    {}
};

// Exception to throw when something isn't supported.
class Unsupported {};

// Convert a hex digit into an integer.
// This returns -1 if the argument passed is no valid hex digit.
int
digit2i(char c)
{
    if (c >= '0' && c <= '9')
        return (c - '0');
    else if (c >= 'a' && c <= 'f')
        return (c - 'a' + 10);
    else if (c >= 'A' && c <= 'F')
        return (c - 'A' + 10);
    else
        return -1;
}

// Convert the low 4 bits of an integer into an hex digit.
char
i2digit(int n)
{
    return "0123456789abcdef"[n & 0x0f];
}

// Convert a byte array into an hex string.
void
mem2hex(char *vdst, const char *vsrc, int len)
{
    char *dst = vdst;
    const char *src = vsrc;

    while (len--) {
        *dst++ = i2digit(*src >> 4);
        *dst++ = i2digit(*src++);
    }
    *dst = '\0';
}

// Convert an hex string into a byte array.
// This returns a pointer to the character following the last valid
// hex digit. If the string ends in the middle of a byte, NULL is
// returned.
const char *
hex2mem(char *vdst, const char *src, int max_len)
{
    char *dst = vdst;
    int msb, lsb;

    while (*src && max_len--) {
        msb = digit2i(*src++);
        if (msb < 0)
            return (src - 1);
        lsb = digit2i(*src++);
        if (lsb < 0)
            return (NULL);
        *dst++ = (msb << 4) | lsb;
    }
    return src;
}

// Convert an hex string into an integer.
// This returns a pointer to the character following the last valid
// hex digit.
Addr
hex2i(const char **srcp)
{
    const char *src = *srcp;
    Addr r = 0;
    int nibble;

    while ((nibble = digit2i(*src)) >= 0) {
        r *= 16;
        r += nibble;
        src++;
    }
    *srcp = src;
    return r;
}

bool
parseThreadId(const char **srcp, bool &all, bool &any, ContextID &tid)
{
    all = any = false;
    tid = 0;
    const char *src = *srcp;
    if (*src == '-') {
        // This could be the start of -1, which means all threads.
        src++;
        if (*src++ != '1')
            return false;
        *srcp += 2;
        all = true;
        return true;
    }
    tid = hex2i(srcp);
    // If *srcp still points to src, no characters were consumed and no thread
    // id was found. Without this check, we can't tell the difference between
    // zero and a parsing error.
    if (*srcp == src)
        return false;

    if (tid == 0)
        any = true;

    tid--;

    return true;
}

int
encodeThreadId(ContextID id)
{
    // Thread ID 0 is reserved and means "pick any thread".
    return id + 1;
}

enum GdbBreakpointType
{
    GdbSoftBp = '0',
    GdbHardBp = '1',
    GdbWriteWp = '2',
    GdbReadWp = '3',
    GdbAccWp = '4',
};

const char *
breakType(char c)
{
    switch(c) {
      case GdbSoftBp: return "software breakpoint";
      case GdbHardBp: return "hardware breakpoint";
      case GdbWriteWp: return "write watchpoint";
      case GdbReadWp: return "read watchpoint";
      case GdbAccWp: return "access watchpoint";
      default: return "unknown breakpoint/watchpoint";
    }
}

std::map<Addr, HardBreakpoint *> hardBreakMap;

}

BaseRemoteGDB::BaseRemoteGDB(System *_system, int _port) :
        incomingConnectionEvent(nullptr), incomingDataEvent(nullptr),
        _port(_port), fd(-1), sys(_system),
        connectEvent(this), disconnectEvent(this), trapEvent(this),
        singleStepEvent(*this)
{}

BaseRemoteGDB::~BaseRemoteGDB()
{
    delete incomingConnectionEvent;
    delete incomingDataEvent;
}

std::string
BaseRemoteGDB::name()
{
    return sys->name() + ".remote_gdb";
}

void
BaseRemoteGDB::listen()
{
    if (ListenSocket::allDisabled()) {
        warn_once("Sockets disabled, not accepting gdb connections");
        return;
    }

    while (!listener.listen(_port, true)) {
        DPRINTF(GDBMisc, "Can't bind port %d\n", _port);
        _port++;
    }

    incomingConnectionEvent =
            new IncomingConnectionEvent(this, listener.getfd(), POLLIN);
    pollQueue.schedule(incomingConnectionEvent);

    ccprintf(std::cerr, "%d: %s: listening for remote gdb on port %d\n",
             curTick(), name(), _port);
}

void
BaseRemoteGDB::connect()
{
    panic_if(!listener.islistening(),
             "Can't accept GDB connections without any threads!");

    pollQueue.remove(incomingConnectionEvent);

    int sfd = listener.accept(true);

    if (sfd != -1) {
        if (isAttached())
            close(sfd);
        else
            attach(sfd);
    }
}

int
BaseRemoteGDB::port() const
{
    panic_if(!listener.islistening(),
             "Remote GDB port is unknown until listen() has been called.\n");
    return _port;
}

void
BaseRemoteGDB::attach(int f)
{
    fd = f;

    attached = true;
    DPRINTFN("remote gdb attached\n");

    processCommands();

    if (isAttached()) {
        // At this point an initial communication with GDB is handled
        // and we're ready to continue. Here we arrange IncomingDataEvent
        // to get notified when GDB breaks in.
        //
        // However, GDB can decide to disconnect during that initial
        // communication. In that case, we cannot arrange data event because
        // the socket is already closed (not that it makes any sense, anyways).
        //
        // Hence the check above.
        incomingDataEvent = new IncomingDataEvent(this, fd, POLLIN);
        pollQueue.schedule(incomingDataEvent);
    }
}

void
BaseRemoteGDB::detach()
{
    attached = false;
    clearSingleStep();
    close(fd);
    fd = -1;

    if (incomingDataEvent) {
        // incomingDataEvent gets scheduled in attach() after
        // initial communication with GDB is handled and GDB tells
        // gem5 to continue.
        //
        // GDB can disconnect before that in which case `incomingDataEvent`
        // is NULL.
        //
        // Hence the check above.

        pollQueue.remove(incomingDataEvent);
        incomingDataEvent = nullptr;
    }
    pollQueue.schedule(incomingConnectionEvent);
    DPRINTFN("remote gdb detached\n");
}

void
BaseRemoteGDB::addThreadContext(ThreadContext *_tc)
{
    [[maybe_unused]] auto it_success = threads.insert({_tc->contextId(), _tc});
    assert(it_success.second);
    // If no ThreadContext is current selected, select this one.
    if (!tc)
        assert(selectThreadContext(_tc->contextId()));

    // Now that we have a thread, we can start listening.
    if (!listener.islistening())
        listen();
}

void
BaseRemoteGDB::replaceThreadContext(ThreadContext *_tc)
{
    auto it = threads.find(_tc->contextId());
    panic_if(it == threads.end(), "No context with ID %d found.",
            _tc->contextId());
    it->second = _tc;
}

bool
BaseRemoteGDB::selectThreadContext(ContextID id)
{
    auto it = threads.find(id);
    if (it == threads.end())
        return false;

    tc = it->second;
    // Update the register cache for the new thread context, if there is one.
    if (regCachePtr)
        regCachePtr->getRegs(tc);
    return true;
}

// This function does all command processing for interfacing to a
// remote gdb.  Note that the error codes are ignored by gdb at
// present, but might eventually become meaningful. (XXX) It might
// makes sense to use POSIX errno values, because that is what the
// gdb/remote.c functions want to return.
void
BaseRemoteGDB::trap(ContextID id, int signum)
{
    if (!attached)
        return;

    if (tc->contextId() != id) {
        if (!selectThreadContext(id))
            return;
    }

    DPRINTF(GDBMisc, "trap: PC=%s\n", tc->pcState());

    clearSingleStep();

    if (threadSwitching) {
        threadSwitching = false;
        // Tell GDB the thread switch has completed.
        send("OK");
    } else {
        // Tell remote host that an exception has occurred.
        send("S%02x", signum);
    }

    processCommands(signum);
}

void
BaseRemoteGDB::incomingConnection(int revent)
{
    if (connectEvent.scheduled()) {
        warn("GDB connect event has already been scheduled!");
        return;
    }

    if (revent & POLLIN) {
        scheduleInstCommitEvent(&connectEvent, 0);
    }
}

void
BaseRemoteGDB::incomingData(int revent)
{
    if (trapEvent.scheduled()) {
        warn("GDB trap event has already been scheduled!");
        return;
    }

    if (revent & POLLIN) {
        trapEvent.type(SIGILL);
        trapEvent.id(tc->contextId());
        scheduleInstCommitEvent(&trapEvent, 0);
    } else if (revent & POLLNVAL) {
        descheduleInstCommitEvent(&trapEvent);
        scheduleInstCommitEvent(&disconnectEvent, 0);
    }
}

uint8_t
BaseRemoteGDB::getbyte()
{
    uint8_t b;
    if (::read(fd, &b, sizeof(b)) == sizeof(b))
        return b;

    throw BadClient("Couldn't read data from debugger.");
}

void
BaseRemoteGDB::putbyte(uint8_t b)
{
    if (::write(fd, &b, sizeof(b)) == sizeof(b))
        return;

    throw BadClient("Couldn't write data to the debugger.");
}

// Receive a packet from gdb
void
BaseRemoteGDB::recv(std::vector<char>& bp)
{
    uint8_t c;
    int csum;
    bp.resize(0);

    do {
        csum = 0;
        // Find the beginning of a packet
        while ((c = getbyte()) != GDBStart);

        // Read until you find the end of the data in the packet, and keep
        // track of the check sum.
        while (true) {
            c = getbyte();
            if (c == GDBEnd)
                break;
            c &= 0x7f;
            csum += c;
            bp.push_back(c);
        }

        // Mask the check sum.
        csum &= 0xff;

        // Bring in the checksum. If the check sum matches, csum will be 0.
        csum -= digit2i(getbyte()) * 16;
        csum -= digit2i(getbyte());

        // If the check sum was correct
        if (csum == 0) {
            // Report that the packet was received correctly
            putbyte(GDBGoodP);
            // Sequence present?
            if (bp.size() > 2 && bp[2] == ':') {
                putbyte(bp[0]);
                putbyte(bp[1]);
                auto begin = std::begin(bp);
                bp.erase(begin, std::next(begin, 3));
            }
            break;
        }
        // Otherwise, report that there was a mistake.
        putbyte(GDBBadP);
    } while (1);
    // Sentinel.
    bp.push_back('\0');
    DPRINTF(GDBRecv, "recv:  %s\n", bp.data());
}

// Send a packet to gdb
void
BaseRemoteGDB::send(const char *bp)
{
    const char *p;
    uint8_t csum, c;

    DPRINTF(GDBSend, "send:  %s\n", bp);

    do {
        p = bp;
        // Start sending a packet
        putbyte(GDBStart);
        // Send the contents, and also keep a check sum.
        for (csum = 0; (c = *p); p++) {
            putbyte(c);
            csum += c;
        }
        // Send the ending character.
        putbyte(GDBEnd);
        // Send the checksum.
        putbyte(i2digit(csum >> 4));
        putbyte(i2digit(csum));
        // Try transmitting over and over again until the other end doesn't
        // send an error back.
        c = getbyte();
    } while ((c & 0x7f) == GDBBadP);
}

void
BaseRemoteGDB::processCommands(int signum)
{
    // Stick frame regs into our reg cache.
    regCachePtr = gdbRegs();
    regCachePtr->getRegs(tc);

    GdbCommand::Context cmd_ctx;
    cmd_ctx.type = signum;
    std::vector<char> data;

    for (;;) {
        try {
            recv(data);
            if (data.size() == 1)
                throw BadClient();
            cmd_ctx.cmdByte = data[0];
            cmd_ctx.data = data.data() + 1;
            // One for sentinel, one for cmdByte.
            cmd_ctx.len = data.size() - 2;

            auto cmd_it = commandMap.find(cmd_ctx.cmdByte);
            if (cmd_it == commandMap.end()) {
                DPRINTF(GDBMisc, "Unknown command: %c(%#x)\n",
                        cmd_ctx.cmdByte, cmd_ctx.cmdByte);
                throw Unsupported();
            }
            cmd_ctx.cmd = &(cmd_it->second);

            if (!(this->*(cmd_ctx.cmd->func))(cmd_ctx))
                break;

        } catch (BadClient &e) {
            if (e.warning)
                warn(e.warning);
            detach();
            break;
        } catch (Unsupported &e) {
            send("");
        } catch (CmdError &e) {
            send(e.error);
        } catch (std::exception &e) {
            panic("Unrecognized GDB exception: %s", e.what());
        } catch (...) {
            panic("Unrecognized GDB exception.");
        }
    }
}

bool
BaseRemoteGDB::readBlob(Addr vaddr, size_t size, char *data)
{
    (FullSystem ? TranslatingPortProxy(tc) : SETranslatingPortProxy(tc))
        .readBlob(vaddr, data, size);
    return true;
}

bool
BaseRemoteGDB::writeBlob(Addr vaddr, size_t size, const char *data)
{
    (FullSystem ? TranslatingPortProxy(tc) : SETranslatingPortProxy(tc))
        .writeBlob(vaddr, data, size);
    return true;
}

// Read bytes from kernel address space for debugger.
bool
BaseRemoteGDB::read(Addr vaddr, size_t size, char *data)
{
    DPRINTF(GDBRead, "read:  addr=%#x, size=%d", vaddr, size);

    bool res = readBlob(vaddr, size, data);

    if (!res)
        return false;

#if TRACING_ON
    if (debug::GDBRead) {
        if (debug::GDBExtra) {
            char buf[1024];
            mem2hex(buf, data, size);
            DPRINTFNR(": %s\n", buf);
        } else
            DPRINTFNR("\n");
    }
#endif

    return true;
}

// Write bytes to kernel address space for debugger.
bool
BaseRemoteGDB::write(Addr vaddr, size_t size, const char *data)
{
    if (debug::GDBWrite) {
        DPRINTFN("write: addr=%#x, size=%d", vaddr, size);
        if (debug::GDBExtra) {
            char buf[1024];
            mem2hex(buf, data, size);
            DPRINTFNR(": %s\n", buf);
        } else
            DPRINTFNR("\n");
    }
    return writeBlob(vaddr, size, data);
}

void
BaseRemoteGDB::singleStep()
{
    if (!singleStepEvent.scheduled())
        scheduleInstCommitEvent(&singleStepEvent, 1);
    trap(tc->contextId(), SIGTRAP);
}

void
BaseRemoteGDB::clearSingleStep()
{
    descheduleInstCommitEvent(&singleStepEvent);
}

void
BaseRemoteGDB::setSingleStep()
{
    if (!singleStepEvent.scheduled())
        scheduleInstCommitEvent(&singleStepEvent, 1);
}

void
BaseRemoteGDB::insertSoftBreak(Addr addr, size_t kind)
{
    if (!checkBpKind(kind))
        throw BadClient("Invalid breakpoint kind.\n");

    return insertHardBreak(addr, kind);
}

void
BaseRemoteGDB::removeSoftBreak(Addr addr, size_t kind)
{
    if (!checkBpKind(kind))
        throw BadClient("Invalid breakpoint kind.\n");

    return removeHardBreak(addr, kind);
}

void
BaseRemoteGDB::insertHardBreak(Addr addr, size_t kind)
{
    if (!checkBpKind(kind))
        throw BadClient("Invalid breakpoint kind.\n");

    DPRINTF(GDBMisc, "Inserting hardware breakpoint at %#x\n", addr);

    HardBreakpoint *&bkpt = hardBreakMap[addr];
    if (bkpt == 0)
        bkpt = new HardBreakpoint(this, sys, addr);

    bkpt->refcount++;
}

void
BaseRemoteGDB::removeHardBreak(Addr addr, size_t kind)
{
    if (!checkBpKind(kind))
        throw BadClient("Invalid breakpoint kind.\n");

    DPRINTF(GDBMisc, "Removing hardware breakpoint at %#x\n", addr);

    auto i = hardBreakMap.find(addr);
    if (i == hardBreakMap.end())
        throw CmdError("E0C");

    HardBreakpoint *hbp = (*i).second;
    if (--hbp->refcount == 0) {
        delete hbp;
        hardBreakMap.erase(i);
    }
}

void
BaseRemoteGDB::scheduleInstCommitEvent(Event *ev, int delta)
{
    if (delta == 0 && tc->status() != ThreadContext::Active) {
        // If delta is zero, we're just trying to wait for an instruction
        // boundary. If the CPU is not active, assume we're already at a
        // boundary without waiting for the CPU to eventually wake up.
        ev->process();
    } else {
        // Here "ticks" aren't simulator ticks which measure time, they're
        // instructions committed by the CPU.
        tc->scheduleInstCountEvent(ev, tc->getCurrentInstCount() + delta);
    }
}

void
BaseRemoteGDB::descheduleInstCommitEvent(Event *ev)
{
    if (ev->scheduled())
        tc->descheduleInstCountEvent(ev);
}

std::map<char, BaseRemoteGDB::GdbCommand> BaseRemoteGDB::commandMap = {
    // last signal
    { '?', { "KGDB_SIGNAL", &BaseRemoteGDB::cmdSignal } },
    // set baud (deprecated)
    { 'b', { "KGDB_SET_BAUD", &BaseRemoteGDB::cmdUnsupported } },
    // set breakpoint (deprecated)
    { 'B', { "KGDB_SET_BREAK", &BaseRemoteGDB::cmdUnsupported } },
    // resume
    { 'c', { "KGDB_CONT", &BaseRemoteGDB::cmdCont } },
    // continue with signal
    { 'C', { "KGDB_ASYNC_CONT", &BaseRemoteGDB::cmdAsyncCont } },
    // toggle debug flags (deprecated)
    { 'd', { "KGDB_DEBUG", &BaseRemoteGDB::cmdUnsupported } },
    // detach remote gdb
    { 'D', { "KGDB_DETACH", &BaseRemoteGDB::cmdDetach } },
    // read general registers
    { 'g', { "KGDB_REG_R", &BaseRemoteGDB::cmdRegR } },
    // write general registers
    { 'G', { "KGDB_REG_W", &BaseRemoteGDB::cmdRegW } },
    // set thread
    { 'H', { "KGDB_SET_THREAD", &BaseRemoteGDB::cmdSetThread } },
    // step a single cycle
    { 'i', { "KGDB_CYCLE_STEP", &BaseRemoteGDB::cmdUnsupported } },
    // signal then cycle step
    { 'I', { "KGDB_SIG_CYCLE_STEP", &BaseRemoteGDB::cmdUnsupported } },
    // kill program
    { 'k', { "KGDB_KILL", &BaseRemoteGDB::cmdDetach } },
    // read memory
    { 'm', { "KGDB_MEM_R", &BaseRemoteGDB::cmdMemR } },
    // write memory
    { 'M', { "KGDB_MEM_W", &BaseRemoteGDB::cmdMemW } },
    // read register
    { 'p', { "KGDB_READ_REG", &BaseRemoteGDB::cmdUnsupported } },
    // write register
    { 'P', { "KGDB_SET_REG", &BaseRemoteGDB::cmdUnsupported } },
    // query variable
    { 'q', { "KGDB_QUERY_VAR", &BaseRemoteGDB::cmdQueryVar } },
    // set variable
    { 'Q', { "KGDB_SET_VAR", &BaseRemoteGDB::cmdUnsupported } },
    // reset system (deprecated)
    { 'r', { "KGDB_RESET", &BaseRemoteGDB::cmdUnsupported } },
    // step
    { 's', { "KGDB_STEP", &BaseRemoteGDB::cmdStep } },
    // signal and step
    { 'S', { "KGDB_ASYNC_STEP", &BaseRemoteGDB::cmdAsyncStep } },
    // find out if the thread is alive
    { 'T', { "KGDB_THREAD_ALIVE", &BaseRemoteGDB::cmdUnsupported } },
    // target exited
    { 'W', { "KGDB_TARGET_EXIT", &BaseRemoteGDB::cmdUnsupported } },
    // write memory
    { 'X', { "KGDB_BINARY_DLOAD", &BaseRemoteGDB::cmdUnsupported } },
    // remove breakpoint or watchpoint
    { 'z', { "KGDB_CLR_HW_BKPT", &BaseRemoteGDB::cmdClrHwBkpt } },
    // insert breakpoint or watchpoint
    { 'Z', { "KGDB_SET_HW_BKPT", &BaseRemoteGDB::cmdSetHwBkpt } },
    // non-standard RSP extension: dump page table
    { '.', { "GET_PAGE_TABLE", &BaseRemoteGDB::cmdDumpPageTable } },
};

bool
BaseRemoteGDB::checkBpKind(size_t kind)
{
    return true;
}

bool
BaseRemoteGDB::cmdUnsupported(GdbCommand::Context &ctx)
{
    DPRINTF(GDBMisc, "Unsupported command: %s\n", ctx.cmd->name);
    DDUMP(GDBMisc, ctx.data, ctx.len);
    throw Unsupported();
}


bool
BaseRemoteGDB::cmdSignal(GdbCommand::Context &ctx)
{
    send("S%02x", ctx.type);
    return true;
}

bool
BaseRemoteGDB::cmdCont(GdbCommand::Context &ctx)
{
    const char *p = ctx.data;
    if (ctx.len) {
        Addr new_pc = hex2i(&p);
        tc->pcState(new_pc);
    }
    clearSingleStep();
    return false;
}

bool
BaseRemoteGDB::cmdAsyncCont(GdbCommand::Context &ctx)
{
    const char *p = ctx.data;
    hex2i(&p);
    if (*p++ == ';') {
        Addr new_pc = hex2i(&p);
        tc->pcState(new_pc);
    }
    clearSingleStep();
    return false;
}

bool
BaseRemoteGDB::cmdDetach(GdbCommand::Context &ctx)
{
    detach();
    return false;
}

bool
BaseRemoteGDB::cmdRegR(GdbCommand::Context &ctx)
{
    char buf[2 * regCachePtr->size() + 1];
    buf[2 * regCachePtr->size()] = '\0';
    mem2hex(buf, regCachePtr->data(), regCachePtr->size());
    send(buf);
    return true;
}

bool
BaseRemoteGDB::cmdRegW(GdbCommand::Context &ctx)
{
    const char *p = ctx.data;
    p = hex2mem(regCachePtr->data(), p, regCachePtr->size());
    if (p == NULL || *p != '\0')
        throw CmdError("E01");

    regCachePtr->setRegs(tc);
    send("OK");

    return true;
}

bool
BaseRemoteGDB::cmdSetThread(GdbCommand::Context &ctx)
{
    const char *p = ctx.data;
    char subcommand = *p++;
    int tid = 0;
    bool all, any;
    if (!parseThreadId(&p, all, any, tid))
        throw CmdError("E01");

    if (subcommand == 'c') {
        // We can only single step or continue all threads at once, since we
        // stop time itself and not individual threads.
        if (!all)
            throw CmdError("E02");
    } else if (subcommand == 'g') {
        // We don't currently support reading registers, memory, etc, from all
        // threads at once. GDB may never ask for this, but if it does we
        // should complain.
        if (all)
            throw CmdError("E03");

        // If GDB doesn't care which thread we're using, keep using the
        // current one, otherwise switch.
        if (!any && tid != tc->contextId()) {
            if (!selectThreadContext(tid))
                throw CmdError("E04");
            // Line up on an instruction boundary in the new thread.
            threadSwitching = true;
            trapEvent.id(tid);
            scheduleInstCommitEvent(&trapEvent, 0);
            return false;
        }
    } else {
        throw CmdError("E05");
    }

    send("OK");
    return true;
}

bool
BaseRemoteGDB::cmdMemR(GdbCommand::Context &ctx)
{
    const char *p = ctx.data;
    Addr addr = hex2i(&p);
    if (*p++ != ',')
        throw CmdError("E02");
    size_t len = hex2i(&p);
    if (*p != '\0')
        throw CmdError("E03");
    if (!acc(addr, len))
        throw CmdError("E05");

    char buf[len];
    if (!read(addr, len, buf))
        throw CmdError("E05");

    char temp[2 * len + 1];
    temp[2 * len] = '\0';
    mem2hex(temp, buf, len);
    send(temp);
    return true;
}

bool
BaseRemoteGDB::cmdMemW(GdbCommand::Context &ctx)
{
    const char *p = ctx.data;
    Addr addr = hex2i(&p);
    if (*p++ != ',')
        throw CmdError("E06");
    size_t len = hex2i(&p);
    if (*p++ != ':')
        throw CmdError("E07");
    if (len * 2 > ctx.len - (p - ctx.data))
        throw CmdError("E08");
    char buf[len];
    p = (char *)hex2mem(buf, p, len);
    if (p == NULL)
        throw CmdError("E09");
    if (!acc(addr, len))
        throw CmdError("E0A");
    if (!write(addr, len, buf))
        throw CmdError("E0B");
    send("OK");
    return true;
}

namespace {

std::pair<std::string, std::string>
splitAt(std::string str, const char * const delim)
{
    size_t pos = str.find_first_of(delim);
    if (pos == std::string::npos)
        return std::pair<std::string, std::string>(str, "");
    else
        return std::pair<std::string, std::string>(
                str.substr(0, pos), str.substr(pos + 1));
}

} // anonymous namespace

std::map<std::string, BaseRemoteGDB::QuerySetCommand>
        BaseRemoteGDB::queryMap = {
    { "C", { &BaseRemoteGDB::queryC } },
    { "Supported", { &BaseRemoteGDB::querySupported, ";" } },
    { "Xfer", { &BaseRemoteGDB::queryXfer } },
    { "fThreadInfo", { &BaseRemoteGDB::queryFThreadInfo } },
    { "sThreadInfo", { &BaseRemoteGDB::querySThreadInfo } },
};

void
BaseRemoteGDB::queryC(QuerySetCommand::Context &ctx)
{
    send("QC%x", encodeThreadId(tc->contextId()));
}

void
BaseRemoteGDB::querySupported(QuerySetCommand::Context &ctx)
{
    std::ostringstream oss;
    // This reply field mandatory. We can receive arbitrarily
    // long packets, so we could choose it to be arbitrarily large.
    // This is just an arbitrary filler value that seems to work.
    oss << "PacketSize=1024";
    for (const auto& feature : availableFeatures())
        oss << ';' << feature;
    send(oss.str());
}

void
BaseRemoteGDB::queryXfer(QuerySetCommand::Context &ctx)
{
    auto split = splitAt(ctx.args.at(0), ":");
    auto object = split.first;

    split = splitAt(split.second, ":");
    auto operation = split.first;

    // Only the "features" object and "read"ing are supported currently.
    if (object != "features" || operation != "read")
        throw Unsupported();

    // Extract the annex name.
    split = splitAt(split.second, ":");
    auto annex = split.first;

    // Read the contents of the annex.
    std::string content;
    if (!getXferFeaturesRead(annex, content))
        throw CmdError("E00");

    // Extract the offset and length.
    split = splitAt(split.second, ",");
    auto offset_str = split.first;
    auto length_str = split.second;

    const char *offset_ptr = offset_str.c_str();
    const char *length_ptr = length_str.c_str();
    auto offset = hex2i(&offset_ptr);
    auto length = hex2i(&length_ptr);
    if (offset_ptr != offset_str.c_str() + offset_str.length() ||
            length_ptr != length_str.c_str() + length_str.length()) {
        throw CmdError("E00");
    }

    std::string encoded;
    encodeXferResponse(content, encoded, offset, length);
    send(encoded);
}

void
BaseRemoteGDB::queryFThreadInfo(QuerySetCommand::Context &ctx)
{
    threadInfoIdx = 0;
    querySThreadInfo(ctx);
}

void
BaseRemoteGDB::querySThreadInfo(QuerySetCommand::Context &ctx)
{
    if (threadInfoIdx >= threads.size()) {
        threadInfoIdx = 0;
        send("l");
    } else {
        auto it = threads.begin();
        std::advance(it, threadInfoIdx++);
        send("m%x", encodeThreadId(it->second->contextId()));
    }
}

bool
BaseRemoteGDB::cmdQueryVar(GdbCommand::Context &ctx)
{
    // The query command goes until the first ':', or the end of the string.
    std::string s(ctx.data, ctx.len);
    auto query_split = splitAt({ ctx.data, (size_t)ctx.len }, ":");
    const auto &query_str = query_split.first;

    // Look up the query command, and report if it isn't found.
    auto query_it = queryMap.find(query_str);
    if (query_it == queryMap.end()) {
        DPRINTF(GDBMisc, "Unknown query %s\n", s);
        throw Unsupported();
    }

    // If it was found, construct a context.
    QuerySetCommand::Context qctx(query_str);

    const auto &query = query_it->second;
    auto remaining = std::move(query_split.second);
    if (!query.argSep) {
        qctx.args.emplace_back(std::move(remaining));
    } else {
        while (remaining != "") {
            auto arg_split = splitAt(remaining, query.argSep);
            qctx.args.emplace_back(std::move(arg_split.first));
            remaining = std::move(arg_split.second);
        }
    }

    (this->*(query.func))(qctx);

    return true;
}

std::vector<std::string>
BaseRemoteGDB::availableFeatures() const
{
    return {};
};

bool
BaseRemoteGDB::getXferFeaturesRead(
    const std::string &annex, std::string &output)
{
    return false;
}

void
BaseRemoteGDB::encodeBinaryData(
    const std::string &unencoded, std::string &encoded) const
{
    for (const char& c : unencoded) {
        if (c == '$' || c == '#' || c == '}' || c == '*') {
            encoded += '}';
            encoded += c ^ 0x20;
        } else {
            encoded += c;
        }
    }
}

void
BaseRemoteGDB::encodeXferResponse(const std::string &unencoded,
    std::string &encoded, size_t offset, size_t unencoded_length) const
{
    if (offset + unencoded_length < unencoded.length())
        encoded += 'm';
    else
        encoded += 'l';
    encodeBinaryData(unencoded.substr(offset, unencoded_length), encoded);
}

bool
BaseRemoteGDB::cmdDumpPageTable(GdbCommand::Context &ctx)
{
    send(tc->getProcessPtr()->pTable->externalize().c_str());
    return true;
}

bool
BaseRemoteGDB::cmdAsyncStep(GdbCommand::Context &ctx)
{
    const char *p = ctx.data;
    hex2i(&p); // Ignore the subcommand byte.
    if (*p++ == ';') {
        Addr new_pc = hex2i(&p);
        tc->pcState(new_pc);
    }
    setSingleStep();
    return false;
}

bool
BaseRemoteGDB::cmdStep(GdbCommand::Context &ctx)
{
    if (ctx.len) {
        const char *p = ctx.data;
        Addr new_pc = hex2i(&p);
        tc->pcState(new_pc);
    }
    setSingleStep();
    return false;
}

bool
BaseRemoteGDB::cmdClrHwBkpt(GdbCommand::Context &ctx)
{
    const char *p = ctx.data;
    char sub_cmd = *p++;
    if (*p++ != ',')
        throw CmdError("E0D");
    Addr addr = hex2i(&p);
    if (*p++ != ',')
        throw CmdError("E0D");
    size_t kind = hex2i(&p);

    DPRINTF(GDBMisc, "clear %s, addr=%#x, kind=%d\n",
            breakType(sub_cmd), addr, kind);

    switch (sub_cmd) {
      case GdbSoftBp:
        removeSoftBreak(addr, kind);
        break;
      case GdbHardBp:
        removeHardBreak(addr, kind);
        break;
      case GdbWriteWp:
      case GdbReadWp:
      case GdbAccWp:
      default: // unknown
        throw Unsupported();
    }
    send("OK");

    return true;
}

bool
BaseRemoteGDB::cmdSetHwBkpt(GdbCommand::Context &ctx)
{
    const char *p = ctx.data;
    char sub_cmd = *p++;
    if (*p++ != ',')
        throw CmdError("E0D");
    Addr addr = hex2i(&p);
    if (*p++ != ',')
        throw CmdError("E0D");
    size_t kind = hex2i(&p);

    DPRINTF(GDBMisc, "set %s, addr=%#x, kind=%d\n",
            breakType(sub_cmd), addr, kind);

    switch (sub_cmd) {
      case GdbSoftBp:
        insertSoftBreak(addr, kind);
        break;
      case GdbHardBp:
        insertHardBreak(addr, kind);
        break;
      case GdbWriteWp:
      case GdbReadWp:
      case GdbAccWp:
      default: // unknown
        throw Unsupported();
    }
    send("OK");

    return true;
}

} // namespace gem5
