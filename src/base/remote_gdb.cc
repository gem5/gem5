/*
 * Copyright 2015 LabWare
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
 *
 * Authors: Nathan Binkert
 *          Boris Shingarov
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

#include <csignal>
#include <cstdint>
#include <cstdio>
#include <string>

#include "arch/vtophys.hh"
#include "base/intmath.hh"
#include "base/socket.hh"
#include "base/trace.hh"
#include "config/the_isa.hh"
#include "cpu/base.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "debug/GDBAll.hh"
#include "mem/fs_translating_port_proxy.hh"
#include "mem/port.hh"
#include "mem/se_translating_port_proxy.hh"
#include "sim/full_system.hh"
#include "sim/system.hh"

using namespace std;
using namespace TheISA;

static const char GDBStart = '$';
static const char GDBEnd = '#';
static const char GDBGoodP = '+';
static const char GDBBadP = '-';

vector<BaseRemoteGDB *> debuggers;

class HardBreakpoint : public PCEvent
{
  private:
    BaseRemoteGDB *gdb;

  public:
    int refcount;

  public:
    HardBreakpoint(BaseRemoteGDB *_gdb, PCEventQueue *q, Addr pc)
        : PCEvent(q, "HardBreakpoint Event", pc),
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
            gdb->trap(SIGTRAP);
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
    string error;
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
        return (-1);
}

// Convert the low 4 bits of an integer into an hex digit.
char
i2digit(int n)
{
    return ("0123456789abcdef"[n & 0x0f]);
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
hex2mem(char *vdst, const char *src, int maxlen)
{
    char *dst = vdst;
    int msb, lsb;

    while (*src && maxlen--) {
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

enum GdbBreakpointType {
    GdbSoftBp = '0',
    GdbHardBp = '1',
    GdbWriteWp = '2',
    GdbReadWp = '3',
    GdbAccWp = '4',
};

#ifndef NDEBUG
const char *
break_type(char c)
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
#endif

std::map<Addr, HardBreakpoint *> hardBreakMap;

EventQueue *
getComInstEventQueue(ThreadContext *tc)
{
    return tc->getCpuPtr()->comInstEventQueue[tc->threadId()];
}

}

BaseRemoteGDB::BaseRemoteGDB(System *_system, ThreadContext *c, int _port) :
        connectEvent(nullptr), dataEvent(nullptr), _port(_port), fd(-1),
        active(false), attached(false), sys(_system), tc(c),
        trapEvent(this), singleStepEvent(*this)
{
    debuggers.push_back(this);
}

BaseRemoteGDB::~BaseRemoteGDB()
{
    delete connectEvent;
    delete dataEvent;
}

string
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

    connectEvent = new ConnectEvent(this, listener.getfd(), POLLIN);
    pollQueue.schedule(connectEvent);

    ccprintf(cerr, "%d: %s: listening for remote gdb on port %d\n",
             curTick(), name(), _port);
}

void
BaseRemoteGDB::connect()
{
    panic_if(!listener.islistening(),
             "Cannot accept GDB connections if we're not listening!");

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

    dataEvent = new DataEvent(this, fd, POLLIN);
    pollQueue.schedule(dataEvent);

    attached = true;
    DPRINTFN("remote gdb attached\n");
}

void
BaseRemoteGDB::detach()
{
    attached = false;
    active = false;
    clearSingleStep();
    close(fd);
    fd = -1;

    pollQueue.remove(dataEvent);
    DPRINTFN("remote gdb detached\n");
}

// This function does all command processing for interfacing to a
// remote gdb.  Note that the error codes are ignored by gdb at
// present, but might eventually become meaningful. (XXX) It might
// makes sense to use POSIX errno values, because that is what the
// gdb/remote.c functions want to return.
bool
BaseRemoteGDB::trap(int type)
{

    if (!attached)
        return false;

    DPRINTF(GDBMisc, "trap: PC=%s\n", tc->pcState());

    clearSingleStep();

    /*
     * The first entry to this function is normally through
     * a breakpoint trap in kgdb_connect(), in which case we
     * must advance past the breakpoint because gdb will not.
     *
     * On the first entry here, we expect that gdb is not yet
     * listening to us, so just enter the interaction loop.
     * After the debugger is "active" (connected) it will be
     * waiting for a "signaled" message from us.
     */
    if (!active) {
        active = true;
    } else {
        // Tell remote host that an exception has occurred.
        send(csprintf("S%02x", type).c_str());
    }

    // Stick frame regs into our reg cache.
    regCachePtr = gdbRegs();
    regCachePtr->getRegs(tc);

    GdbCommand::Context cmdCtx;
    cmdCtx.type = type;
    std::vector<char> data;

    for (;;) {
        try {
            recv(data);
            if (data.size() == 1)
                throw BadClient();
            cmdCtx.cmd_byte = data[0];
            cmdCtx.data = data.data() + 1;
            // One for sentinel, one for cmd_byte.
            cmdCtx.len = data.size() - 2;

            auto cmdIt = command_map.find(cmdCtx.cmd_byte);
            if (cmdIt == command_map.end()) {
                DPRINTF(GDBMisc, "Unknown command: %c(%#x)\n",
                        cmdCtx.cmd_byte, cmdCtx.cmd_byte);
                throw Unsupported();
            }
            cmdCtx.cmd = &(cmdIt->second);

            if (!(this->*(cmdCtx.cmd->func))(cmdCtx))
                break;

        } catch (BadClient &e) {
            if (e.warning)
                warn(e.warning);
            detach();
            break;
        } catch (Unsupported &e) {
            send("");
        } catch (CmdError &e) {
            send(e.error.c_str());
        } catch (...) {
            panic("Unrecognzied GDB exception.");
        }
    }

    return true;
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
        scheduleInstCommitEvent(&trapEvent, 0);
    } else if (revent & POLLNVAL) {
        descheduleInstCommitEvent(&trapEvent);
        detach();
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

// Read bytes from kernel address space for debugger.
bool
BaseRemoteGDB::read(Addr vaddr, size_t size, char *data)
{
    static Addr lastaddr = 0;
    static size_t lastsize = 0;

    if (vaddr < 10) {
      DPRINTF(GDBRead, "read:  reading memory location zero!\n");
      vaddr = lastaddr + lastsize;
    }

    DPRINTF(GDBRead, "read:  addr=%#x, size=%d", vaddr, size);

    if (FullSystem) {
        FSTranslatingPortProxy &proxy = tc->getVirtProxy();
        proxy.readBlob(vaddr, (uint8_t*)data, size);
    } else {
        SETranslatingPortProxy &proxy = tc->getMemProxy();
        proxy.readBlob(vaddr, (uint8_t*)data, size);
    }

#if TRACING_ON
    if (DTRACE(GDBRead)) {
        if (DTRACE(GDBExtra)) {
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
    static Addr lastaddr = 0;
    static size_t lastsize = 0;

    if (vaddr < 10) {
      DPRINTF(GDBWrite, "write: writing memory location zero!\n");
      vaddr = lastaddr + lastsize;
    }

    if (DTRACE(GDBWrite)) {
        DPRINTFN("write: addr=%#x, size=%d", vaddr, size);
        if (DTRACE(GDBExtra)) {
            char buf[1024];
            mem2hex(buf, data, size);
            DPRINTFNR(": %s\n", buf);
        } else
            DPRINTFNR("\n");
    }
    if (FullSystem) {
        FSTranslatingPortProxy &proxy = tc->getVirtProxy();
        proxy.writeBlob(vaddr, (uint8_t*)data, size);
    } else {
        SETranslatingPortProxy &proxy = tc->getMemProxy();
        proxy.writeBlob(vaddr, (uint8_t*)data, size);
    }

    return true;
}

void
BaseRemoteGDB::singleStep()
{
    if (!singleStepEvent.scheduled())
        scheduleInstCommitEvent(&singleStepEvent, 1);
    trap(SIGTRAP);
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
BaseRemoteGDB::insertSoftBreak(Addr addr, size_t len)
{
    if (!checkBpLen(len))
        throw BadClient("Invalid breakpoint length\n");

    return insertHardBreak(addr, len);
}

void
BaseRemoteGDB::removeSoftBreak(Addr addr, size_t len)
{
    if (!checkBpLen(len))
        throw BadClient("Invalid breakpoint length.\n");

    return removeHardBreak(addr, len);
}

void
BaseRemoteGDB::insertHardBreak(Addr addr, size_t len)
{
    if (!checkBpLen(len))
        throw BadClient("Invalid breakpoint length\n");

    DPRINTF(GDBMisc, "Inserting hardware breakpoint at %#x\n", addr);

    HardBreakpoint *&bkpt = hardBreakMap[addr];
    if (bkpt == 0)
        bkpt = new HardBreakpoint(this, &sys->pcEventQueue, addr);

    bkpt->refcount++;
}

void
BaseRemoteGDB::removeHardBreak(Addr addr, size_t len)
{
    if (!checkBpLen(len))
        throw BadClient("Invalid breakpoint length\n");

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
BaseRemoteGDB::clearTempBreakpoint(Addr &bkpt)
{
    DPRINTF(GDBMisc, "setTempBreakpoint: addr=%#x\n", bkpt);
    removeHardBreak(bkpt, sizeof(TheISA::MachInst));
    bkpt = 0;
}

void
BaseRemoteGDB::setTempBreakpoint(Addr bkpt)
{
    DPRINTF(GDBMisc, "setTempBreakpoint: addr=%#x\n", bkpt);
    insertHardBreak(bkpt, sizeof(TheISA::MachInst));
}

void
BaseRemoteGDB::scheduleInstCommitEvent(Event *ev, int delta)
{
    EventQueue *eq = getComInstEventQueue(tc);
    // Here "ticks" aren't simulator ticks which measure time, they're
    // instructions committed by the CPU.
    eq->schedule(ev, eq->getCurTick() + delta);
}

void
BaseRemoteGDB::descheduleInstCommitEvent(Event *ev)
{
    if (ev->scheduled())
        getComInstEventQueue(tc)->deschedule(ev);
}

std::map<char, BaseRemoteGDB::GdbCommand> BaseRemoteGDB::command_map = {
    // last signal
    { '?', { "KGDB_SIGNAL", &BaseRemoteGDB::cmd_signal } },
    // set baud (deprecated)
    { 'b', { "KGDB_SET_BAUD", &BaseRemoteGDB::cmd_unsupported } },
    // set breakpoint (deprecated)
    { 'B', { "KGDB_SET_BREAK", &BaseRemoteGDB::cmd_unsupported } },
    // resume
    { 'c', { "KGDB_CONT", &BaseRemoteGDB::cmd_cont } },
    // continue with signal
    { 'C', { "KGDB_ASYNC_CONT", &BaseRemoteGDB::cmd_async_cont } },
    // toggle debug flags (deprecated)
    { 'd', { "KGDB_DEBUG", &BaseRemoteGDB::cmd_unsupported } },
    // detach remote gdb
    { 'D', { "KGDB_DETACH", &BaseRemoteGDB::cmd_detach } },
    // read general registers
    { 'g', { "KGDB_REG_R", &BaseRemoteGDB::cmd_reg_r } },
    // write general registers
    { 'G', { "KGDB_REG_W", &BaseRemoteGDB::cmd_reg_w } },
    // set thread
    { 'H', { "KGDB_SET_THREAD", &BaseRemoteGDB::cmd_set_thread } },
    // step a single cycle
    { 'i', { "KGDB_CYCLE_STEP", &BaseRemoteGDB::cmd_unsupported } },
    // signal then cycle step
    { 'I', { "KGDB_SIG_CYCLE_STEP", &BaseRemoteGDB::cmd_unsupported } },
    // kill program
    { 'k', { "KGDB_KILL", &BaseRemoteGDB::cmd_detach } },
    // read memory
    { 'm', { "KGDB_MEM_R", &BaseRemoteGDB::cmd_mem_r } },
    // write memory
    { 'M', { "KGDB_MEM_W", &BaseRemoteGDB::cmd_mem_w } },
    // read register
    { 'p', { "KGDB_READ_REG", &BaseRemoteGDB::cmd_unsupported } },
    // write register
    { 'P', { "KGDB_SET_REG", &BaseRemoteGDB::cmd_unsupported } },
    // query variable
    { 'q', { "KGDB_QUERY_VAR", &BaseRemoteGDB::cmd_query_var } },
    // set variable
    { 'Q', { "KGDB_SET_VAR", &BaseRemoteGDB::cmd_unsupported } },
    // reset system (deprecated)
    { 'r', { "KGDB_RESET", &BaseRemoteGDB::cmd_unsupported } },
    // step
    { 's', { "KGDB_STEP", &BaseRemoteGDB::cmd_step } },
    // signal and step
    { 'S', { "KGDB_ASYNC_STEP", &BaseRemoteGDB::cmd_async_step } },
    // find out if the thread is alive
    { 'T', { "KGDB_THREAD_ALIVE", &BaseRemoteGDB::cmd_unsupported } },
    // target exited
    { 'W', { "KGDB_TARGET_EXIT", &BaseRemoteGDB::cmd_unsupported } },
    // write memory
    { 'X', { "KGDB_BINARY_DLOAD", &BaseRemoteGDB::cmd_unsupported } },
    // remove breakpoint or watchpoint
    { 'z', { "KGDB_CLR_HW_BKPT", &BaseRemoteGDB::cmd_clr_hw_bkpt } },
    // insert breakpoint or watchpoint
    { 'Z', { "KGDB_SET_HW_BKPT", &BaseRemoteGDB::cmd_set_hw_bkpt } },
};

bool
BaseRemoteGDB::checkBpLen(size_t len)
{
    return len == sizeof(MachInst);
}

bool
BaseRemoteGDB::cmd_unsupported(GdbCommand::Context &ctx)
{
    DPRINTF(GDBMisc, "Unsupported command: %s\n", ctx.cmd->name);
    DDUMP(GDBMisc, ctx.data, ctx.len);
    throw Unsupported();
}


bool
BaseRemoteGDB::cmd_signal(GdbCommand::Context &ctx)
{
    send(csprintf("S%02x", ctx.type).c_str());
    return true;
}

bool
BaseRemoteGDB::cmd_cont(GdbCommand::Context &ctx)
{
    const char *p = ctx.data;
    if (ctx.len) {
        Addr newPc = hex2i(&p);
        tc->pcState(newPc);
    }
    clearSingleStep();
    return false;
}

bool
BaseRemoteGDB::cmd_async_cont(GdbCommand::Context &ctx)
{
    const char *p = ctx.data;
    hex2i(&p);
    if (*p++ == ';') {
        Addr newPc = hex2i(&p);
        tc->pcState(newPc);
    }
    clearSingleStep();
    return false;
}

bool
BaseRemoteGDB::cmd_detach(GdbCommand::Context &ctx)
{
    detach();
    return false;
}

bool
BaseRemoteGDB::cmd_reg_r(GdbCommand::Context &ctx)
{
    char buf[2 * regCachePtr->size() + 1];
    buf[2 * regCachePtr->size()] = '\0';
    mem2hex(buf, regCachePtr->data(), regCachePtr->size());
    send(buf);
    return true;
}

bool
BaseRemoteGDB::cmd_reg_w(GdbCommand::Context &ctx)
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
BaseRemoteGDB::cmd_set_thread(GdbCommand::Context &ctx)
{
    const char *p = ctx.data + 1; // Ignore the subcommand byte.
    if (hex2i(&p) != 0)
        throw CmdError("E01");
    send("OK");
    return true;
}

bool
BaseRemoteGDB::cmd_mem_r(GdbCommand::Context &ctx)
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
BaseRemoteGDB::cmd_mem_w(GdbCommand::Context &ctx)
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

bool
BaseRemoteGDB::cmd_query_var(GdbCommand::Context &ctx)
{
    if (string(ctx.data, ctx.len - 1) != "C")
        throw Unsupported();
    send("QC0");
    return true;
}

bool
BaseRemoteGDB::cmd_async_step(GdbCommand::Context &ctx)
{
    const char *p = ctx.data;
    hex2i(&p); // Ignore the subcommand byte.
    if (*p++ == ';') {
        Addr newPc = hex2i(&p);
        tc->pcState(newPc);
    }
    setSingleStep();
    return false;
}

bool
BaseRemoteGDB::cmd_step(GdbCommand::Context &ctx)
{
    if (ctx.len) {
        const char *p = ctx.data;
        Addr newPc = hex2i(&p);
        tc->pcState(newPc);
    }
    setSingleStep();
    return false;
}

bool
BaseRemoteGDB::cmd_clr_hw_bkpt(GdbCommand::Context &ctx)
{
    const char *p = ctx.data;
    char subcmd = *p++;
    if (*p++ != ',')
        throw CmdError("E0D");
    Addr addr = hex2i(&p);
    if (*p++ != ',')
        throw CmdError("E0D");
    size_t len = hex2i(&p);

    DPRINTF(GDBMisc, "clear %s, addr=%#x, len=%d\n",
            break_type(subcmd), addr, len);

    switch (subcmd) {
      case GdbSoftBp:
        removeSoftBreak(addr, len);
        break;
      case GdbHardBp:
        removeHardBreak(addr, len);
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
BaseRemoteGDB::cmd_set_hw_bkpt(GdbCommand::Context &ctx)
{
    const char *p = ctx.data;
    char subcmd = *p++;
    if (*p++ != ',')
        throw CmdError("E0D");
    Addr addr = hex2i(&p);
    if (*p++ != ',')
        throw CmdError("E0D");
    size_t len = hex2i(&p);

    DPRINTF(GDBMisc, "set %s, addr=%#x, len=%d\n",
            break_type(subcmd), addr, len);

    switch (subcmd) {
      case GdbSoftBp:
        insertSoftBreak(addr, len);
        break;
      case GdbHardBp:
        insertHardBreak(addr, len);
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
