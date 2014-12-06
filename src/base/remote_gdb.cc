/*
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

#include <signal.h>
#include <sys/signal.h>
#include <unistd.h>

#include <cstdio>
#include <string>

#include "arch/vtophys.hh"
#include "base/intmath.hh"
#include "base/remote_gdb.hh"
#include "base/socket.hh"
#include "base/trace.hh"
#include "config/the_isa.hh"
#include "cpu/base.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "debug/GDBAll.hh"
#include "mem/port.hh"
#include "mem/fs_translating_port_proxy.hh"
#include "mem/se_translating_port_proxy.hh"
#include "sim/full_system.hh"
#include "sim/system.hh"

using namespace std;
using namespace TheISA;

#ifndef NDEBUG
vector<BaseRemoteGDB *> debuggers;

void
debugger()
{
    static int current_debugger = -1;
    if (current_debugger >= 0 && current_debugger < (int)debuggers.size()) {
        BaseRemoteGDB *gdb = debuggers[current_debugger];
        if (!gdb->isattached())
            gdb->listener->accept();
        if (gdb->isattached())
            gdb->trap(SIGILL);
    }
}
#endif

///////////////////////////////////////////////////////////
//
//
//

GDBListener::InputEvent::InputEvent(GDBListener *l, int fd, int e)
    : PollEvent(fd, e), listener(l)
{}

void
GDBListener::InputEvent::process(int revent)
{
    listener->accept();
}

GDBListener::GDBListener(BaseRemoteGDB *g, int p)
    : inputEvent(NULL), gdb(g), port(p)
{
    assert(!gdb->listener);
    gdb->listener = this;
}

GDBListener::~GDBListener()
{
    if (inputEvent)
        delete inputEvent;
}

string
GDBListener::name()
{
    return gdb->name() + ".listener";
}

void
GDBListener::listen()
{
    if (ListenSocket::allDisabled()) {
        warn_once("Sockets disabled, not accepting gdb connections");
        return;
    }

    while (!listener.listen(port, true)) {
        DPRINTF(GDBMisc, "Can't bind port %d\n", port);
        port++;
    }

    inputEvent = new InputEvent(this, listener.getfd(), POLLIN);
    pollQueue.schedule(inputEvent);

#ifndef NDEBUG
    gdb->number = debuggers.size();
    debuggers.push_back(gdb);
#endif

#ifndef NDEBUG
    ccprintf(cerr, "%d: %s: listening for remote gdb #%d on port %d\n",
             curTick(), name(), gdb->number, port);
#else
    ccprintf(cerr, "%d: %s: listening for remote gdb on port %d\n",
             curTick(), name(), port);
#endif
}

void
GDBListener::accept()
{
    if (!listener.islistening())
        panic("GDBListener::accept(): cannot accept if we're not listening!");

    int sfd = listener.accept(true);

    if (sfd != -1) {
        if (gdb->isattached())
            close(sfd);
        else
            gdb->attach(sfd);
    }
}

BaseRemoteGDB::InputEvent::InputEvent(BaseRemoteGDB *g, int fd, int e)
    : PollEvent(fd, e), gdb(g)
{}

void
BaseRemoteGDB::InputEvent::process(int revent)
{
    if (revent & POLLIN) {
        gdb->trapEvent.type(SIGILL);
        gdb->scheduleInstCommitEvent(&gdb->trapEvent, 0);
    } else if (revent & POLLNVAL) {
        gdb->descheduleInstCommitEvent(&gdb->trapEvent);
        gdb->detach();
    }
}

void
BaseRemoteGDB::TrapEvent::process()
{
    gdb->trap(_type);
}

void
BaseRemoteGDB::SingleStepEvent::process()
{
    if (!gdb->singleStepEvent.scheduled())
        gdb->scheduleInstCommitEvent(&gdb->singleStepEvent, 1);
    gdb->trap(SIGTRAP);
}

BaseRemoteGDB::BaseRemoteGDB(System *_system, ThreadContext *c,
        size_t cacheSize) : inputEvent(NULL), trapEvent(this), listener(NULL),
        number(-1), fd(-1), active(false), attached(false), system(_system),
        context(c), gdbregs(cacheSize), singleStepEvent(this)
{
    memset(gdbregs.regs, 0, gdbregs.bytes());
}

BaseRemoteGDB::~BaseRemoteGDB()
{
    if (inputEvent)
        delete inputEvent;
}

string
BaseRemoteGDB::name()
{
    return system->name() + ".remote_gdb";
}

bool
BaseRemoteGDB::isattached()
{ return attached; }

void
BaseRemoteGDB::attach(int f)
{
    fd = f;

    inputEvent = new InputEvent(this, fd, POLLIN);
    pollQueue.schedule(inputEvent);

    attached = true;
    DPRINTFN("remote gdb attached\n");
}

void
BaseRemoteGDB::detach()
{
    attached = false;
    close(fd);
    fd = -1;

    pollQueue.remove(inputEvent);
    DPRINTFN("remote gdb detached\n");
}

const char *
BaseRemoteGDB::gdb_command(char cmd)
{
    switch (cmd) {
      case GDBSignal: return "KGDB_SIGNAL";
      case GDBSetBaud: return "KGDB_SET_BAUD";
      case GDBSetBreak: return "KGDB_SET_BREAK";
      case GDBCont: return "KGDB_CONT";
      case GDBAsyncCont: return "KGDB_ASYNC_CONT";
      case GDBDebug: return "KGDB_DEBUG";
      case GDBDetach: return "KGDB_DETACH";
      case GDBRegR: return "KGDB_REG_R";
      case GDBRegW: return "KGDB_REG_W";
      case GDBSetThread: return "KGDB_SET_THREAD";
      case GDBCycleStep: return "KGDB_CYCLE_STEP";
      case GDBSigCycleStep: return "KGDB_SIG_CYCLE_STEP";
      case GDBKill: return "KGDB_KILL";
      case GDBMemW: return "KGDB_MEM_W";
      case GDBMemR: return "KGDB_MEM_R";
      case GDBSetReg: return "KGDB_SET_REG";
      case GDBReadReg: return "KGDB_READ_REG";
      case GDBQueryVar: return "KGDB_QUERY_VAR";
      case GDBSetVar: return "KGDB_SET_VAR";
      case GDBReset: return "KGDB_RESET";
      case GDBStep: return "KGDB_STEP";
      case GDBAsyncStep: return "KGDB_ASYNC_STEP";
      case GDBThreadAlive: return "KGDB_THREAD_ALIVE";
      case GDBTargetExit: return "KGDB_TARGET_EXIT";
      case GDBBinaryDload: return "KGDB_BINARY_DLOAD";
      case GDBClrHwBkpt: return "KGDB_CLR_HW_BKPT";
      case GDBSetHwBkpt: return "KGDB_SET_HW_BKPT";
      case GDBStart: return "KGDB_START";
      case GDBEnd: return "KGDB_END";
      case GDBGoodP: return "KGDB_GOODP";
      case GDBBadP: return "KGDB_BADP";
      default: return "KGDB_UNKNOWN";
    }
}

/////////////////////////
//
//

uint8_t
BaseRemoteGDB::getbyte()
{
    uint8_t b;
    if (::read(fd, &b, 1) != 1)
        warn("could not read byte from debugger");
    return b;
}

void
BaseRemoteGDB::putbyte(uint8_t b)
{
    if (::write(fd, &b, 1) != 1)
        warn("could not write byte to debugger");
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
        //Start sending a packet
        putbyte(GDBStart);
        //Send the contents, and also keep a check sum.
        for (csum = 0; (c = *p); p++) {
            putbyte(c);
            csum += c;
        }
        //Send the ending character.
        putbyte(GDBEnd);
        //Sent the checksum.
        putbyte(i2digit(csum >> 4));
        putbyte(i2digit(csum));
        //Try transmitting over and over again until the other end doesn't send an
        //error back.
    } while ((c = getbyte() & 0x7f) == GDBBadP);
}

// Receive a packet from gdb
int
BaseRemoteGDB::recv(char *bp, int maxlen)
{
    char *p;
    int c, csum;
    int len;

    do {
        p = bp;
        csum = len = 0;
        //Find the beginning of a packet
        while ((c = getbyte()) != GDBStart)
            ;

        //Read until you find the end of the data in the packet, and keep
        //track of the check sum.
        while ((c = getbyte()) != GDBEnd && len < maxlen) {
            c &= 0x7f;
            csum += c;
            *p++ = c;
            len++;
        }

        //Mask the check sum, and terminate the command string.
        csum &= 0xff;
        *p = '\0';

        //If the command was too long, report an error.
        if (len >= maxlen) {
            putbyte(GDBBadP);
            continue;
        }

        //Bring in the checksum. If the check sum matches, csum will be 0.
        csum -= digit2i(getbyte()) * 16;
        csum -= digit2i(getbyte());

        //If the check sum was correct
        if (csum == 0) {
            //Report that the packet was received correctly
            putbyte(GDBGoodP);
            // Sequence present?
            if (bp[2] == ':') {
                putbyte(bp[0]);
                putbyte(bp[1]);
                len -= 3;
                memcpy(bp, bp+3, len);
            }
            break;
        }
        //Otherwise, report that there was a mistake.
        putbyte(GDBBadP);
    } while (1);

    DPRINTF(GDBRecv, "recv:  %s: %s\n", gdb_command(*bp), bp);

    return (len);
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
        FSTranslatingPortProxy &proxy = context->getVirtProxy();
        proxy.readBlob(vaddr, (uint8_t*)data, size);
    } else {
        SETranslatingPortProxy &proxy = context->getMemProxy();
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
        FSTranslatingPortProxy &proxy = context->getVirtProxy();
        proxy.writeBlob(vaddr, (uint8_t*)data, size);
    } else {
        SETranslatingPortProxy &proxy = context->getMemProxy();
        proxy.writeBlob(vaddr, (uint8_t*)data, size);
    }

    return true;
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

PCEventQueue *BaseRemoteGDB::getPcEventQueue()
{
    return &system->pcEventQueue;
}

EventQueue *
BaseRemoteGDB::getComInstEventQueue()
{
    BaseCPU *cpu = context->getCpuPtr();
    return cpu->comInstEventQueue[context->threadId()];
}

void
BaseRemoteGDB::scheduleInstCommitEvent(Event *ev, int delta)
{
    EventQueue *eq = getComInstEventQueue();
    // Here "ticks" aren't simulator ticks which measure time, they're
    // instructions committed by the CPU.
    eq->schedule(ev, eq->getCurTick() + delta);
}

void
BaseRemoteGDB::descheduleInstCommitEvent(Event *ev)
{
    if (ev->scheduled())
        getComInstEventQueue()->deschedule(ev);
}

bool
BaseRemoteGDB::checkBpLen(size_t len)
{
    return len == sizeof(MachInst);
}

BaseRemoteGDB::HardBreakpoint::HardBreakpoint(BaseRemoteGDB *_gdb, Addr pc)
    : PCEvent(_gdb->getPcEventQueue(), "HardBreakpoint Event", pc),
      gdb(_gdb), refcount(0)
{
    DPRINTF(GDBMisc, "creating hardware breakpoint at %#x\n", evpc);
}

void
BaseRemoteGDB::HardBreakpoint::process(ThreadContext *tc)
{
    DPRINTF(GDBMisc, "handling hardware breakpoint at %#x\n", pc());

    if (tc == gdb->context)
        gdb->trap(SIGTRAP);
}

bool
BaseRemoteGDB::insertSoftBreak(Addr addr, size_t len)
{
    if (!checkBpLen(len))
        panic("invalid length\n");

    return insertHardBreak(addr, len);
}

bool
BaseRemoteGDB::removeSoftBreak(Addr addr, size_t len)
{
    if (!checkBpLen(len))
        panic("invalid length\n");

    return removeHardBreak(addr, len);
}

bool
BaseRemoteGDB::insertHardBreak(Addr addr, size_t len)
{
    if (!checkBpLen(len))
        panic("invalid length\n");

    DPRINTF(GDBMisc, "inserting hardware breakpoint at %#x\n", addr);

    HardBreakpoint *&bkpt = hardBreakMap[addr];
    if (bkpt == 0)
        bkpt = new HardBreakpoint(this, addr);

    bkpt->refcount++;

    return true;
}

bool
BaseRemoteGDB::removeHardBreak(Addr addr, size_t len)
{
    if (!checkBpLen(len))
        panic("invalid length\n");

    DPRINTF(GDBMisc, "removing hardware breakpoint at %#x\n", addr);

    break_iter_t i = hardBreakMap.find(addr);
    if (i == hardBreakMap.end())
        return false;

    HardBreakpoint *hbp = (*i).second;
    if (--hbp->refcount == 0) {
        delete hbp;
        hardBreakMap.erase(i);
    }

    return true;
}

void
BaseRemoteGDB::setTempBreakpoint(Addr bkpt)
{
    DPRINTF(GDBMisc, "setTempBreakpoint: addr=%#x\n", bkpt);
    insertHardBreak(bkpt, sizeof(TheISA::MachInst));
}

void
BaseRemoteGDB::clearTempBreakpoint(Addr &bkpt)
{
    DPRINTF(GDBMisc, "setTempBreakpoint: addr=%#x\n", bkpt);
    removeHardBreak(bkpt, sizeof(TheISA::MachInst));
    bkpt = 0;
}

const char *
BaseRemoteGDB::break_type(char c)
{
    switch(c) {
      case '0': return "software breakpoint";
      case '1': return "hardware breakpoint";
      case '2': return "write watchpoint";
      case '3': return "read watchpoint";
      case '4': return "access watchpoint";
      default: return "unknown breakpoint/watchpoint";
    }
}

// This function does all command processing for interfacing to a
// remote gdb.  Note that the error codes are ignored by gdb at
// present, but might eventually become meaningful. (XXX) It might
// makes sense to use POSIX errno values, because that is what the
// gdb/remote.c functions want to return.
bool
BaseRemoteGDB::trap(int type)
{
    uint64_t val;
    size_t datalen, len;
    char data[GDBPacketBufLen + 1];
    char *buffer;
    size_t bufferSize;
    const char *p;
    char command, subcmd;
    string var;
    bool ret;

    if (!attached)
        return false;

    bufferSize = gdbregs.bytes() * 2 + 256;
    buffer = (char*)malloc(bufferSize);

    DPRINTF(GDBMisc, "trap: PC=%s\n", context->pcState());

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
        snprintf((char *)buffer, bufferSize, "S%02x", type);
        send(buffer);
    }

    // Stick frame regs into our reg cache.
    getregs();

    for (;;) {
        datalen = recv(data, sizeof(data));
        data[sizeof(data) - 1] = 0; // Sentinel
        command = data[0];
        subcmd = 0;
        p = data + 1;
        switch (command) {

          case GDBSignal:
            // if this command came from a running gdb, answer it --
            // the other guy has no way of knowing if we're in or out
            // of this loop when he issues a "remote-signal".
            snprintf((char *)buffer, bufferSize,
                    "S%02x", type);
            send(buffer);
            continue;

          case GDBRegR:
            if (2 * gdbregs.bytes() > bufferSize)
                panic("buffer too small");

            mem2hex(buffer, gdbregs.regs, gdbregs.bytes());
            send(buffer);
            continue;

          case GDBRegW:
            p = hex2mem(gdbregs.regs, p, gdbregs.bytes());
            if (p == NULL || *p != '\0')
                send("E01");
            else {
                setregs();
                send("OK");
            }
            continue;

#if 0
          case GDBSetReg:
            val = hex2i(&p);
            if (*p++ != '=') {
                send("E01");
                continue;
            }
            if (val < 0 && val >= KGDB_NUMREGS) {
                send("E01");
                continue;
            }

            gdbregs.regs[val] = hex2i(&p);
            setregs();
            send("OK");

            continue;
#endif

          case GDBMemR:
            val = hex2i(&p);
            if (*p++ != ',') {
                send("E02");
                continue;
            }
            len = hex2i(&p);
            if (*p != '\0') {
                send("E03");
                continue;
            }
            if (len > bufferSize) {
                send("E04");
                continue;
            }
            if (!acc(val, len)) {
                send("E05");
                continue;
            }

            if (read(val, (size_t)len, (char *)buffer)) {
               // variable length array would be nice, but C++ doesn't
               // officially support those...
               char *temp = new char[2*len+1];
               mem2hex(temp, buffer, len);
               send(temp);
               delete [] temp;
            } else {
               send("E05");
            }
            continue;

          case GDBMemW:
            val = hex2i(&p);
            if (*p++ != ',') {
                send("E06");
                continue;
            }
            len = hex2i(&p);
            if (*p++ != ':') {
                send("E07");
                continue;
            }
            if (len > datalen - (p - data)) {
                send("E08");
                continue;
            }
            p = hex2mem(buffer, p, bufferSize);
            if (p == NULL) {
                send("E09");
                continue;
            }
            if (!acc(val, len)) {
                send("E0A");
                continue;
            }
            if (write(val, (size_t)len, (char *)buffer))
              send("OK");
            else
              send("E0B");
            continue;

          case GDBSetThread:
            subcmd = *p++;
            val = hex2i(&p);
            if (val == 0)
                send("OK");
            else
                send("E01");
            continue;

          case GDBDetach:
          case GDBKill:
            active = false;
            clearSingleStep();
            detach();
            goto out;

          case GDBAsyncCont:
            subcmd = hex2i(&p);
            if (*p++ == ';') {
                val = hex2i(&p);
                context->pcState(val);
            }
            clearSingleStep();
            goto out;

          case GDBCont:
            if (p - data < (ptrdiff_t)datalen) {
                val = hex2i(&p);
                context->pcState(val);
            }
            clearSingleStep();
            goto out;

          case GDBAsyncStep:
            subcmd = hex2i(&p);
            if (*p++ == ';') {
                val = hex2i(&p);
                context->pcState(val);
            }
            setSingleStep();
            goto out;

          case GDBStep:
            if (p - data < (ptrdiff_t)datalen) {
                val = hex2i(&p);
                context->pcState(val);
            }
            setSingleStep();
            goto out;

          case GDBClrHwBkpt:
            subcmd = *p++;
            if (*p++ != ',') send("E0D");
            val = hex2i(&p);
            if (*p++ != ',') send("E0D");
            len = hex2i(&p);

            DPRINTF(GDBMisc, "clear %s, addr=%#x, len=%d\n",
                    break_type(subcmd), val, len);

            ret = false;

            switch (subcmd) {
              case '0': // software breakpoint
                ret = removeSoftBreak(val, len);
                break;

              case '1': // hardware breakpoint
                ret = removeHardBreak(val, len);
                break;

              case '2': // write watchpoint
              case '3': // read watchpoint
              case '4': // access watchpoint
              default: // unknown
                send("");
                break;
            }

            send(ret ? "OK" : "E0C");
            continue;

          case GDBSetHwBkpt:
            subcmd = *p++;
            if (*p++ != ',') send("E0D");
            val = hex2i(&p);
            if (*p++ != ',') send("E0D");
            len = hex2i(&p);

            DPRINTF(GDBMisc, "set %s, addr=%#x, len=%d\n",
                    break_type(subcmd), val, len);

            ret = false;

            switch (subcmd) {
              case '0': // software breakpoint
                ret = insertSoftBreak(val, len);
                break;

              case '1': // hardware breakpoint
                ret = insertHardBreak(val, len);
                break;

              case '2': // write watchpoint
              case '3': // read watchpoint
              case '4': // access watchpoint
              default: // unknown
                send("");
                break;
            }

            send(ret ? "OK" : "E0C");
            continue;

          case GDBQueryVar:
            var = string(p, datalen - 1);
            if (var == "C")
                send("QC0");
            else
                send("");
            continue;

          case GDBSetBaud:
          case GDBSetBreak:
          case GDBDebug:
          case GDBCycleStep:
          case GDBSigCycleStep:
          case GDBReadReg:
          case GDBSetVar:
          case GDBReset:
          case GDBThreadAlive:
          case GDBTargetExit:
          case GDBBinaryDload:
            // Unsupported command
            DPRINTF(GDBMisc, "Unsupported command: %s\n",
                    gdb_command(command));
            DDUMP(GDBMisc, (uint8_t *)data, datalen);
            send("");
            continue;

          default:
            // Unknown command.
            DPRINTF(GDBMisc, "Unknown command: %c(%#x)\n",
                    command, command);
            send("");
            continue;


        }
    }

  out:
    free(buffer);
    return true;
}

// Convert a hex digit into an integer.
// This returns -1 if the argument passed is no valid hex digit.
int
BaseRemoteGDB::digit2i(char c)
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
BaseRemoteGDB::i2digit(int n)
{
    return ("0123456789abcdef"[n & 0x0f]);
}

// Convert a byte array into an hex string.
void
BaseRemoteGDB::mem2hex(void *vdst, const void *vsrc, int len)
{
    char *dst = (char *)vdst;
    const char *src = (const char *)vsrc;

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
BaseRemoteGDB::hex2mem(void *vdst, const char *src, int maxlen)
{
    char *dst = (char *)vdst;
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
    return (src);
}

// Convert an hex string into an integer.
// This returns a pointer to the character following the last valid
// hex digit.
Addr
BaseRemoteGDB::hex2i(const char **srcp)
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
    return (r);
}

