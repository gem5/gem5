/*
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
 * Copyright (c) 1990, 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * This software was developed by the Computer Systems Engineering group
 * at Lawrence Berkeley Laboratory under DARPA contract BG 91-66 and
 * contributed to Berkeley.
 *
 * All advertising materials mentioning features or use of this software
 * must display the following acknowledgement:
 *	This product includes software developed by the University of
 *	California, Lawrence Berkeley Laboratories.
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
 *	This product includes software developed by the University of
 *	California, Berkeley and its contributors.
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
 *	@(#)kgdb_stub.c	8.4 (Berkeley) 1/12/94
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
 *	This product includes software developed by the NetBSD
 *	Foundation, Inc. and its contributors.
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

#include <sys/signal.h>

#include <cstdio>
#include <string>
#include <unistd.h>

#include "base/intmath.hh"
#include "base/kgdb.h"
#include "base/remote_gdb.hh"
#include "base/socket.hh"
#include "base/trace.hh"
#include "cpu/exec_context.hh"
#include "cpu/static_inst.hh"
#include "mem/functional/physical.hh"
#include "sim/system.hh"
#include "targetarch/vtophys.hh"

using namespace std;

#ifdef DEBUG
vector<RemoteGDB *> debuggers;
int current_debugger = -1;

void
debugger()
{
    if (current_debugger >= 0 && current_debugger < debuggers.size()) {
        RemoteGDB *gdb = debuggers[current_debugger];
        if (!gdb->isattached())
            gdb->listener->accept();
        if (gdb->isattached())
            gdb->trap(ALPHA_KENTRY_IF);
    }
}
#endif

///////////////////////////////////////////////////////////
//
//
//

GDBListener::Event::Event(GDBListener *l, int fd, int e)
    : PollEvent(fd, e), listener(l)
{}

void
GDBListener::Event::process(int revent)
{
    listener->accept();
}

GDBListener::GDBListener(RemoteGDB *g, int p)
    : event(NULL), gdb(g), port(p)
{
    assert(!gdb->listener);
    gdb->listener = this;
}

GDBListener::~GDBListener()
{
    if (event)
        delete event;
}

string
GDBListener::name()
{
    return gdb->name() + ".listener";
}

void
GDBListener::listen()
{
    while (!listener.listen(port, true)) {
        DPRINTF(GDBMisc, "Can't bind port %d\n", port);
        port++;
    }

    event = new Event(this, listener.getfd(), POLLIN);
    pollQueue.schedule(event);

#ifdef DEBUG
    gdb->number = debuggers.size();
    debuggers.push_back(gdb);
#endif

#ifdef DEBUG
    ccprintf(cerr, "%d: %s: listening for remote gdb #%d on port %d\n",
             curTick, name(), gdb->number, port);
#else
    ccprintf(cerr, "%d: %s: listening for remote gdb on port %d\n",
             curTick, name(), port);
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

///////////////////////////////////////////////////////////
//
//
//
int digit2i(char);
char i2digit(int);
void mem2hex(void *, const void *, int);
const char *hex2mem(void *, const char *, int);
Addr hex2i(const char **);

RemoteGDB::Event::Event(RemoteGDB *g, int fd, int e)
    : PollEvent(fd, e), gdb(g)
{}

void
RemoteGDB::Event::process(int revent)
{
    if (revent & POLLIN)
        gdb->trap(ALPHA_KENTRY_IF);
    else if (revent & POLLNVAL)
        gdb->detach();
}

RemoteGDB::RemoteGDB(System *_system, ExecContext *c)
    : event(NULL), listener(NULL), number(-1), fd(-1),
      active(false), attached(false),
      system(_system), pmem(_system->physmem), context(c)
{
    memset(gdbregs, 0, sizeof(gdbregs));
}

RemoteGDB::~RemoteGDB()
{
    if (event)
        delete event;
}

string
RemoteGDB::name()
{
    return system->name() + ".remote_gdb";
}

bool
RemoteGDB::isattached()
{ return attached; }

void
RemoteGDB::attach(int f)
{
    fd = f;

    event = new Event(this, fd, POLLIN);
    pollQueue.schedule(event);

    attached = true;
    DPRINTFN("remote gdb attached\n");
}

void
RemoteGDB::detach()
{
    attached = false;
    close(fd);
    fd = -1;

    pollQueue.remove(event);
    DPRINTFN("remote gdb detached\n");
}

const char *
gdb_command(char cmd)
{
    switch (cmd) {
      case KGDB_SIGNAL: return "KGDB_SIGNAL";
      case KGDB_SET_BAUD: return "KGDB_SET_BAUD";
      case KGDB_SET_BREAK: return "KGDB_SET_BREAK";
      case KGDB_CONT: return "KGDB_CONT";
      case KGDB_ASYNC_CONT: return "KGDB_ASYNC_CONT";
      case KGDB_DEBUG: return "KGDB_DEBUG";
      case KGDB_DETACH: return "KGDB_DETACH";
      case KGDB_REG_R: return "KGDB_REG_R";
      case KGDB_REG_W: return "KGDB_REG_W";
      case KGDB_SET_THREAD: return "KGDB_SET_THREAD";
      case KGDB_CYCLE_STEP: return "KGDB_CYCLE_STEP";
      case KGDB_SIG_CYCLE_STEP: return "KGDB_SIG_CYCLE_STEP";
      case KGDB_KILL: return "KGDB_KILL";
      case KGDB_MEM_W: return "KGDB_MEM_W";
      case KGDB_MEM_R: return "KGDB_MEM_R";
      case KGDB_SET_REG: return "KGDB_SET_REG";
      case KGDB_READ_REG: return "KGDB_READ_REG";
      case KGDB_QUERY_VAR: return "KGDB_QUERY_VAR";
      case KGDB_SET_VAR: return "KGDB_SET_VAR";
      case KGDB_RESET: return "KGDB_RESET";
      case KGDB_STEP: return "KGDB_STEP";
      case KGDB_ASYNC_STEP: return "KGDB_ASYNC_STEP";
      case KGDB_THREAD_ALIVE: return "KGDB_THREAD_ALIVE";
      case KGDB_TARGET_EXIT: return "KGDB_TARGET_EXIT";
      case KGDB_BINARY_DLOAD: return "KGDB_BINARY_DLOAD";
      case KGDB_CLR_HW_BKPT: return "KGDB_CLR_HW_BKPT";
      case KGDB_SET_HW_BKPT: return "KGDB_SET_HW_BKPT";
      case KGDB_START: return "KGDB_START";
      case KGDB_END: return "KGDB_END";
      case KGDB_GOODP: return "KGDB_GOODP";
      case KGDB_BADP: return "KGDB_BADP";
      default: return "KGDB_UNKNOWN";
    }
}

///////////////////////////////////////////////////////////
// RemoteGDB::acc
//
//	Determine if the mapping at va..(va+len) is valid.
//
bool
RemoteGDB::acc(Addr va, size_t len)
{
    Addr last_va;

    va = TheISA::TruncPage(va);
    last_va = TheISA::RoundPage(va + len);

    do  {
        if (TheISA::IsK0Seg(va)) {
            if (va < (TheISA::K0SegBase + pmem->size())) {
                DPRINTF(GDBAcc, "acc:   Mapping is valid  K0SEG <= "
                        "%#x < K0SEG + size\n", va);
                return true;
            } else {
                DPRINTF(GDBAcc, "acc:   Mapping invalid %#x > K0SEG + size\n",
                        va);
                return false;
            }
        }

    /**
     * This code says that all accesses to palcode (instruction and data)
     * are valid since there isn't a va->pa mapping because palcode is
     * accessed physically. At some point this should probably be cleaned up
     * but there is no easy way to do it.
     */

        if (AlphaISA::PcPAL(va) || va < 0x10000)
            return true;

        Addr ptbr = context->regs.ipr[AlphaISA::IPR_PALtemp20];
        TheISA::PageTableEntry pte = kernel_pte_lookup(pmem, ptbr, va);
        if (!pte.valid()) {
            DPRINTF(GDBAcc, "acc:   %#x pte is invalid\n", va);
            return false;
        }
        va += TheISA::PageBytes;
    } while (va < last_va);

    DPRINTF(GDBAcc, "acc:   %#x mapping is valid\n", va);
    return true;
}

///////////////////////////////////////////////////////////
// RemoteGDB::signal
//
//	Translate a trap number into a Unix-compatible signal number.
//	(GDB only understands Unix signal numbers.)
//
int
RemoteGDB::signal(int type)
{
    switch (type) {
      case ALPHA_KENTRY_INT:
        return (SIGTRAP);

      case ALPHA_KENTRY_UNA:
        return (SIGBUS);

      case ALPHA_KENTRY_ARITH:
        return (SIGFPE);

      case ALPHA_KENTRY_IF:
        return (SIGILL);

      case ALPHA_KENTRY_MM:
        return (SIGSEGV);

      default:
        panic("unknown signal type");
        return 0;
    }
}

///////////////////////////////////////////////////////////
// RemoteGDB::getregs
//
//	Translate the kernel debugger register format into
//	the GDB register format.
void
RemoteGDB::getregs()
{
    memset(gdbregs, 0, sizeof(gdbregs));
    memcpy(&gdbregs[KGDB_REG_V0], context->regs.intRegFile, 32 * sizeof(uint64_t));
#ifdef KGDB_FP_REGS
    memcpy(&gdbregs[KGDB_REG_F0], context->regs.floatRegFile.q,
           32 * sizeof(uint64_t));
#endif
    gdbregs[KGDB_REG_PC] = context->regs.pc;
}

///////////////////////////////////////////////////////////
// RemoteGDB::setregs
//
//	Translate the GDB register format into the kernel
//	debugger register format.
//
void
RemoteGDB::setregs()
{
    memcpy(context->regs.intRegFile, &gdbregs[KGDB_REG_V0],
           32 * sizeof(uint64_t));
#ifdef KGDB_FP_REGS
    memcpy(context->regs.floatRegFile.q, &gdbregs[KGDB_REG_F0],
           32 * sizeof(uint64_t));
#endif
    context->regs.pc = gdbregs[KGDB_REG_PC];
}

void
RemoteGDB::setTempBreakpoint(TempBreakpoint &bkpt, Addr addr)
{
    DPRINTF(GDBMisc, "setTempBreakpoint: addr=%#x\n", addr);

    bkpt.address = addr;
    insertHardBreak(addr, 4);
}

void
RemoteGDB::clearTempBreakpoint(TempBreakpoint &bkpt)
{
    DPRINTF(GDBMisc, "setTempBreakpoint: addr=%#x\n",
            bkpt.address);


    removeHardBreak(bkpt.address, 4);
    bkpt.address = 0;
}

void
RemoteGDB::clearSingleStep()
{
    DPRINTF(GDBMisc, "clearSingleStep bt_addr=%#x nt_addr=%#x\n",
            takenBkpt.address, notTakenBkpt.address);

    if (takenBkpt.address != 0)
        clearTempBreakpoint(takenBkpt);

    if (notTakenBkpt.address != 0)
        clearTempBreakpoint(notTakenBkpt);
}

void
RemoteGDB::setSingleStep()
{
    Addr pc = context->regs.pc;
    Addr npc, bpc;
    bool set_bt = false;

    npc = pc + sizeof(MachInst);

    // User was stopped at pc, e.g. the instruction at pc was not
    // executed.
    MachInst inst = read<MachInst>(pc);
    StaticInstPtr<TheISA> si(inst);
    if (si->hasBranchTarget(pc, context, bpc)) {
        // Don't bother setting a breakpoint on the taken branch if it
        // is the same as the next pc
        if (bpc != npc)
            set_bt = true;
    }

    DPRINTF(GDBMisc, "setSingleStep bt_addr=%#x nt_addr=%#x\n",
            takenBkpt.address, notTakenBkpt.address);

    setTempBreakpoint(notTakenBkpt, npc);

    if (set_bt)
        setTempBreakpoint(takenBkpt, bpc);
}

/////////////////////////
//
//

uint8_t
RemoteGDB::getbyte()
{
    uint8_t b;
    ::read(fd, &b, 1);
    return b;
}

void
RemoteGDB::putbyte(uint8_t b)
{
    ::write(fd, &b, 1);
}

// Send a packet to gdb
void
RemoteGDB::send(const char *bp)
{
    const char *p;
    uint8_t csum, c;

    DPRINTF(GDBSend, "send:  %s\n", bp);

    do {
        p = bp;
        putbyte(KGDB_START);
        for (csum = 0; (c = *p); p++) {
            putbyte(c);
            csum += c;
        }
        putbyte(KGDB_END);
        putbyte(i2digit(csum >> 4));
        putbyte(i2digit(csum));
    } while ((c = getbyte() & 0x7f) == KGDB_BADP);
}

// Receive a packet from gdb
int
RemoteGDB::recv(char *bp, int maxlen)
{
    char *p;
    int c, csum;
    int len;

    do {
        p = bp;
        csum = len = 0;
        while ((c = getbyte()) != KGDB_START)
            ;

        while ((c = getbyte()) != KGDB_END && len < maxlen) {
            c &= 0x7f;
            csum += c;
            *p++ = c;
            len++;
        }
        csum &= 0xff;
        *p = '\0';

        if (len >= maxlen) {
            putbyte(KGDB_BADP);
            continue;
        }

        csum -= digit2i(getbyte()) * 16;
        csum -= digit2i(getbyte());

        if (csum == 0) {
            putbyte(KGDB_GOODP);
            // Sequence present?
            if (bp[2] == ':') {
                putbyte(bp[0]);
                putbyte(bp[1]);
                len -= 3;
                bcopy(bp + 3, bp, len);
            }
            break;
        }
        putbyte(KGDB_BADP);
    } while (1);

    DPRINTF(GDBRecv, "recv:  %s: %s\n", gdb_command(*bp), bp);

    return (len);
}

// Read bytes from kernel address space for debugger.
bool
RemoteGDB::read(Addr vaddr, size_t size, char *data)
{
    static Addr lastaddr = 0;
    static size_t lastsize = 0;

    uint8_t *maddr;

    if (vaddr < 10) {
      DPRINTF(GDBRead, "read:  reading memory location zero!\n");
      vaddr = lastaddr + lastsize;
    }

    DPRINTF(GDBRead, "read:  addr=%#x, size=%d", vaddr, size);
#if TRACING_ON
    char *d = data;
    size_t s = size;
#endif

    lastaddr = vaddr;
    lastsize = size;

    size_t count = min((Addr)size,
                       VMPageSize - (vaddr & (VMPageSize - 1)));

    maddr = vtomem(context, vaddr, count);
    memcpy(data, maddr, count);

    vaddr += count;
    data += count;
    size -= count;

    while (size >= VMPageSize) {
        maddr = vtomem(context, vaddr, count);
        memcpy(data, maddr, VMPageSize);

        vaddr += VMPageSize;
        data += VMPageSize;
        size -= VMPageSize;
    }

    if (size > 0) {
        maddr = vtomem(context, vaddr, count);
        memcpy(data, maddr, size);
    }

#if TRACING_ON
    if (DTRACE(GDBRead)) {
        if (DTRACE(GDBExtra)) {
            char buf[1024];
            mem2hex(buf, d, s);
            DPRINTFNR(": %s\n", buf);
        } else
            DPRINTFNR("\n");
    }
#endif

    return true;
}

// Write bytes to kernel address space for debugger.
bool
RemoteGDB::write(Addr vaddr, size_t size, const char *data)
{
    static Addr lastaddr = 0;
    static size_t lastsize = 0;

    uint8_t *maddr;

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

    lastaddr = vaddr;
    lastsize = size;

    size_t count = min((Addr)size,
                       VMPageSize - (vaddr & (VMPageSize - 1)));

    maddr = vtomem(context, vaddr, count);
    memcpy(maddr, data, count);

    vaddr += count;
    data += count;
    size -= count;

    while (size >= VMPageSize) {
        maddr = vtomem(context, vaddr, count);
        memcpy(maddr, data, VMPageSize);

        vaddr += VMPageSize;
        data += VMPageSize;
        size -= VMPageSize;
    }

    if (size > 0) {
        maddr = vtomem(context, vaddr, count);
        memcpy(maddr, data, size);
    }

#ifdef IMB
    alpha_pal_imb();
#endif

    return true;
}


PCEventQueue *RemoteGDB::getPcEventQueue()
{
    return &system->pcEventQueue;
}


RemoteGDB::HardBreakpoint::HardBreakpoint(RemoteGDB *_gdb, Addr pc)
    : PCEvent(_gdb->getPcEventQueue(), "HardBreakpoint Event", pc),
      gdb(_gdb), refcount(0)
{
    DPRINTF(GDBMisc, "creating hardware breakpoint at %#x\n", evpc);
}

void
RemoteGDB::HardBreakpoint::process(ExecContext *xc)
{
    DPRINTF(GDBMisc, "handling hardware breakpoint at %#x\n", pc());

    if (xc == gdb->context)
        gdb->trap(ALPHA_KENTRY_INT);
}

bool
RemoteGDB::insertSoftBreak(Addr addr, size_t len)
{
    if (len != sizeof(MachInst))
        panic("invalid length\n");

    return insertHardBreak(addr, len);
}

bool
RemoteGDB::removeSoftBreak(Addr addr, size_t len)
{
    if (len != sizeof(MachInst))
        panic("invalid length\n");

    return removeHardBreak(addr, len);
}

bool
RemoteGDB::insertHardBreak(Addr addr, size_t len)
{
    if (len != sizeof(MachInst))
        panic("invalid length\n");

    DPRINTF(GDBMisc, "inserting hardware breakpoint at %#x\n", addr);

    HardBreakpoint *&bkpt = hardBreakMap[addr];
    if (bkpt == 0)
        bkpt = new HardBreakpoint(this, addr);

    bkpt->refcount++;

    return true;
}

bool
RemoteGDB::removeHardBreak(Addr addr, size_t len)
{
    if (len != sizeof(MachInst))
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

const char *
break_type(char c)
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
RemoteGDB::trap(int type)
{
    uint64_t val;
    size_t datalen, len;
    char data[KGDB_BUFLEN + 1];
    char buffer[sizeof(gdbregs) * 2 + 256];
    char temp[KGDB_BUFLEN];
    const char *p;
    char command, subcmd;
    string var;
    bool ret;

    if (!attached)
        return false;

    DPRINTF(GDBMisc, "trap: PC=%#x NPC=%#x\n",
            context->regs.pc, context->regs.npc);

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
    if (!active)
        active = true;
    else
        // Tell remote host that an exception has occurred.
        snprintf((char *)buffer, sizeof(buffer), "S%02x", signal(type));
        send(buffer);

    // Stick frame regs into our reg cache.
    getregs();

    for (;;) {
        datalen = recv(data, sizeof(data));
        data[sizeof(data) - 1] = 0; // Sentinel
        command = data[0];
        subcmd = 0;
        p = data + 1;
        switch (command) {

          case KGDB_SIGNAL:
            // if this command came from a running gdb, answer it --
            // the other guy has no way of knowing if we're in or out
            // of this loop when he issues a "remote-signal".
            snprintf((char *)buffer, sizeof(buffer), "S%02x", signal(type));
            send(buffer);
            continue;

          case KGDB_REG_R:
            if (2 * sizeof(gdbregs) > sizeof(buffer))
                panic("buffer too small");

            mem2hex(buffer, gdbregs, sizeof(gdbregs));
            send(buffer);
            continue;

          case KGDB_REG_W:
            p = hex2mem(gdbregs, p, sizeof(gdbregs));
            if (p == NULL || *p != '\0')
                send("E01");
            else {
                setregs();
                send("OK");
            }
            continue;

#if 0
          case KGDB_SET_REG:
            val = hex2i(&p);
            if (*p++ != '=') {
                send("E01");
                continue;
            }
            if (val < 0 && val >= KGDB_NUMREGS) {
                send("E01");
                continue;
            }

            gdbregs[val] = hex2i(&p);
            setregs();
            send("OK");

            continue;
#endif

          case KGDB_MEM_R:
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
            if (len > sizeof(buffer)) {
                send("E04");
                continue;
            }
            if (!acc(val, len)) {
                send("E05");
                continue;
            }

            if (read(val, (size_t)len, (char *)buffer)) {
              mem2hex(temp, buffer, len);
              send(temp);
            } else {
              send("E05");
            }
            continue;

          case KGDB_MEM_W:
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
            p = hex2mem(buffer, p, sizeof(buffer));
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

          case KGDB_SET_THREAD:
            subcmd = *p++;
            val = hex2i(&p);
            if (val == 0)
                send("OK");
            else
                send("E01");
            continue;

          case KGDB_DETACH:
          case KGDB_KILL:
            active = false;
            clearSingleStep();
            detach();
            goto out;

          case KGDB_ASYNC_CONT:
            subcmd = hex2i(&p);
            if (*p++ == ';') {
                val = hex2i(&p);
                context->regs.pc = val;
                context->regs.npc = val + sizeof(MachInst);
            }
            clearSingleStep();
            goto out;

          case KGDB_CONT:
            if (p - data < datalen) {
                val = hex2i(&p);
                context->regs.pc = val;
                context->regs.npc = val + sizeof(MachInst);
            }
            clearSingleStep();
            goto out;

          case KGDB_ASYNC_STEP:
            subcmd = hex2i(&p);
            if (*p++ == ';') {
                val = hex2i(&p);
                context->regs.pc = val;
                context->regs.npc = val + sizeof(MachInst);
            }
            setSingleStep();
            goto out;

          case KGDB_STEP:
            if (p - data < datalen) {
                val = hex2i(&p);
                context->regs.pc = val;
                context->regs.npc = val + sizeof(MachInst);
            }
            setSingleStep();
            goto out;

          case KGDB_CLR_HW_BKPT:
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

          case KGDB_SET_HW_BKPT:
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

          case KGDB_QUERY_VAR:
            var = string(p, datalen - 1);
            if (var == "C")
                send("QC0");
            else
                send("");
            continue;

          case KGDB_SET_BAUD:
          case KGDB_SET_BREAK:
          case KGDB_DEBUG:
          case KGDB_CYCLE_STEP:
          case KGDB_SIG_CYCLE_STEP:
          case KGDB_READ_REG:
          case KGDB_SET_VAR:
          case KGDB_RESET:
          case KGDB_THREAD_ALIVE:
          case KGDB_TARGET_EXIT:
          case KGDB_BINARY_DLOAD:
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
    return true;
}

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
mem2hex(void *vdst, const void *vsrc, int len)
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
hex2mem(void *vdst, const char *src, int maxlen)
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
    return (r);
}

