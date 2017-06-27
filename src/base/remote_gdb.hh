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

#ifndef __REMOTE_GDB_HH__
#define __REMOTE_GDB_HH__

#include <sys/signal.h>

#include <exception>
#include <map>
#include <string>

#include "arch/types.hh"
#include "base/intmath.hh"
#include "base/pollevent.hh"
#include "base/socket.hh"
#include "cpu/pc_event.hh"

class System;
class ThreadContext;

class GDBListener;

class BaseRemoteGDB;

struct GdbCommand
{
  public:
    struct Context
    {
        const GdbCommand *cmd;
        char cmd_byte;
        int type;
        char *data;
        int len;
    };

    typedef bool (BaseRemoteGDB::*Func)(Context &ctx);

    const char * const name;
    const Func func;

    GdbCommand(const char *_name, Func _func) : name(_name), func(_func)
    {}
};

class BaseRemoteGDB
{
  private:
    friend void debugger();
    friend class GDBListener;

  protected:
    /// Exception to throw when the connection to the client is broken.
    struct BadClient
    {
        const char *warning;
        BadClient(const char *_warning=NULL) : warning(_warning)
        {}
    };
    /// Exception to throw when an error needs to be reported to the client.
    struct CmdError
    {
        std::string error;
        CmdError(std::string _error) : error(_error)
        {}
    };
    /// Exception to throw when something isn't supported.
    class Unsupported {};

    // Helper functions
  protected:
    int digit2i(char);
    char i2digit(int);
    Addr hex2i(const char **);
    // Address formats, break types, and gdb commands may change
    // between architectures, so they're defined as virtual
    // functions.
    virtual void mem2hex(char *, const char *, int);
    virtual const char * hex2mem(char *, const char *, int);
    virtual const char * break_type(char c);

  protected:
    static std::map<char, GdbCommand> command_map;

    bool cmd_unsupported(GdbCommand::Context &ctx);

    bool cmd_signal(GdbCommand::Context &ctx);
    bool cmd_cont(GdbCommand::Context &ctx);
    bool cmd_async_cont(GdbCommand::Context &ctx);
    bool cmd_detach(GdbCommand::Context &ctx);
    bool cmd_reg_r(GdbCommand::Context &ctx);
    bool cmd_reg_w(GdbCommand::Context &ctx);
    bool cmd_set_thread(GdbCommand::Context &ctx);
    bool cmd_mem_r(GdbCommand::Context &ctx);
    bool cmd_mem_w(GdbCommand::Context &ctx);
    bool cmd_query_var(GdbCommand::Context &ctx);
    bool cmd_step(GdbCommand::Context &ctx);
    bool cmd_async_step(GdbCommand::Context &ctx);
    bool cmd_clr_hw_bkpt(GdbCommand::Context &ctx);
    bool cmd_set_hw_bkpt(GdbCommand::Context &ctx);

  protected:
    class InputEvent : public PollEvent
    {
      protected:
        BaseRemoteGDB *gdb;

      public:
        InputEvent(BaseRemoteGDB *g, int fd, int e);
        void process(int revent);
    };

    class TrapEvent : public Event
    {
      protected:
        int _type;
        BaseRemoteGDB *gdb;

      public:
        TrapEvent(BaseRemoteGDB *g) : gdb(g)
        {}

        void type(int t) { _type = t; }
        void process();
    };

    friend class InputEvent;
    InputEvent *inputEvent;
    TrapEvent trapEvent;
    GDBListener *listener;
    int number;

  protected:
    // The socket commands come in through
    int fd;

  protected:
    bool active;
    bool attached;

    System *system;
    ThreadContext *context;

  protected:
    /**
     * Concrete subclasses of this abstract class represent how the
     * register values are transmitted on the wire.  Usually each
     * architecture should define one subclass, but there can be more
     * if there is more than one possible wire format.  For example,
     * ARM defines both AArch32GdbRegCache and AArch64GdbRegCache.
     */
    class BaseGdbRegCache
    {
      public:

        /**
         * Return the pointer to the raw bytes buffer containing the
         * register values.  Each byte of this buffer is literally
         * encoded as two hex digits in the g or G RSP packet.
         */
        virtual char *data() const = 0;

        /**
         * Return the size of the raw buffer, in bytes
         * (i.e., half of the number of digits in the g/G packet).
         */
        virtual size_t size() const = 0;

        /**
         * Fill the raw buffer from the registers in the ThreadContext.
         */
        virtual void getRegs(ThreadContext*) = 0;

        /**
         * Set the ThreadContext's registers from the values
         * in the raw buffer.
         */
        virtual void setRegs(ThreadContext*) const = 0;

        /**
         * Return the name to use in places like DPRINTF.
         * Having each concrete superclass redefine this member
         * is useful in situations where the class of the regCache
         * can change on the fly.
         */
        virtual const std::string name() const = 0;

        BaseGdbRegCache(BaseRemoteGDB *g) : gdb(g)
        {}
        virtual ~BaseGdbRegCache()
        {}

      protected:
        BaseRemoteGDB *gdb;
    };

    BaseGdbRegCache *regCachePtr;

  protected:
    uint8_t getbyte();
    void putbyte(uint8_t b);

    int recv(char *data, int len);
    void send(const char *data);

  protected:
    // Machine memory
    virtual bool read(Addr addr, size_t size, char *data);
    virtual bool write(Addr addr, size_t size, const char *data);

    template <class T> T read(Addr addr);
    template <class T> void write(Addr addr, T data);

  public:
    BaseRemoteGDB(System *system, ThreadContext *context);
    virtual ~BaseRemoteGDB();
    virtual BaseGdbRegCache *gdbRegs() = 0;

    void replaceThreadContext(ThreadContext *tc) { context = tc; }

    void attach(int fd);
    void detach();
    bool isattached();

    virtual bool acc(Addr addr, size_t len) = 0;
    bool trap(int type);
    virtual bool breakpoint()
    {
        return trap(SIGTRAP);
    }

    void processSingleStepEvent();
    EventFunctionWrapper singleStepEvent;

    void clearSingleStep();
    void setSingleStep();

    PCEventQueue *getPcEventQueue();
    EventQueue *getComInstEventQueue();

    /// Schedule an event which will be triggered "delta" instructions later.
    void scheduleInstCommitEvent(Event *ev, int delta);
    /// Deschedule an instruction count based event.
    void descheduleInstCommitEvent(Event *ev);

  protected:
    virtual bool checkBpLen(size_t len);

    class HardBreakpoint : public PCEvent
    {
      private:
        BaseRemoteGDB *gdb;

      public:
        int refcount;

      public:
        HardBreakpoint(BaseRemoteGDB *_gdb, Addr addr);
        const std::string name() const { return gdb->name() + ".hwbkpt"; }

        virtual void process(ThreadContext *tc);
    };
    friend class HardBreakpoint;

    typedef std::map<Addr, HardBreakpoint *> break_map_t;
    typedef break_map_t::iterator break_iter_t;
    break_map_t hardBreakMap;

    void insertSoftBreak(Addr addr, size_t len);
    void removeSoftBreak(Addr addr, size_t len);
    virtual void insertHardBreak(Addr addr, size_t len);
    void removeHardBreak(Addr addr, size_t len);

  protected:
    void clearTempBreakpoint(Addr &bkpt);
    void setTempBreakpoint(Addr bkpt);

  public:
    std::string name();
};

template <class T>
inline T
BaseRemoteGDB::read(Addr addr)
{
    T temp;
    read(addr, sizeof(T), (char *)&temp);
    return temp;
}

template <class T>
inline void
BaseRemoteGDB::write(Addr addr, T data)
{ write(addr, sizeof(T), (const char *)&data); }

class GDBListener
{
  protected:
    class InputEvent : public PollEvent
    {
      protected:
        GDBListener *listener;

      public:
        InputEvent(GDBListener *l, int fd, int e);
        void process(int revent);
    };

    friend class InputEvent;
    InputEvent *inputEvent;

  protected:
    ListenSocket listener;
    BaseRemoteGDB *gdb;
    int port;

  public:
    GDBListener(BaseRemoteGDB *g, int p);
    ~GDBListener();

    void accept();
    void listen();
    std::string name();

    int getPort() const;
};

#endif /* __REMOTE_GDB_H__ */
