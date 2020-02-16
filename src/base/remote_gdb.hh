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

class BaseRemoteGDB;
class HardBreakpoint;

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

class BaseRemoteGDB
{
    friend class HardBreakpoint;
  public:

    /*
     * Interface to other parts of the simulator.
     */
    BaseRemoteGDB(System *system, ThreadContext *context, int _port);
    virtual ~BaseRemoteGDB();

    std::string name();

    void listen();
    void connect();

    int port() const;

    void attach(int fd);
    void detach();
    bool isAttached() { return attached; }

    void replaceThreadContext(ThreadContext *_tc) { tc = _tc; }

    bool trap(int type);
    bool breakpoint() { return trap(SIGTRAP); }

  private:
    /*
     * Connection to the external GDB.
     */
    void incomingData(int revent);
    void connectWrapper(int revent) { connect(); }

    template <void (BaseRemoteGDB::*F)(int revent)>
    class SocketEvent : public PollEvent
    {
      protected:
        BaseRemoteGDB *gdb;

      public:
        SocketEvent(BaseRemoteGDB *gdb, int fd, int e) :
            PollEvent(fd, e), gdb(gdb)
        {}

        void process(int revent) { (gdb->*F)(revent); }
    };

    typedef SocketEvent<&BaseRemoteGDB::connectWrapper> ConnectEvent;
    typedef SocketEvent<&BaseRemoteGDB::incomingData> DataEvent;

    friend ConnectEvent;
    friend DataEvent;

    ConnectEvent *connectEvent;
    DataEvent *dataEvent;

    ListenSocket listener;
    int _port;

    // The socket commands come in through.
    int fd;

    // Transfer data to/from GDB.
    uint8_t getbyte();
    void putbyte(uint8_t b);

    void recv(std::vector<char> &bp);
    void send(const char *data);

    /*
     * Simulator side debugger state.
     */
    bool active;
    bool attached;

    System *sys;
    ThreadContext *tc;

    BaseGdbRegCache *regCachePtr;

    class TrapEvent : public Event
    {
      protected:
        int _type;
        BaseRemoteGDB *gdb;

      public:
        TrapEvent(BaseRemoteGDB *g) : gdb(g)
        {}

        void type(int t) { _type = t; }
        void process() { gdb->trap(_type); }
    } trapEvent;

    /*
     * The interface to the simulated system.
     */
    // Machine memory.
    bool read(Addr addr, size_t size, char *data);
    bool write(Addr addr, size_t size, const char *data);

    template <class T> T read(Addr addr);
    template <class T> void write(Addr addr, T data);

    // Single step.
    void singleStep();
    EventWrapper<BaseRemoteGDB, &BaseRemoteGDB::singleStep> singleStepEvent;

    void clearSingleStep();
    void setSingleStep();

    /// Schedule an event which will be triggered "delta" instructions later.
    void scheduleInstCommitEvent(Event *ev, int delta);
    /// Deschedule an instruction count based event.
    void descheduleInstCommitEvent(Event *ev);

    // Breakpoints.
    void insertSoftBreak(Addr addr, size_t len);
    void removeSoftBreak(Addr addr, size_t len);
    void insertHardBreak(Addr addr, size_t len);
    void removeHardBreak(Addr addr, size_t len);

    void clearTempBreakpoint(Addr &bkpt);
    void setTempBreakpoint(Addr bkpt);

    /*
     * GDB commands.
     */
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

        GdbCommand(const char *_name, Func _func) : name(_name), func(_func) {}
    };

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
    ThreadContext *context() { return tc; }
    System *system() { return sys; }

    void encodeBinaryData(const std::string &unencoded,
            std::string &encoded) const;

    void encodeXferResponse(const std::string &unencoded,
        std::string &encoded, size_t offset, size_t unencoded_length) const;

    // To be implemented by subclasses.
    virtual bool checkBpLen(size_t len);

    virtual BaseGdbRegCache *gdbRegs() = 0;

    virtual bool acc(Addr addr, size_t len) = 0;

    virtual std::vector<std::string> availableFeatures() const;

    /**
     * Get an XML target description.
     *
     * @param[in] annex the XML filename
     * @param[out] output set to the decoded XML
     * @return true if the given annex was found
     */
    virtual bool getXferFeaturesRead(const std::string &annex,
            std::string &output);
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
{
    write(addr, sizeof(T), (const char *)&data);
}

#endif /* __REMOTE_GDB_H__ */
