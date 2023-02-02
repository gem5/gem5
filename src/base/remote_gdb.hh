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

#ifndef __REMOTE_GDB_HH__
#define __REMOTE_GDB_HH__


#include <cstdint>
#include <exception>
#include <map>
#include <string>
#include <vector>

#include "arch/generic/pcstate.hh"
#include "base/cprintf.hh"
#include "base/pollevent.hh"
#include "base/socket.hh"
#include "base/types.hh"
#include "cpu/pc_event.hh"
#include "gdbremote/signals.hh"
#include "sim/debug.hh"
#include "sim/eventq.hh"

/*
 * This file implements a client for the GDB remote serial protocol as
 * described in this official documentation:
 *
 * https://sourceware.org/gdb/current/onlinedocs/gdb/Remote-Protocol.html
 */

namespace gem5
{

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
     *
     * @ingroup api_remote_gdb
     */
    virtual char *data() const = 0;

    /**
     * Return the size of the raw buffer, in bytes
     * (i.e., half of the number of digits in the g/G packet).
     *
     * @ingroup api_remote_gdb
     */
    virtual size_t size() const = 0;

    /**
     * Fill the raw buffer from the registers in the ThreadContext.
     *
     * @ingroup api_remote_gdb
     */
    virtual void getRegs(ThreadContext*) = 0;

    /**
     * Set the ThreadContext's registers from the values
     * in the raw buffer.
     *
     * @ingroup api_remote_gdb
     */
    virtual void setRegs(ThreadContext*) const = 0;

    /**
     * Return the name to use in places like DPRINTF.
     * Having each concrete superclass redefine this member
     * is useful in situations where the class of the regCache
     * can change on the fly.
     *
     * @ingroup api_remote_gdb
     */
    virtual const std::string name() const = 0;

    /**
     * @ingroup api_remote_gdb
     */
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

    /**
     * @ingroup api_remote_gdb
     * @{
     */

    /**
     * Interface to other parts of the simulator.
     */
    BaseRemoteGDB(System *system, int _port);
    virtual ~BaseRemoteGDB();

    std::string name();

    void listen();
    void connect();

    int port() const;

    void attach(int fd);
    void detach();
    bool isAttached() { return attached; }

    void addThreadContext(ThreadContext *_tc);
    void replaceThreadContext(ThreadContext *_tc);
    bool selectThreadContext(ContextID id);

    void trap(ContextID id, GDBSignal sig,const std::string& stopReason="");
    bool sendMessage(std::string message);
    //schedule a trap event with these properties
    void scheduleTrapEvent(ContextID id,GDBSignal type, int delta,
      std::string stopReason);
    /** @} */ // end of api_remote_gdb

    template <class GDBStub, class ...Args>
    static BaseRemoteGDB *
    build(int port, Args... args)
    {
        if (port)
            return new GDBStub(args..., port);
        else
            return nullptr;
    }

  private:
    /*
     * Connection to the external GDB.
     */

    /*
     * Asynchronous socket events and event handlers.
     *
     * These events occur asynchronously and are handled asynchronously
     * to main simulation loop - therefore they *shall not* interact with
     * rest of gem5.
     *
     * The only thing they do is to schedule a synchronous event at instruction
     * boundary to deal with the request.
     */
    void incomingData(int revent);
    void incomingConnection(int revent);

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

    typedef SocketEvent<&BaseRemoteGDB::incomingConnection>
        IncomingConnectionEvent;
    typedef SocketEvent<&BaseRemoteGDB::incomingData>
        IncomingDataEvent;

    friend IncomingConnectionEvent;
    friend IncomingDataEvent;

    IncomingConnectionEvent *incomingConnectionEvent;
    IncomingDataEvent *incomingDataEvent;

    ListenSocket listener;
    int _port;

    // The socket commands come in through.
    int fd;

    // Transfer data to/from GDB.
    uint8_t getbyte();
    bool try_getbyte(uint8_t* c,int timeout=-1);//return true if successful
    void putbyte(uint8_t b);

    void recv(std::vector<char> &bp);
    void send(const char *data);
    void send(const std::string &data) { send(data.c_str()); }

    template <typename ...Args>
    void
    send(const char *format, const Args &...args)
    {
        send(csprintf(format, args...));
    }

    /*
     * Process commands from remote GDB. If simulation has been
     * stopped because of some kind of fault (as segmentation violation,
     * or SW trap), 'signum' is the signal value reported back to GDB
     * in "S" packet (this is done in trap()).
     */
    void processCommands(GDBSignal sig=GDBSignal::ZERO);

    /*
     * Simulator side debugger state.
     */
    bool attached = false;
    bool threadSwitching = false;

    System *sys;

    std::map<ContextID, ThreadContext *> threads;
    ThreadContext *tc = nullptr;

    BaseGdbRegCache *regCachePtr = nullptr;

    MemberEventWrapper<&BaseRemoteGDB::connect> connectEvent;
    MemberEventWrapper<&BaseRemoteGDB::detach>  disconnectEvent;

    class TrapEvent : public Event
    {
      protected:
        GDBSignal _type;
        ContextID _id;
        std::string _stopReason;
        BaseRemoteGDB *gdb;

      public:
        TrapEvent(BaseRemoteGDB *g) : gdb(g)
        {}

        void type(GDBSignal t) { _type = t; }
        void stopReason(std::string s) {_stopReason = s; }
        void id(ContextID id) { _id = id; }
         void process() { gdb->trap(_id, _type,_stopReason); }
    } trapEvent;

    /*
     * The interface to the simulated system.
     */
    virtual bool readBlob(Addr vaddr, size_t size, char *data);
    virtual bool writeBlob(Addr vaddr, size_t size, const char *data);
    bool read(Addr vaddr, size_t size, char *data);
    bool write(Addr vaddr, size_t size, const char *data);

    template <class T> T read(Addr addr);
    template <class T> void write(Addr addr, T data);

    // Single step.
    void singleStep();
    MemberEventWrapper<&BaseRemoteGDB::singleStep> singleStepEvent;

    void clearSingleStep();
    void setSingleStep();

    /// Schedule an event which will be triggered "delta" instructions later.
    void scheduleInstCommitEvent(Event *ev, int delta,ThreadContext* _tc);
    void scheduleInstCommitEvent(Event *ev, int delta){
       scheduleInstCommitEvent(ev, delta,tc);
    };
    /// Deschedule an instruction count based event.
    void descheduleInstCommitEvent(Event *ev);

    // Breakpoints.
    void insertSoftBreak(Addr addr, size_t kind);
    void removeSoftBreak(Addr addr, size_t kind);
    void insertHardBreak(Addr addr, size_t kind);
    void removeHardBreak(Addr addr, size_t kind);

    void sendTPacket(GDBSignal sig, ContextID id,
      const std::string& stopReason);
    void sendSPacket(GDBSignal sig);
    //The OPacket allow to send string to be displayed by the remote GDB
    void sendOPacket(const std::string message);
    /*
     * GDB commands.
     */
    struct GdbCommand
    {
      public:
        struct Context
        {
            const GdbCommand *cmd;
            char cmdByte;
            GDBSignal type;
            char *data;
            int len;
        };

        typedef bool (BaseRemoteGDB::*Func)(Context &ctx);

        const char * const name;
        const Func func;

        GdbCommand(const char *_name, Func _func) : name(_name), func(_func) {}
    };

    static std::map<char, GdbCommand> commandMap;

    struct GdbMultiLetterCommand
    {
      public:
        struct Context
        {
            const GdbMultiLetterCommand *cmd;
            std::string cmdTxt;
            GDBSignal type;
            char *data;
            int len;
        };

        typedef bool (BaseRemoteGDB::*Func)(Context &ctx);

        const char * const name;
        const Func func;

        GdbMultiLetterCommand(const char *_name, Func _func) :
          name(_name), func(_func) {}
    };


    static std::map<std::string, GdbMultiLetterCommand> multiLetterMap;

    bool cmdUnsupported(GdbCommand::Context &ctx);

    bool cmdSignal(GdbCommand::Context &ctx);
    bool cmdCont(GdbCommand::Context &ctx);
    bool cmdAsyncCont(GdbCommand::Context &ctx);
    bool cmdDetach(GdbCommand::Context &ctx);
    bool cmdRegR(GdbCommand::Context &ctx);
    bool cmdRegW(GdbCommand::Context &ctx);
    bool cmdSetThread(GdbCommand::Context &ctx);
    bool cmdIsThreadAlive(GdbCommand::Context &ctx);
    bool cmdMemR(GdbCommand::Context &ctx);
    bool cmdMemW(GdbCommand::Context &ctx);
    bool cmdQueryVar(GdbCommand::Context &ctx);
    bool cmdStep(GdbCommand::Context &ctx);
    bool cmdAsyncStep(GdbCommand::Context &ctx);
    bool cmdClrHwBkpt(GdbCommand::Context &ctx);
    bool cmdSetHwBkpt(GdbCommand::Context &ctx);
    bool cmdDumpPageTable(GdbCommand::Context &ctx);
    bool cmdMultiLetter(GdbCommand::Context &ctx);

    //Multi letter command
    bool cmdMultiUnsupported(GdbMultiLetterCommand::Context &ctx);

    bool cmdReplyEmpty(GdbMultiLetterCommand::Context &ctx);
    bool cmdVKill(GdbMultiLetterCommand::Context &ctx);

    struct QuerySetCommand
    {
        struct Context
        {
            const std::string &name;
            std::vector<std::string> args;

            Context(const std::string &_name) : name(_name) {}
        };

        using Func = bool (BaseRemoteGDB::*)(Context &ctx);

        const char * const argSep;
        const Func func;

        QuerySetCommand(Func _func, const char *_argSep=nullptr) :
            argSep(_argSep), func(_func)
        {}
    };

    static std::map<std::string, QuerySetCommand> queryMap;

    bool queryC(QuerySetCommand::Context &ctx);
    bool querySupported(QuerySetCommand::Context &ctx);
    bool queryXfer(QuerySetCommand::Context &ctx);
    bool querySymbol(QuerySetCommand::Context &ctx);
    bool queryRcmd(QuerySetCommand::Context &ctx);
    bool queryAttached(QuerySetCommand::Context &ctx);

    size_t threadInfoIdx = 0;
    bool queryFThreadInfo(QuerySetCommand::Context &ctx);
    bool querySThreadInfo(QuerySetCommand::Context &ctx);

  protected:
    ThreadContext *context() { return tc; }
    System *system() { return sys; }

    void encodeBinaryData(const std::string &unencoded,
            std::string &encoded) const;

    void encodeXferResponse(const std::string &unencoded,
        std::string &encoded, size_t offset, size_t unencoded_length) const;

    // checkBpKind checks if a kind of breakpoint is legal. This function should
    // be implemented by subclasses by arch. The "kind" is considered to be
    // breakpoint size in some arch.
    virtual bool checkBpKind(size_t kind);

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

} // namespace gem5

#endif /* __REMOTE_GDB_H__ */
