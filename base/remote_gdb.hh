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

#ifndef __REMOTE_GDB_HH__
#define __REMOTE_GDB_HH__

#include <map>

#include "base/kgdb.h"
#include "cpu/pc_event.hh"
#include "base/pollevent.hh"
#include "base/socket.hh"

class System;
class ExecContext;
class PhysicalMemory;

class GDBListener;
class RemoteGDB
{
  private:
    friend void debugger();
    friend class GDBListener;

  protected:
    class Event : public PollEvent
    {
      protected:
        RemoteGDB *gdb;

      public:
        Event(RemoteGDB *g, int fd, int e);
        void process(int revent);
    };

    friend class Event;
    Event *event;
    GDBListener *listener;
    int number;

  protected:
    int fd;
    uint64_t gdbregs[KGDB_NUMREGS];

  protected:
#ifdef notyet
    label_t recover;
#endif
    bool active;
    bool attached;

    System *system;
    PhysicalMemory *pmem;
    ExecContext *context;

  protected:
    uint8_t getbyte();
    void putbyte(uint8_t b);

    int recv(char *data, int len);
    void send(const char *data);

  protected:
    // Machine memory
    bool read(Addr addr, size_t size, char *data);
    bool write(Addr addr, size_t size, const char *data);

    template <class T> T read(Addr addr);
    template <class T> void write(Addr addr, T data);

  public:
    RemoteGDB(System *system, ExecContext *context);
    ~RemoteGDB();

    void replaceExecContext(ExecContext *xc) { context = xc; }

    void attach(int fd);
    void detach();
    bool isattached();

    bool acc(Addr addr, size_t len);
    static int signal(int type);
    bool trap(int type);

  protected:
    void getregs();
    void setregs();

    void clearSingleStep();
    void setSingleStep();

    PCEventQueue *getPcEventQueue();

  protected:
    class HardBreakpoint : public PCEvent
    {
      private:
        RemoteGDB *gdb;

      public:
        int refcount;

      public:
        HardBreakpoint(RemoteGDB *_gdb, Addr addr);
        std::string name() { return gdb->name() + ".hwbkpt"; }

        virtual void process(ExecContext *xc);
    };
    friend class HardBreakpoint;

    typedef std::map<Addr, HardBreakpoint *> break_map_t;
    typedef break_map_t::iterator break_iter_t;
    break_map_t hardBreakMap;

    bool insertSoftBreak(Addr addr, size_t len);
    bool removeSoftBreak(Addr addr, size_t len);
    bool insertHardBreak(Addr addr, size_t len);
    bool removeHardBreak(Addr addr, size_t len);

  protected:
    struct TempBreakpoint {
        Addr	address;		// set here
        MachInst	bkpt_inst;		// saved instruction at bkpt
        int		init_count;		// number of times to skip bkpt
        int		count;			// current count
    };

    TempBreakpoint notTakenBkpt;
    TempBreakpoint takenBkpt;

    void clearTempBreakpoint(TempBreakpoint &bkpt);
    void setTempBreakpoint(TempBreakpoint &bkpt, Addr addr);

  public:
    std::string name();
};

template <class T>
inline T
RemoteGDB::read(Addr addr)
{
    T temp;
    read(addr, sizeof(T), (char *)&temp);
    return temp;
}

template <class T>
inline void
RemoteGDB::write(Addr addr, T data)
{ write(addr, sizeof(T), (const char *)&data); }

class GDBListener
{
  protected:
    class Event : public PollEvent
    {
      protected:
        GDBListener *listener;

      public:
        Event(GDBListener *l, int fd, int e);
        void process(int revent);
    };

    friend class Event;
    Event *event;

  protected:
    ListenSocket listener;
    RemoteGDB *gdb;
    int port;

  public:
    GDBListener(RemoteGDB *g, int p);
    ~GDBListener();

    void accept();
    void listen();
    std::string name();
};

#endif /* __REMOTE_GDB_H__ */
