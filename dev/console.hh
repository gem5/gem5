/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

/* @file
 * User Console Interface
 */

#ifndef __CONSOLE_HH__
#define __CONSOLE_HH__

#include <iostream>

#include "base/circlebuf.hh"
#include "cpu/intr_control.hh"
#include "base/pollevent.hh"
#include "base/socket.hh"
#include "sim/sim_object.hh"

class ConsoleListener;
class SimConsole : public SimObject
{
  protected:
    class Event : public PollEvent
    {
      protected:
        SimConsole *cons;

      public:
        Event(SimConsole *c, int fd, int e);
        void process(int revent);
    };

    friend class Event;
    Event *event;

  protected:
    int number;
    int in_fd;
    int out_fd;

  protected:
    ConsoleListener *listener;

  public:
    SimConsole(const std::string &name, const std::string &file, int num);
    ~SimConsole();

  protected:
    CircleBuf txbuf;
    CircleBuf rxbuf;
    std::ostream *outfile;

  public:
    ///////////////////////
    // Terminal Interface

    void attach(int fd, ConsoleListener *l = NULL) { attach(fd, fd, l); }
    void attach(int in, int out, ConsoleListener *l = NULL);
    void detach();

    void data();

    void close();
    void read(uint8_t &c) { read(&c, 1); }
    size_t read(uint8_t *buf, size_t len);
    void write(uint8_t c) { write(&c, 1); }
    size_t write(const uint8_t *buf, size_t len);

    void configTerm();

  protected:
    // interrupt status/enable
    int _status;
    int _enable;

    // interrupt handle
    IntrControl *intr;

  public:
    /////////////////
    // OS interface

    // Get a character from the console.
    // return of -1 means there is no character pending.
    // Interrupts are cleared when the buffer is empty.
    int in();

    // Send a character to the console
    void out(char c, bool raise_int = true);

    enum {
        TransmitInterrupt = 1,
        ReceiveInterrupt = 2
    };

    // Read the current interrupt status of this console.
    int intStatus() { return _status; }

    // Set the interrupt enable bits.
    int clearInt(int i);
    void raiseInt(int i);

    void initInt(IntrControl *i);
    void setInt(int bits);

    virtual void serialize();
    virtual void unserialize(IniFile &db, const std::string &category,
                             ConfigNode *node);
};

class ConsoleListener : public SimObject
{
  protected:
    class Event : public PollEvent
    {
      protected:
        ConsoleListener *listener;

      public:
        Event(ConsoleListener *l, int fd, int e)
            : PollEvent(fd, e), listener(l) {}
        void process(int revent);
    };

    friend class Event;
    Event *event;

    typedef std::list<SimConsole *> list_t;
    typedef list_t::iterator iter_t;
    list_t ConsoleList;

  protected:
    ListenSocket listener;

  public:
    ConsoleListener(const std::string &name);
    ~ConsoleListener();

    void add(SimConsole *cons);

    void accept();
    void listen(int port);
};

#endif // __CONSOLE_HH__
