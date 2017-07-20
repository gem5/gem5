/*
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
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
 *          Ali Saidi
 */

/* @file
 * User Terminal Interface
 */

#ifndef __DEV_TERMINAL_HH__
#define __DEV_TERMINAL_HH__

#include <iostream>

#include "base/callback.hh"
#include "base/circlebuf.hh"
#include "base/pollevent.hh"
#include "base/socket.hh"
#include "cpu/intr_control.hh"
#include "dev/serial/serial.hh"
#include "params/Terminal.hh"
#include "sim/sim_object.hh"

class OutputStream;
class TerminalListener;

class Terminal : public SerialDevice
{
  protected:
    class ListenEvent : public PollEvent
    {
      protected:
        Terminal *term;

      public:
        ListenEvent(Terminal *t, int fd, int e);
        void process(int revent);
    };

    friend class ListenEvent;
    ListenEvent *listenEvent;

    class DataEvent : public PollEvent
    {
      protected:
        Terminal *term;

      public:
        DataEvent(Terminal *t, int fd, int e);
        void process(int revent);
    };

    friend class DataEvent;
    DataEvent *dataEvent;

  protected:
    int number;
    int data_fd;

  public:
    typedef TerminalParams Params;
    Terminal(const Params *p);
    ~Terminal();

  protected:
    ListenSocket listener;

    void listen(int port);
    void accept();

  protected:
    CircleBuf<char> txbuf;
    CircleBuf<char> rxbuf;
    OutputStream *outfile;
#if TRACING_ON == 1
    CircleBuf<char> linebuf;
#endif

  public:
    ///////////////////////
    // Terminal Interface

    void data();

    void read(uint8_t &c) { read(&c, 1); }
    size_t read(uint8_t *buf, size_t len);
    void write(uint8_t c) { write(&c, 1); }
    size_t write(const uint8_t *buf, size_t len);
    void detach();

  public: // SerialDevice interface
    uint8_t readData() override;
    void writeData(uint8_t c) override;
    bool dataAvailable() const override { return !rxbuf.empty(); }

  public:
    /////////////////
    // OS interface

    // get a character from the terminal in the console specific format
    // corresponds to GETC:
    // retval<63:61>
    //     000: success: character received
    //     001: success: character received, more pending
    //     100: failure: no character ready
    //     110: failure: character received with error
    //     111: failure: character received with error, more pending
    // retval<31:0>
    //     character read from console
    //
    // Interrupts are cleared when the buffer is empty.
    uint64_t console_in();
};

#endif // __DEV_TERMINAL_HH__
