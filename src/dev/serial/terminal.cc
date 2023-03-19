/*
 * Copyright (c) 2019 ARM Limited
 * All rights reserved
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
 */

/* @file
 * Implements the user interface to a serial terminal
 */

#include <sys/ioctl.h>

#if defined(__FreeBSD__)
#include <termios.h>

#else
#include <sys/termios.h>

#endif
#include "dev/serial/terminal.hh"

#include <poll.h>
#include <unistd.h>

#include <cctype>
#include <cerrno>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "base/atomicio.hh"
#include "base/logging.hh"
#include "base/output.hh"
#include "base/socket.hh"
#include "base/trace.hh"
#include "debug/Terminal.hh"
#include "debug/TerminalVerbose.hh"
#include "dev/platform.hh"
#include "dev/serial/uart.hh"

namespace gem5
{

/*
 * Poll event for the listen socket
 */
Terminal::ListenEvent::ListenEvent(Terminal *t, int fd, int e)
    : PollEvent(fd, e), term(t)
{
}

void
Terminal::ListenEvent::process(int revent)
{
    // As a consequence of being called from the PollQueue, we might
    // have been called from a different thread. Migrate to "our"
    // thread.
    EventQueue::ScopedMigration migrate(term->eventQueue());
    term->accept();
}

/*
 * Poll event for the data socket
 */
Terminal::DataEvent::DataEvent(Terminal *t, int fd, int e)
    : PollEvent(fd, e), term(t)
{
}

void
Terminal::DataEvent::process(int revent)
{
    // As a consequence of being called from the PollQueue, we might
    // have been called from a different thread. Migrate to "our"
    // thread.
    EventQueue::ScopedMigration migrate(term->eventQueue());

    if (revent & POLLIN)
        term->data();
    else if (revent & POLLNVAL)
        term->detach();
}

/*
 * Terminal code
 */
Terminal::Terminal(const Params &p)
    : SerialDevice(p), listenEvent(NULL), dataEvent(NULL),
      number(p.number), data_fd(-1), listener(p.port.build(p.name)),
      txbuf(16384), rxbuf(16384), outfile(terminalDump(p))
#if TRACING_ON == 1
      , linebuf(16384)
#endif
{
    if (outfile)
        outfile->stream()->setf(std::ios::unitbuf);

    if (p.port)
        listen();
}

Terminal::~Terminal()
{
    if (data_fd != -1)
        ::close(data_fd);

    if (listenEvent)
        delete listenEvent;

    if (dataEvent)
        delete dataEvent;
}

OutputStream *
Terminal::terminalDump(const TerminalParams &p)
{
    switch (p.outfile) {
      case TerminalDump::none:
        return nullptr;
      case TerminalDump::stdoutput:
        return simout.findOrCreate("stdout");
      case TerminalDump::stderror:
        return simout.findOrCreate("stderr");
      case TerminalDump::file:
        return simout.findOrCreate(p.name);
      default:
        panic("Invalid option\n");
    }
}

///////////////////////////////////////////////////////////////////////
// socket creation and terminal attach
//

void
Terminal::listen()
{
    if (ListenSocket::allDisabled()) {
        warn_once("Sockets disabled, not accepting terminal connections");
        return;
    }

    listener->listen();

    listenEvent = new ListenEvent(this, listener->getfd(), POLLIN);
    pollQueue.schedule(listenEvent);
}

void
Terminal::accept()
{
    if (!listener->islistening())
        panic("%s: cannot accept a connection if not listening!", name());

    int fd = listener->accept();
    if (data_fd != -1) {
        char message[] = "terminal already attached!\n";
        atomic_write(fd, message, sizeof(message));
        ::close(fd);
        return;
    }

    data_fd = fd;
    dataEvent = new DataEvent(this, data_fd, POLLIN);
    pollQueue.schedule(dataEvent);

    std::stringstream stream;
    ccprintf(stream, "==== m5 terminal: Terminal %d ====", number);

    // we need an actual carriage return followed by a newline for the
    // terminal
    stream << "\r\n";

    write((const uint8_t *)stream.str().c_str(), stream.str().size());

    DPRINTFN("attach terminal %d\n", number);
    char buf[1024];
    for (size_t i = 0; i < txbuf.size(); i += sizeof(buf)) {
        const size_t chunk_len(std::min(txbuf.size() - i, sizeof(buf)));
        txbuf.peek(buf, i, chunk_len);
        write((const uint8_t *)buf, chunk_len);
    }
}

void
Terminal::detach()
{
    if (data_fd != -1) {
        ::close(data_fd);
        data_fd = -1;
    }

    pollQueue.remove(dataEvent);
    delete dataEvent;
    dataEvent = NULL;

    DPRINTFN("detach terminal %d\n", number);
}

void
Terminal::data()
{
    uint8_t buf[1024];
    int len;

    len = read(buf, sizeof(buf));
    if (len) {
        rxbuf.write((char *)buf, len);
        notifyInterface();
    }
}

size_t
Terminal::read(uint8_t *buf, size_t len)
{
    if (data_fd < 0)
        panic("Terminal not properly attached.\n");

    ssize_t ret;
    do {
      ret = ::read(data_fd, buf, len);
    } while (ret == -1 && errno == EINTR);


    if (ret < 0)
        DPRINTFN("Read failed.\n");

    if (ret <= 0) {
        detach();
        return 0;
    }

    return ret;
}

// Terminal output.
size_t
Terminal::write(const uint8_t *buf, size_t len)
{
    if (data_fd < 0)
        panic("Terminal not properly attached.\n");

    ssize_t ret = atomic_write(data_fd, buf, len);
    if (ret < len)
        detach();

    return ret;
}

#define MORE_PENDING (1ULL << 61)
#define RECEIVE_SUCCESS (0ULL << 62)
#define RECEIVE_NONE (2ULL << 62)
#define RECEIVE_ERROR (3ULL << 62)

uint8_t
Terminal::readData()
{
    uint8_t c;

    assert(!rxbuf.empty());
    rxbuf.read((char *)&c, 1);

    DPRINTF(TerminalVerbose, "in: \'%c\' %#02x more: %d\n",
            isprint(c) ? c : ' ', c, !rxbuf.empty());

    return c;
}

uint64_t
Terminal::console_in()
{
    uint64_t value;

    if (dataAvailable()) {
        value = RECEIVE_SUCCESS | readData();
        if (!rxbuf.empty())
            value  |= MORE_PENDING;
    } else {
        value = RECEIVE_NONE;
    }

    DPRINTF(TerminalVerbose, "console_in: return: %#x\n", value);

    return value;
}

void
Terminal::writeData(uint8_t c)
{
#if TRACING_ON == 1
    if (debug::Terminal) {
        static char last = '\0';

        if ((c != '\n' && c != '\r') || (last != '\n' && last != '\r')) {
            if (c == '\n' || c == '\r') {
                int size = linebuf.size();
                char *buffer = new char[size + 1];
                linebuf.read(buffer, size);
                buffer[size] = '\0';
                DPRINTF(Terminal, "%s\n", buffer);
                delete [] buffer;
            } else {
                linebuf.write(&c, 1);
            }
        }

        last = c;
    }
#endif

    txbuf.write(&c, 1);

    if (data_fd >= 0)
        write(c);

    if (outfile)
        outfile->stream()->put((char)c);

    DPRINTF(TerminalVerbose, "out: \'%c\' %#02x\n",
            isprint(c) ? c : ' ', (int)c);

}

} // namespace gem5
