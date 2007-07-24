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
 * Implements the user interface to a serial console
 */

#include <sys/ioctl.h>
#include <sys/termios.h>
#include <sys/types.h>
#include <errno.h>
#include <poll.h>
#include <unistd.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include "base/misc.hh"
#include "base/output.hh"
#include "base/socket.hh"
#include "base/trace.hh"
#include "dev/platform.hh"
#include "dev/simconsole.hh"
#include "dev/uart.hh"
#include "params/SimConsole.hh"

using namespace std;


/*
 * Poll event for the listen socket
 */
SimConsole::ListenEvent::ListenEvent(SimConsole *c, int fd, int e)
    : PollEvent(fd, e), cons(c)
{
}

void
SimConsole::ListenEvent::process(int revent)
{
    cons->accept();
}

/*
 * Poll event for the data socket
 */
SimConsole::DataEvent::DataEvent(SimConsole *c, int fd, int e)
    : PollEvent(fd, e), cons(c)
{
}

void
SimConsole::DataEvent::process(int revent)
{
    if (revent & POLLIN)
        cons->data();
    else if (revent & POLLNVAL)
        cons->detach();
}

/*
 * SimConsole code
 */
SimConsole::SimConsole(const string &name, ostream *os, int num, int port)
    : SimObject(name), listenEvent(NULL), dataEvent(NULL), number(num),
      data_fd(-1), txbuf(16384), rxbuf(16384), outfile(os)
#if TRACING_ON == 1
      , linebuf(16384)
#endif
{
    if (outfile)
        outfile->setf(ios::unitbuf);

    if (port)
        listen(port);
}

SimConsole::~SimConsole()
{
    if (data_fd != -1)
        ::close(data_fd);

    if (listenEvent)
        delete listenEvent;

    if (dataEvent)
        delete dataEvent;
}

///////////////////////////////////////////////////////////////////////
// socket creation and console attach
//

void
SimConsole::listen(int port)
{
    while (!listener.listen(port, true)) {
        DPRINTF(Console,
                ": can't bind address console port %d inuse PID %d\n",
                port, getpid());
        port++;
    }

    int p1, p2;
    p2 = name().rfind('.') - 1;
    p1 = name().rfind('.', p2);
    ccprintf(cerr, "Listening for %s connection on port %d\n",
            name().substr(p1+1,p2-p1), port);

    listenEvent = new ListenEvent(this, listener.getfd(), POLLIN);
    pollQueue.schedule(listenEvent);
}

void
SimConsole::accept()
{
    if (!listener.islistening())
        panic("%s: cannot accept a connection if not listening!", name());

    int fd = listener.accept(true);
    if (data_fd != -1) {
        char message[] = "console already attached!\n";
        ::write(fd, message, sizeof(message));
        ::close(fd);
        return;
    }

    data_fd = fd;
    dataEvent = new DataEvent(this, data_fd, POLLIN);
    pollQueue.schedule(dataEvent);

    stringstream stream;
    ccprintf(stream, "==== m5 slave console: Console %d ====", number);

    // we need an actual carriage return followed by a newline for the
    // terminal
    stream << "\r\n";

    write((const uint8_t *)stream.str().c_str(), stream.str().size());

    DPRINTFN("attach console %d\n", number);

    txbuf.readall(data_fd);
}

void
SimConsole::detach()
{
    if (data_fd != -1) {
        ::close(data_fd);
        data_fd = -1;
    }

    pollQueue.remove(dataEvent);
    delete dataEvent;
    dataEvent = NULL;

    DPRINTFN("detach console %d\n", number);
}

void
SimConsole::data()
{
    uint8_t buf[1024];
    int len;

    len = read(buf, sizeof(buf));
    if (len) {
        rxbuf.write((char *)buf, len);
        // Inform the UART there is data available
        uart->dataAvailable();
    }
}

size_t
SimConsole::read(uint8_t *buf, size_t len)
{
    if (data_fd < 0)
        panic("Console not properly attached.\n");

    size_t ret;
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

// Console output.
size_t
SimConsole::write(const uint8_t *buf, size_t len)
{
    if (data_fd < 0)
        panic("Console not properly attached.\n");

    size_t ret;
    for (;;) {
      ret = ::write(data_fd, buf, len);

      if (ret >= 0)
        break;

      if (errno != EINTR)
      detach();
    }

    return ret;
}

#define MORE_PENDING (ULL(1) << 61)
#define RECEIVE_SUCCESS (ULL(0) << 62)
#define RECEIVE_NONE (ULL(2) << 62)
#define RECEIVE_ERROR (ULL(3) << 62)

uint8_t
SimConsole::in()
{
    bool empty;
    uint8_t c;

    empty = rxbuf.empty();
    assert(!empty);
    rxbuf.read((char *)&c, 1);
    empty = rxbuf.empty();


    DPRINTF(ConsoleVerbose, "in: \'%c\' %#02x more: %d\n",
            isprint(c) ? c : ' ', c, !empty);

    return c;
}

uint64_t
SimConsole::console_in()
{
    uint64_t value;

    if (dataAvailable()) {
        value = RECEIVE_SUCCESS | in();
        if (!rxbuf.empty())
            value  |= MORE_PENDING;
    } else {
        value = RECEIVE_NONE;
    }

    DPRINTF(ConsoleVerbose, "console_in: return: %#x\n", value);

    return value;
}

void
SimConsole::out(char c)
{
#if TRACING_ON == 1
    if (DTRACE(Console)) {
        static char last = '\0';

        if (c != '\n' && c != '\r' ||
            last != '\n' && last != '\r') {
            if (c == '\n' || c == '\r') {
                int size = linebuf.size();
                char *buffer = new char[size + 1];
                linebuf.read(buffer, size);
                buffer[size] = '\0';
                DPRINTF(Console, "%s\n", buffer);
                delete [] buffer;
            } else {
                linebuf.write(c);
            }
        }

        last = c;
    }
#endif

    txbuf.write(c);

    if (data_fd >= 0)
        write(c);

    if (outfile)
        outfile->write(&c, 1);

    DPRINTF(ConsoleVerbose, "out: \'%c\' %#02x\n",
            isprint(c) ? c : ' ', (int)c);

}

SimConsole *
SimConsoleParams::create()
{
    string filename = output;
    ostream *stream = NULL;

    if (!filename.empty()) {
        if (append_name)
            filename += "." + name;
        stream = simout.find(filename);
    }

    return new SimConsole(name, stream, number, port);
}
