/* $Id$ */

/* @file
 * User Console Definitions
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
#include "targetarch/ev5.hh"

#include "dev/console.hh"
#include "base/socket.hh"
#include "base/trace.hh"
#include "mem/functional_mem/memory_control.hh"

using namespace std;

// check whether an int is pending
inline bool
IntPending(int status, int mask)
{ return (status & mask) != 0; }

inline bool
IntTransition(int ostaus, int omask, int nstatus, int nmask)
{ return IntPending(ostaus, omask) != IntPending(nstatus, nmask); }

////////////////////////////////////////////////////////////////////////
//
//

SimConsole::Event::Event(SimConsole *c, int fd, int e)
    : PollEvent(fd, e), cons(c)
{
}

void
SimConsole::Event::process(int revent)
{
    if (revent & POLLIN)
        cons->data();
    else if (revent & POLLNVAL)
        cons->detach();
}

SimConsole::SimConsole(const string &name, const string &file, int num)
    : SimObject(name), event(NULL), number(num), in_fd(-1), out_fd(-1),
      listener(NULL), txbuf(16384), rxbuf(16384), outfile(NULL),
      intr_status(0), intr_enable(0), intr(NULL)
{
    if (!file.empty())
        outfile = new ofstream(file.c_str());

    if (outfile)
        outfile->setf(ios::unitbuf);
}

SimConsole::~SimConsole()
{
    close();

    if (outfile)
        delete outfile;
}

void
SimConsole::close()
{
    if (in_fd != -1)
        ::close(in_fd);

    if (out_fd != in_fd && out_fd != -1)
        ::close(out_fd);
}

void
SimConsole::attach(int in, int out, ConsoleListener *l)
{
    in_fd = in;
    out_fd = out;
    listener = l;

    event = new Event(this, in, POLLIN);
    pollQueue.schedule(event);

    stringstream stream;
    ccprintf(stream, "==== Simplescalar slave console: Console %d ====",
             number);
    // we need an actual carriage return followed by a newline for the
    // terminal
    stream << "\r\n";

    write((const uint8_t *)stream.str().c_str(), stream.str().size());


    DPRINTFN("attach console %d\n", number);

    txbuf.readall(out);
}

void
SimConsole::detach()
{
    close();
    in_fd = -1;
    out_fd = -1;

    pollQueue.remove(event);

    if (listener) {
        listener->add(this);
        listener = NULL;
    }

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
        raiseInt(ReceiveInterrupt);
    }
}

size_t
SimConsole::read(uint8_t *buf, size_t len)
{
    if (in_fd < 0)
        panic("SimConsole(read): Console not properly attached.\n");

    size_t ret;
    do {
      ret = ::read(in_fd, buf, len);
    } while (ret == -1 && errno == EINTR);


    if (ret < 0)
        DPRINTFN("SimConsole(read): Read failed.\n");

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
    if (out_fd < 0)
        panic("SimConsole(write): Console not properly attached.\n");

    size_t ret;
    for (;;) {
      ret = ::write(out_fd, buf, len);

      if (ret >= 0)
        break;

      if (errno != EINTR)
          detach();
    }

    return ret;
}

///////////////////////////////////////////////////////////////////////
// ConfigureTerm turns off all character processing by the host OS so
// the launched OS can control it.
//
// We ignore anything except stdin; the sconsole program runs this
// same code on the ttys for the slave consoles before connecting.
//
void
SimConsole::configTerm()
{
    struct termios ios;

    if (isatty(out_fd)) {
        if (tcgetattr(out_fd, &ios) < 0) {
            panic( "tcgetattr\n");
        }
        ios.c_iflag &= ~(ISTRIP|ICRNL|IGNCR|ICRNL|IXOFF|IXON);
        ios.c_oflag &= ~(OPOST);
        ios.c_oflag &= (ONLCR);
        ios.c_lflag &= ~(ISIG|ICANON|ECHO);
        ios.c_cc[VMIN] = 1;
        ios.c_cc[VTIME] = 0;
        if (tcsetattr(out_fd, TCSANOW, &ios) < 0) {
            panic( "tcsetattr\n");
        }
    }
}


///////////////////////////////////////////////////////////////////////
// console i/o
//

///////////////////////////////////////////////////////////////////////
//
// Console input.
// Returns -1 if there is no character pending, otherwise returns the
// char. Calling this function clears the input int (if no further
// chars are pending).
//
int
SimConsole::in()
{
    if (rxbuf.empty()) {
        clearInt(ReceiveInterrupt);
        return -1;
    }

    char c;
    rxbuf.read(&c, 1);

    DPRINTF(Console, "in: \'%c\' %#02x status: %#x\n",
            isprint(c) ? c : ' ', c, intr_status);

    return c;
}

///////////////////////////////////////////////////////////////////////
//
// Console output.
// NOTE: this very rudimentary device generates a TX int as soon as
// a character is output, since it has unlimited TX buffer capacity.
//
// Console output.
// Uses sim_console_out to perform functionality similar to 'write'
void
SimConsole::out(char c)
{
    txbuf.write(c);

    if (out_fd >= 0)
        write(c);

    if (outfile)
        outfile->write(&c, 1);

    raiseInt(TransmitInterrupt);

    DPRINTF(Console, "out: \'%c\' %#02x status: %#x\n",
            isprint(c) ? c : ' ', (int)c, intr_status);
}

// Simple console output used by Alpha firmware (not by the OS) -
// outputs the character to console n, and doesn't raise any
// interrupts
void
SimConsole::simple(char c)
{
    txbuf.write(c);

    if (out_fd >= 0)
        write(c);

    if (outfile)
        outfile->write(&c, 1);

    DPRINTF(Console, "simple char: \'%c\' %#02x\n",
            isprint(c) ? c : ' ', (int)c);
}

// Read the current interrupt status of this console.
int
SimConsole::intStatus()
{
#if 0
    DPRINTF(Console, "interrupt %d status: %#x\n",
            number, intr_status);
#endif

    return intr_status;
}

int
SimConsole::clearInt(int i)
{
    int old_status = intr_status;
    intr_status &= ~i;
    if (IntTransition(old_status, intr_enable, intr_status, intr_enable) &&
        intr)
        intr->clear(TheISA::INTLEVEL_IRQ0);
    return old_status;
}

void
SimConsole::raiseInt(int i)
{
    int old = intr_status;
    intr_status |= i;
    if (IntTransition(old, intr_enable, intr_status, intr_enable) && intr)
        intr->post(TheISA::INTLEVEL_IRQ0);
}

void
SimConsole::initInt(IntrControl *i)
{
    if (intr)
        panic("Console has already been initialized.");

    // note: intr_status and intr_enable will normally be 0, since
    // cs is statically allocated. When restoring from a checkpoint,
    // these fields will be set, so don't touch them here.
    intr = i; // interrupt handler
}

// Set the interrupt enable bits.
void
SimConsole::setInt(int bits)
{
    int old_enable;

    if (bits & ~(TransmitInterrupt | ReceiveInterrupt))
        panic("An interrupt was not set!");

    old_enable = intr_enable;
    intr_enable |= bits;

    if (IntTransition(intr_status, old_enable, intr_status, intr_enable) &&
        intr) {
        if (IntPending(intr_status, intr_enable))
            intr->post(TheISA::INTLEVEL_IRQ0);
        else
            intr->clear(TheISA::INTLEVEL_IRQ0);
    }
}


void
SimConsole::serialize()
{
    panic("Unimplemented");
}

void
SimConsole::unserialize(IniFile &db, const std::string &category,
                             ConfigNode *node)
{
    panic("Unimplemented");
}


BEGIN_DECLARE_SIM_OBJECT_PARAMS(SimConsole)

    SimObjectParam<ConsoleListener *> listener;
    SimObjectParam<IntrControl *> intr_control;
    Param<string> output;
    Param<int> number;

END_DECLARE_SIM_OBJECT_PARAMS(SimConsole)

BEGIN_INIT_SIM_OBJECT_PARAMS(SimConsole)

    INIT_PARAM(listener, "console listener"),
    INIT_PARAM(intr_control, "interrupt controller"),
    INIT_PARAM_DFLT(output, "file to dump output to", ""),
    INIT_PARAM_DFLT(number, "console number", 0)

END_INIT_SIM_OBJECT_PARAMS(SimConsole)

CREATE_SIM_OBJECT(SimConsole)
{
    SimConsole *console = new SimConsole(getInstanceName(), output, number);
    ((ConsoleListener *)listener)->add(console);
    ((SimConsole *)console)->initInt(intr_control);
    ((SimConsole *)console)->setInt(SimConsole::TransmitInterrupt |
                                    SimConsole::ReceiveInterrupt);

    return console;
}

REGISTER_SIM_OBJECT("SimConsole", SimConsole)

////////////////////////////////////////////////////////////////////////
//
//

ConsoleListener::ConsoleListener(const string &name)
    : SimObject(name), event(NULL)
{}

ConsoleListener::~ConsoleListener()
{
    if (event)
        delete event;
}

void
ConsoleListener::Event::process(int revent)
{
    listener->accept();
}

///////////////////////////////////////////////////////////////////////
// socket creation and console attach
//

void
ConsoleListener::listen(int port)
{
    while (!listener.listen(port, true)) {
        DPRINTF(Console, ": can't bind address console port %d inuse PID %d\n",
                port, getpid());
        port++;
    }

    cerr << "Listening for console connection on port " << port << endl;
    event = new Event(this, listener.getfd(), POLLIN);
    pollQueue.schedule(event);
}

void
ConsoleListener::add(SimConsole *cons)
{ ConsoleList.push_back(cons);}

void
ConsoleListener::accept()
{
    if (!listener.islistening())
        panic("%s: cannot accept a connection if we're not listening!",
              name());

    int sfd = listener.accept(true);
    if (sfd != -1) {
        iter_t i = ConsoleList.begin();
        iter_t end = ConsoleList.end();
        if (i == end) {
            close(sfd);
        } else {
            (*i)->attach(sfd, this);
            i = ConsoleList.erase(i);
        }
    }
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(ConsoleListener)

    Param<int> port;

END_DECLARE_SIM_OBJECT_PARAMS(ConsoleListener)

BEGIN_INIT_SIM_OBJECT_PARAMS(ConsoleListener)

    INIT_PARAM_DFLT(port, "listen port", 3456)

END_INIT_SIM_OBJECT_PARAMS(ConsoleListener)

CREATE_SIM_OBJECT(ConsoleListener)
{
    ConsoleListener *listener = new ConsoleListener(getInstanceName());
    listener->listen(port);

    return listener;
}

REGISTER_SIM_OBJECT("ConsoleListener", ConsoleListener)
