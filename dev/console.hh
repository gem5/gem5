/* $Id$ */

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
    int intr_status;
    int intr_enable;

    // interrupt handle
    IntrControl *intr;

  public:
    /////////////////
    // OS interface

    // Input a character from the console. Returns the character (if
    // any) or -1 if there is no character pending on this console. If
    // no further characters are pending, the (input) interrupt is
    // cleared.
    int in();

    // Output a character to the console. This never fails, as this
    // device doesn't model finite buffering capacity.
    void out(char c);
    void simple(char c);

    enum {
        TransmitInterrupt = 1,
        ReceiveInterrupt = 2
    };

    // Read the current interrupt status of this console.
    int intStatus();

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
