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
 * Device module for modelling a fixed bandwidth full duplex ethernet link
 */

#ifndef __ETHERLINK_HH__
#define __ETHERLINK_HH__

#include "sim/host.hh"
#include "sim/eventq.hh"
#include "dev/etherint.hh"
#include "dev/etherpkt.hh"
#include "sim/sim_object.hh"

class EtherDump;

/*
 * Model for a fixed bandwidth full duplex ethernet link
 */
class EtherLink : public SimObject
{
  protected:
    class Interface;

     /*
      * Model for a single uni-directional link
      */
    class Link : public Serializable {
      protected:
        std::string objName;

        Interface *txint;
        Interface *rxint;

        double ticks_per_byte;
        EtherDump *dump;

      protected:
        /*
         * Transfer is complete
         */
        class DoneEvent : public Event
        {
          protected:
            Link *link;

          public:
            DoneEvent(EventQueue *q, Link *l)
                : Event(q), link(l) {}
            virtual void process() { link->txDone(); }
            virtual const char *description()
                { return "ethernet link completion"; }
        };

        friend class DoneEvent;
        DoneEvent event;
        PacketPtr packet;

        void txDone();

      public:
        Link(const std::string &name, double rate, EtherDump *dump);
        ~Link() {}

        virtual std::string name() const { return objName; }

        bool busy() const { return (bool)packet; }
        bool transmit(PacketPtr &packet);

        void setTxInt(Interface *i) { assert(!txint); txint = i; }
        void setRxInt(Interface *i) { assert(!rxint); rxint = i; }
    };

    /*
     * Interface at each end of the link
     */
    class Interface : public EtherInt
    {
      private:
        Link *txlink;

      public:
        Interface(const std::string &name, Link *txlink, Link *rxlink);
        bool recvPacket(PacketPtr &packet) { return txlink->transmit(packet); }
        void sendDone() { }
    };

    Link *link1;
    Link *link2;

    EtherInt *int1;
    EtherInt *int2;

  public:
    EtherLink(const std::string &name, EtherInt *i1, EtherInt *i2,
              Tick speed, EtherDump *dump);
    virtual ~EtherLink();
};

#endif // __ETHERLINK_HH__
