/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 */

/* @file
 * Interface to connect a simulated ethernet device to the real world
 */

#ifndef __ETHERTAP_HH__
#define __ETHERTAP_HH__

#include <queue>
#include <string>

#include "dev/etherint.hh"
#include "dev/etherpkt.hh"
#include "sim/eventq.hh"
#include "base/pollevent.hh"
#include "sim/sim_object.hh"

class TapEvent;
class TapListener;

/*
 * Interface to connect a simulated ethernet device to the real world
 */
class EtherTap : public EtherInt
{
  protected:
    friend class TapEvent;
    TapEvent *event;

  protected:
    friend class TapListener;
    TapListener *listener;
    int socket;
    char *buffer;
    int buflen;
    int32_t buffer_offset;
    int32_t data_len;

    EtherDump *dump;

    void attach(int fd);
    void detach();

  protected:
    std::string device;
    std::queue<EthPacketPtr> packetBuffer;

    void process(int revent);
    void enqueue(EthPacketData *packet);
    void retransmit();

    /*
     */
    class TxEvent : public Event
    {
      protected:
        EtherTap *tap;

      public:
        TxEvent(EtherTap *_tap)
            : Event(&mainEventQueue), tap(_tap) {}
        void process() { tap->retransmit(); }
        virtual const char *description() { return "retransmit event"; }
    };

    friend class TxEvent;
    TxEvent txEvent;

  public:
    EtherTap(const std::string &name, EtherDump *dump, int port, int bufsz);
    virtual ~EtherTap();

    virtual bool recvPacket(EthPacketPtr packet);
    virtual void sendDone();

    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);
};

#endif // __ETHERTAP_HH__
