/*
 * Copyright (c) 2004 The Regents of The University of Michigan
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
 * Tsunami UART
 */

#ifndef __TSUNAMI_UART_HH__
#define __TSUNAMI_UART_HH__

#include "dev/tsunamireg.h"
#include "base/range.hh"
#include "dev/io_device.hh"

class SimConsole;

/*
 * Tsunami UART
 */
class TsunamiUart : public PioDevice
{
  private:
    Addr addr;
    static const Addr size = 0x8;


  protected:
    SimConsole *cons;
    int status_store;
    uint8_t next_char;
    bool valid_char;
    uint8_t IER;

    class IntrEvent : public Event
    {
        protected:
            TsunamiUart *uart;
        public:
            IntrEvent(TsunamiUart *u);
            virtual void process();
            virtual const char *description();
            void scheduleIntr();
    };

    IntrEvent intrEvent;

  public:
    TsunamiUart(const string &name, SimConsole *c, MemoryController *mmu,
            Addr a, HierParams *hier, Bus *bus);

    Fault read(MemReqPtr &req, uint8_t *data);
    Fault write(MemReqPtr &req, const uint8_t *data);


    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);

    /**
     * Return how long this access will take.
     * @param req the memory request to calcuate
     * @return Tick when the request is done
     */
    Tick cacheAccess(MemReqPtr &req);
};

#endif // __TSUNAMI_UART_HH__
