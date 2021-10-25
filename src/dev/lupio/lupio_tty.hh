/*
 * Copyright (c) 2021 The Regents of the University of California
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

#ifndef __LUPIO_TTY_HH__
#define __LUPIO_TTY_HH__

#include "dev/io_device.hh"
#include "dev/lupio/lupio_pic.hh"
#include "dev/platform.hh"
#include "dev/serial/serial.hh"
#include "params/LupioTTY.hh"

namespace gem5
{

class Terminal;
/**
 * LupioTTY:
 * The LupioTTY is a virtual terminal device that can both transmit characters
 * to a screen, as well as receive characters input from a keyboard.
 */
class LupioTTY : public BasicPioDevice
{
  private:
    // Register map
    enum
    {
        LUPIO_TTY_WRIT,
        LUPIO_TTY_READ,
        LUPIO_TTY_CTRL,
        LUPIO_TTY_STAT,

        // Max offset
        LUPIO_TTY_MAX,
    };

    // Internal registers
    int8_t writChar;
    int8_t readChar;
    bool writIntrEn;
    bool readIntrEn;

    uint64_t lupioTTYRead(const uint8_t addr);
    void lupioTTYWrite(const uint8_t addr, uint64_t c);
   /**
    * IRQ management
    */
    void lupioTTYUpdateIRQ();

    SerialDevice *terminal;
    const ByteOrder byteOrder = ByteOrder::little;
    Platform *platform;

  public:
    PARAMS(LupioTTY);
    LupioTTY(const Params &p);

    /**
     * Inform the LupIO-TTY there is data available
     */
    void dataAvailable();

    /**
     * Implement BasicPioDevice virtual functions
     */
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;
};

} //namespace gem5

#endif // __LUPIO_TTY_HH__
