/*
 * Copyright (c) 2005 The Regents of The University of Michigan
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

/** @file
 * Defines a 8250 UART
 */

#ifndef __DEV_UART8250_HH__
#define __DEV_UART8250_HH__

#include "base/bitunion.hh"
#include "base/logging.hh"
#include "dev/io_device.hh"
#include "dev/reg_bank.hh"
#include "dev/serial/uart.hh"
#include "params/Uart8250.hh"

namespace gem5
{

const uint8_t UART_MCR_LOOP = 0x10;

class Terminal;
class Platform;

class Uart8250 : public Uart
{
  protected:
    BitUnion8(Ier)
        Bitfield<0> rdi; // Receive data available interrupt.
        Bitfield<1> thri; // Transmit holding register interrupt.
        Bitfield<2> rlsi; // Receive line status interrupt.
        Bitfield<3> msi; // Modem status interrupt.
    EndBitUnion(Ier)

    BitUnion8(Iir)
        Bitfield<0> pending; // 0 = pending, 1 = not pending.
        Bitfield<2, 1> id; // ID of highest priority interrupt.
        Bitfield<7, 3> zeroes;
    EndBitUnion(Iir)

    BitUnion8(Lcr)
        Bitfield<1, 0> wordSize;
        Bitfield<2> stopBits;
        Bitfield<5, 3> parity;
        Bitfield<6> breakCont;
        Bitfield<7> dlab;
    EndBitUnion(Lcr)

    BitUnion8(Lsr)
        Bitfield<0> rdr; // Received data ready?
        Bitfield<1> overrunError;
        Bitfield<2> parityError;
        Bitfield<3> framingError;
        Bitfield<4> breakCond;
        Bitfield<5> tbe; // Transmit buffer empty.
        Bitfield<6> txEmpty; // Transmitter empty.
        Bitfield<7> unused;
    EndBitUnion(Lsr)

    enum class InterruptIds
    {
        Modem = 0, // Modem Status (lowest priority).
        Tx = 1,    // Tx Data.
        Rx = 2,    // Rx Data.
        Line = 3,  // Rx Line Status (highest priority).
    };

    class Registers : public RegisterBankLE
    {
      public:
        Registers(Uart8250 *uart, const std::string &new_name);

        class PairedRegister : public RegisterBase
        {
          protected:
            RegisterBase &_reg1, &_reg2;

          public:
            PairedRegister(RegisterBase &reg1, RegisterBase &reg2) :
                RegisterBase(reg1.name() + "/" + reg2.name(), reg1.size()),
                _reg1(reg1), _reg2(reg2)
            {
                panic_if(reg1.size() != reg2.size(),
                        "Mismatched paired register sizes %d, %d",
                        reg1.size(), reg2.size());
            }

            void serialize(std::ostream &os) const override {}
            bool unserialize(const std::string &s) override { return true; }

            void
            reset() override
            {
                _reg1.reset();
                _reg2.reset();
            }
        };

        class BankedRegister : public PairedRegister
        {
          private:
            RegisterBase *selected = nullptr;

          public:
            BankedRegister(RegisterBase &reg1, RegisterBase &reg2) :
                PairedRegister(reg1, reg2), selected(&reg1)
            {}

            void select(bool second) { selected = second ? &_reg2 : &_reg1; }

            const std::string &
            name() const override
            {
                return selected->name();
            }

            void read(void *buf) override { selected->read(buf); }
            void
            read(void *buf, off_t offset, size_t bytes) override
            {
                selected->read(buf, offset, bytes);
            }
            void write(const void *buf) override { selected->write(buf); }
            void
            write(const void *buf, off_t offset, size_t bytes) override
            {
                selected->write(buf, offset, bytes);
            }
        };

        class RWSwitchedRegister : public PairedRegister
        {
          public:
            using PairedRegister::PairedRegister;

            void read(void *buf) override { _reg1.read(buf); }
            void
            read(void *buf, off_t offset, size_t bytes) override
            {
                _reg1.read(buf, offset, bytes);
            }
            void write(const void *buf) override { _reg2.write(buf); }
            void
            write(const void *buf, off_t offset, size_t bytes) override
            {
                _reg2.write(buf, offset, bytes);
            }
        };

        // Offset 0.
        Register8 rbr = {"rbr"};
        Register8 thr = {"thr"};
        RWSwitchedRegister rbrThr;

        Register8 dll = {"dll"};
        BankedRegister rbrThrDll;

        // Offset 1.
        Register<Ier> ier = {"ier", 0};
        Register8 dlh = {"dlh"};
        BankedRegister ierDlh;

        // Offset 2.
        Register<Iir> iir = {"iir"};
        Register8 fcr = {"fcr"};
        RWSwitchedRegister iirFcr;

        // Offsets 3 - 6.
        Register<Lcr> lcr = {"lcr"};
        Register8 mcr = {"mcr"};
        Register<Lsr> lsr = {"lsr"};
        Register8 msr = {"msr"};

        // The scratch register didn't exist on the 8250.
        RegisterRaz sr = {"sr", 1};
    };
    using Register8 = Registers::Register8;
    template <class T>
    using Register = Registers::Register<T>;

    Registers registers;

    uint8_t readRbr(Register8 &reg);
    void writeThr(Register8 &reg, const uint8_t &data);
    void writeIer(Register<Ier> &reg, const Ier &ier);
    Iir readIir(Register<Iir> &reg);

    Tick lastTxInt;

    void processIntrEvent(int intrBit);
    void scheduleIntr(Event *event);
    void clearIntr(int intrBit);

    EventFunctionWrapper txIntrEvent;
    EventFunctionWrapper rxIntrEvent;

  public:
    using Params = Uart8250Params;
    Uart8250(const Params &p);

    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;
    AddrRangeList getAddrRanges() const override;

    /**
     * Inform the uart that there is data available.
     */
    void dataAvailable() override;


    /**
     * Return if we have an interrupt pending
     * @return interrupt status
     */
    virtual bool intStatus() { return status ? true : false; }

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

} // namespace gem5

#endif // __TSUNAMI_UART_HH__
