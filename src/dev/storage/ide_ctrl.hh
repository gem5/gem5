/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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
 * Simple PCI IDE controller with bus mastering capability and UDMA
 * modeled after controller in the Intel PIIX4 chip
 */

#ifndef __DEV_STORAGE_IDE_CTRL_HH__
#define __DEV_STORAGE_IDE_CTRL_HH__

#include "base/bitunion.hh"
#include "dev/io_device.hh"
#include "dev/pci/device.hh"
#include "dev/reg_bank.hh"
#include "params/IdeController.hh"

namespace gem5
{

class IdeDisk;

/**
 * Device model for an Intel PIIX4 IDE controller
 */

class IdeController : public PciDevice
{
  private:
    // Bus master IDE status register bit fields
    BitUnion8(BMIStatusReg)
        Bitfield<6> dmaCap0;
        Bitfield<5> dmaCap1;
        Bitfield<2> intStatus;
        Bitfield<1> dmaError;
        Bitfield<0> active;
    EndBitUnion(BMIStatusReg)

    BitUnion8(BMICommandReg)
        Bitfield<3> rw;
        Bitfield<0> startStop;
    EndBitUnion(BMICommandReg)

    /** Registers used in device specific PCI configuration */
    class ConfigSpaceRegs : public RegisterBankLE
    {
      public:
        ConfigSpaceRegs(const std::string &name) :
            RegisterBankLE(name, PCI_DEVICE_SPECIFIC)
        {
            // None of these registers are actually hooked up to control
            // anything, so they have no specially defined behaviors. They
            // just store values for now, but should presumably do something
            // in a more accurate model.
            addRegisters({primaryTiming, secondaryTiming, deviceTiming, raz0,
                          udmaControl, raz1, udmaTiming, raz2});
        }

        enum
        {
            TimeRegWithDecodeEnabled = 0x8000
        };

        /* Offset in config space */
        /* 0x40-0x41 */ Register16 primaryTiming =
                            {"primary timing", TimeRegWithDecodeEnabled};
        /* 0x42-0x43 */ Register16 secondaryTiming =
                            {"secondary timing", TimeRegWithDecodeEnabled};
        /* 0x44      */ Register8 deviceTiming = {"device timing"};
        /* 0x45-0x47 */ RegisterRaz raz0 = {"raz0", 3};
        /* 0x48      */ Register8 udmaControl = {"udma control"};
        /* 0x49      */ RegisterRaz raz1 = {"raz1", 1};
        /* 0x4a-0x4b */ Register16 udmaTiming = {"udma timing"};
        /* 0x4c-...  */ RegisterRaz raz2 =
                            {"raz2", (PCI_CONFIG_SIZE + 1) - 0x4c};

        void serialize(CheckpointOut &cp) const;
        void unserialize(CheckpointIn &cp);
    };

    ConfigSpaceRegs configSpaceRegs;

  public:
    class Channel : public Named
    {
      private:
        IdeController *ctrl;

        /** IDE disks connected to this controller
         * For more details about device0 and device1 see:
         * https://en.wikipedia.org/wiki/Parallel_ATA
         * #Multiple_devices_on_a_cable
         *
        */
        IdeDisk *device0 = nullptr, *device1 = nullptr;

        /** Currently selected disk */
        IdeDisk *_selected = nullptr;

        bool selectBit = false;
        bool primary;

        bool _pendingInterrupt = false;

      public:
        bool isPrimary() const { return primary; }

        bool pendingInterrupt() const { return _pendingInterrupt; }

        IdeDisk *selected() const { return _selected; }
        IdeController *controller() const { return ctrl; }

        void
        setDevice0(IdeDisk *disk)
        {
            assert(!device0 && disk);
            device0 = disk;
        }

        void
        setDevice1(IdeDisk *disk)
        {
            assert(!device1 && disk);
            device1 = disk;
        }

        /** Registers used for bus master interface */
        struct BMIRegs
        {
            void
            reset()
            {
                memset(static_cast<void *>(this), 0, sizeof(*this));
            }

            BMICommandReg command;
            uint8_t reserved0;
            BMIStatusReg status;
            uint8_t reserved1;
            uint32_t bmidtp;
        } bmiRegs;

        void
        select(bool select_device_1)
        {
            selectBit = select_device_1;
            _selected = selectBit ? device1 : device0;
        }

        void accessCommand(Addr offset, int size, uint8_t *data, bool read);
        void accessControl(Addr offset, int size, uint8_t *data, bool read);
        void accessBMI(Addr offset, int size, uint8_t *data, bool read);

        void setDmaComplete();

        void postInterrupt();
        void clearInterrupt();

        Channel(std::string new_name, IdeController *new_ctrl,
                bool new_primary);

        void serialize(const std::string &base, std::ostream &os) const;
        void unserialize(const std::string &base, CheckpointIn &cp);
    };

  private:
    Channel primary;
    Channel secondary;

    uint32_t ioShift, ctrlOffset;

    void dispatchAccess(PacketPtr pkt, bool read);

  public:
    PARAMS(IdeController);
    IdeController(const Params &p);

    virtual void postInterrupt(bool is_primary);
    virtual void clearInterrupt(bool is_primary);

    Tick writeConfig(PacketPtr pkt) override;
    Tick readConfig(PacketPtr pkt) override;

    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

} // namespace gem5

#endif // __DEV_STORAGE_IDE_CTRL_HH_
