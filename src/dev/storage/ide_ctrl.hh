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
#include "params/IdeController.hh"

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

    struct Channel
    {
        std::string _name;

        const std::string
        name()
        {
            return _name;
        }

        /** Command and control block registers */
        Addr cmdAddr, cmdSize, ctrlAddr, ctrlSize;

        /** Registers used for bus master interface */
        struct BMIRegs
        {
            void reset() {
                memset(static_cast<void *>(this), 0, sizeof(*this));
            }

            BMICommandReg command;
            uint8_t reserved0;
            BMIStatusReg status;
            uint8_t reserved1;
            uint32_t bmidtp;
        } bmiRegs;

        /** IDE disks connected to this controller
         * For more details about device0 and device1 see:
         * https://en.wikipedia.org/wiki/Parallel_ATA
         * #Multiple_devices_on_a_cable
         *
        */
        IdeDisk *device0, *device1;

        /** Currently selected disk */
        IdeDisk *selected;

        bool selectBit;

        void
        select(bool select_device_1)
        {
            selectBit = select_device_1;
            selected = selectBit ? device1 : device0;
        }

        void accessCommand(Addr offset, int size, uint8_t *data, bool read);
        void accessControl(Addr offset, int size, uint8_t *data, bool read);
        void accessBMI(Addr offset, int size, uint8_t *data, bool read);

        Channel(std::string newName, Addr _cmdSize, Addr _ctrlSize);
        ~Channel();

        void serialize(const std::string &base, std::ostream &os) const;
        void unserialize(const std::string &base, CheckpointIn &cp);
    };

    Channel primary;
    Channel secondary;

    /** Bus master interface (BMI) registers */
    Addr bmiAddr, bmiSize;

    /** Registers used in device specific PCI configuration */
    uint16_t primaryTiming, secondaryTiming;
    uint8_t deviceTiming;
    uint8_t udmaControl;
    uint16_t udmaTiming;
    uint16_t ideConfig;

    // Internal management variables
    bool ioEnabled;
    bool bmEnabled;

    uint32_t ioShift, ctrlOffset;

    void dispatchAccess(PacketPtr pkt, bool read);

  public:
    typedef IdeControllerParams Params;
    const Params *params() const { return (const Params *)_params; }
    IdeController(Params *p);

    /** See if a disk is selected based on its pointer */
    bool isDiskSelected(IdeDisk *diskPtr);

    void intrPost();

    Tick writeConfig(PacketPtr pkt) override;
    Tick readConfig(PacketPtr pkt) override;

    void setDmaComplete(IdeDisk *disk);

    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};
#endif // __DEV_STORAGE_IDE_CTRL_HH_
