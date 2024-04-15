/*
 * Copyright (c) 2019 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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

#ifndef __DEV_ARM_GICV3_ITS_H__
#define __DEV_ARM_GICV3_ITS_H__

#include <cstdint>
#include <memory>
#include <queue>
#include <vector>

#include "base/addr_range.hh"
#include "base/bitunion.hh"
#include "base/coroutine.hh"
#include "base/types.hh"
#include "dev/dma_device.hh"
#include "params/Gicv3Its.hh"

namespace gem5
{

class Gicv3;
class Gicv3Redistributor;
class ItsProcess;
class ItsTranslation;
class ItsCommand;

enum class ItsActionType
{
    INITIAL_NOP,
    SEND_REQ,
    TERMINATE,
};

struct ItsAction
{
    ItsActionType type;
    PacketPtr pkt;
    Tick delay;
};

/**
 * GICv3 ITS module. This class is just modelling a pio device with its
 * memory mapped registers. Most of the ITS functionalities are
 * implemented as processes (ItsProcess) objects, like ItsTranslation or
 * ItsCommand.
 * Main job of Gicv3Its is to spawn those processes upon receival of packets.
 */
class Gicv3Its : public BasicPioDevice
{
    friend class gem5::ItsProcess;
    friend class gem5::ItsTranslation;
    friend class gem5::ItsCommand;

  public:
    class DataPort : public RequestPort
    {
      protected:
        Gicv3Its &its;

      public:
        DataPort(const std::string &_name, Gicv3Its &_its)
            : RequestPort(_name), its(_its)
        {}

        virtual ~DataPort() {}

        bool
        recvTimingResp(PacketPtr pkt)
        {
            return its.recvTimingResp(pkt);
        }

        void
        recvReqRetry()
        {
            return its.recvReqRetry();
        }
    };

    DataPort dmaPort;

    Port &getPort(const std::string &if_name, PortID idx) override;
    bool recvTimingResp(PacketPtr pkt);
    void recvReqRetry();

    Gicv3Its(const Gicv3ItsParams &params);

    void setGIC(Gicv3 *_gic);

    static const uint32_t itsControl = 0x0;
    static const uint32_t itsTranslate = 0x10000;

    // Address range part of Control frame
    static const AddrRange GITS_BASER;

    static const uint32_t NUM_BASER_REGS = 8;

    // We currently don't support two level ITS tables
    // The indirect bit is RAZ/WI for implementations that only
    // support flat tables.
    static const uint64_t BASER_INDIRECT = 0x4000000000000000;
    static const uint64_t BASER_TYPE = 0x0700000000000000;
    static const uint64_t BASER_ESZ = 0x001F000000000000;
    static const uint64_t BASER_SZ = 0x00000000000000FF;
    static const uint64_t BASER_WMASK =
        ~(BASER_INDIRECT | BASER_TYPE | BASER_ESZ);
    static const uint64_t BASER_WMASK_UNIMPL =
        ~(BASER_INDIRECT | BASER_TYPE | BASER_ESZ | BASER_SZ);

    // GITS_CTLR.quiescent mask
    static const uint32_t CTLR_QUIESCENT;

    enum : Addr
    {
        // Control frame
        GITS_CTLR = itsControl + 0x0000,
        GITS_IIDR = itsControl + 0x0004,
        GITS_TYPER = itsControl + 0x0008,
        GITS_CBASER = itsControl + 0x0080,
        GITS_CWRITER = itsControl + 0x0088,
        GITS_CREADR = itsControl + 0x0090,
        GITS_PIDR2 = itsControl + 0xffe8,

        // Translation frame
        GITS_TRANSLATER = itsTranslate + 0x0040
    };

    AddrRangeList getAddrRanges() const override;

    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

    DrainState drain() override;
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    void translate(PacketPtr pkt);

    BitUnion32(CTLR)
        Bitfield<31> quiescent;
        Bitfield<7, 4> itsNumber;
        Bitfield<1> imDe;
        Bitfield<0> enabled;
    EndBitUnion(CTLR)

    // Command read/write, (CREADR, CWRITER)
    BitUnion64(CRDWR)
        Bitfield<63, 32> high;
        Bitfield<31, 0> low;
        Bitfield<19, 5> offset;
        Bitfield<0> retry;
        Bitfield<0> stalled;
    EndBitUnion(CRDWR)

    BitUnion64(CBASER)
        Bitfield<63, 32> high;
        Bitfield<31, 0> low;
        Bitfield<63> valid;
        Bitfield<61, 59> innerCache;
        Bitfield<55, 53> outerCache;
        Bitfield<51, 12> physAddr;
        Bitfield<11, 10> shareability;
        Bitfield<7, 0> size;
    EndBitUnion(CBASER)

    BitUnion64(BASER)
        Bitfield<63> valid;
        Bitfield<62> indirect;
        Bitfield<61, 59> innerCache;
        Bitfield<58, 56> type;
        Bitfield<55, 53> outerCache;
        Bitfield<52, 48> entrySize;
        Bitfield<47, 12> physAddr;
        Bitfield<11, 10> shareability;
        Bitfield<9, 8> pageSize;
        Bitfield<7, 0> size;
    EndBitUnion(BASER)

    BitUnion64(TYPER)
        Bitfield<63, 32> high;
        Bitfield<31, 0> low;
        Bitfield<37> vmovp;
        Bitfield<36> cil;
        Bitfield<35, 32> cidBits;
        Bitfield<31, 24> hcc;
        Bitfield<19> pta;
        Bitfield<18> seis;
        Bitfield<17, 13> devBits;
        Bitfield<12, 8> idBits;
        Bitfield<7, 4> ittEntrySize;
        Bitfield<2> cct;
        Bitfield<1> _virtual;
        Bitfield<0> physical;
    EndBitUnion(TYPER)

    CTLR gitsControl;
    TYPER gitsTyper;
    CBASER gitsCbaser;
    CRDWR gitsCreadr;
    CRDWR gitsCwriter;
    uint32_t gitsIidr;
    uint32_t gitsTranslater;

    std::vector<BASER> tableBases;

    /**
     * Returns TRUE if the eventID supplied has bits above the implemented
     * size or above the itt_range
     */
    bool idOutOfRange(uint32_t event_id, uint8_t itt_range) const;

    /**
     * Returns TRUE if the value supplied has bits above the implemented range
     * or if the value supplied exceeds the maximum configured size in the
     * appropriate GITS_BASER<n>
     */
    bool deviceOutOfRange(uint32_t device_id) const;

    /**
     * Returns TRUE if the value (size) supplied exceeds the maximum
     * allowed by GITS_TYPER.ID_bits. Size is the parameter which is
     * passed to the ITS via the MAPD command and is stored in the
     * DTE.ittRange field.
     */
    bool sizeOutOfRange(uint32_t size) const;

    /**
     * Returns TRUE if the value supplied has bits above the implemented range
     * or if the value exceeds the total number of collections supported in
     * hardware and external memory
     */
    bool collectionOutOfRange(uint32_t collection_id) const;

    /**
     * Returns TRUE if the value supplied is larger than that permitted by
     * GICD_TYPER.IDbits or not in the LPI range and is not 1023
     */
    bool lpiOutOfRange(uint32_t intid) const;

  private: // Command
    uint64_t maxCommands() const;
    void checkCommandQueue();
    void incrementReadPointer();

  public: // TableWalk
    BitUnion64(DTE)
        Bitfield<57, 53> ittRange;
        Bitfield<52, 1> ittAddress;
        Bitfield<0> valid;
    EndBitUnion(DTE)

    BitUnion64(ITTE)
        Bitfield<59, 46> vpeid;
        Bitfield<45, 30> icid;
        Bitfield<29, 16> intNumHyp;
        Bitfield<15, 2> intNum;
        Bitfield<1> intType;
        Bitfield<0> valid;
    EndBitUnion(ITTE)

    BitUnion64(CTE)
        Bitfield<40, 1> rdBase;
        Bitfield<0> valid;
    EndBitUnion(CTE)

    enum InterruptType
    {
        VIRTUAL_INTERRUPT = 0,
        PHYSICAL_INTERRUPT = 1
    };

  private:
    Gicv3Redistributor *getRedistributor(uint64_t rd_base);

    Gicv3Redistributor *
    getRedistributor(CTE cte)
    {
        return getRedistributor(cte.rdBase);
    }

    ItsAction runProcess(ItsProcess *proc, PacketPtr pkt);
    ItsAction runProcessTiming(ItsProcess *proc, PacketPtr pkt);
    ItsAction runProcessAtomic(ItsProcess *proc, PacketPtr pkt);

    enum ItsTables
    {
        DEVICE_TABLE = 1,
        VPE_TABLE = 2,
        TRANSLATION_TABLE = 3,
        COLLECTION_TABLE = 4
    };

    enum PageSize
    {
        SIZE_4K,
        SIZE_16K,
        SIZE_64K
    };

    Addr pageAddress(enum ItsTables table);

    void moveAllPendingState(Gicv3Redistributor *rd1, Gicv3Redistributor *rd2);

  private:
    std::queue<ItsAction> packetsToRetry;
    uint32_t requestorId;
    Gicv3 *gic;
    EventFunctionWrapper commandEvent;

    bool pendingCommands;
    uint32_t pendingTranslations;
};

/**
 * ItsProcess is a base coroutine wrapper which is spawned by
 * the Gicv3Its module when the latter needs to perform different
 * actions, like translating a peripheral's MSI into an LPI
 * (See derived ItsTranslation) or processing a Command from the
 * ITS queue (ItsCommand).
 * The action to take is implemented by the method:
 *
 * virtual void main(Yield &yield) = 0;
 * It's inheriting from Packet::SenderState since the generic process
 * will be stopped (we are using coroutines) and sent with the packet
 * to memory when doing table walks.
 * When Gicv3Its receives a response, it will resume the coroutine from
 * the point it stopped when yielding.
 */
class ItsProcess : public Packet::SenderState
{
  public:
    using DTE = Gicv3Its::DTE;
    using ITTE = Gicv3Its::ITTE;
    using CTE = Gicv3Its::CTE;
    using Coroutine = gem5::Coroutine<PacketPtr, ItsAction>;
    using Yield = Coroutine::CallerType;

    ItsProcess(Gicv3Its &_its);
    virtual ~ItsProcess();

    /** Returns the Gicv3Its name. Mainly used for DPRINTS */
    const std::string name() const;

    ItsAction run(PacketPtr pkt);

  protected:
    void reinit();
    virtual void main(Yield &yield) = 0;

    void writeDeviceTable(Yield &yield, uint32_t device_id, DTE dte);

    void writeIrqTranslationTable(Yield &yield, const Addr itt_base,
                                  uint32_t event_id, ITTE itte);

    void writeIrqCollectionTable(Yield &yield, uint32_t collection_id,
                                 CTE cte);

    uint64_t readDeviceTable(Yield &yield, uint32_t device_id);

    uint64_t readIrqTranslationTable(Yield &yield, const Addr itt_base,
                                     uint32_t event_id);

    uint64_t readIrqCollectionTable(Yield &yield, uint32_t collection_id);

    void doRead(Yield &yield, Addr addr, void *ptr, size_t size);
    void doWrite(Yield &yield, Addr addr, void *ptr, size_t size);
    void terminate(Yield &yield);

  protected:
    Gicv3Its &its;

  private:
    std::unique_ptr<Coroutine> coroutine;
};

/**
 * An ItsTranslation is created whenever a peripheral writes a message in
 * GITS_TRANSLATER (MSI). In this case main will simply do the table walks
 * until it gets a redistributor and an INTID. It will then raise the
 * LPI interrupt to the target redistributor.
 */
class ItsTranslation : public ItsProcess
{
  public:
    ItsTranslation(Gicv3Its &_its);
    ~ItsTranslation();

  protected:
    void main(Yield &yield) override;

    std::pair<uint32_t, Gicv3Redistributor *>
    translateLPI(Yield &yield, uint32_t device_id, uint32_t event_id);
};

/**
 * An ItsCommand is created whenever there is a new command in the command
 * queue. Only one command can be executed per time.
 * main will firstly read the command from memory and then it will process
 * it.
 */
class ItsCommand : public ItsProcess
{
  public:
    union CommandEntry
    {
        struct
        {
            uint32_t type;
            uint32_t deviceId;
            uint32_t eventId;
            uint32_t pintId;

            uint32_t data[4];
        };

        uint64_t raw[4];
    };

    enum CommandType : uint32_t
    {
        CLEAR = 0x04,
        DISCARD = 0x0F,
        INT = 0x03,
        INV = 0x0C,
        INVALL = 0x0D,
        MAPC = 0x09,
        MAPD = 0x08,
        MAPI = 0x0B,
        MAPTI = 0x0A,
        MOVALL = 0x0E,
        MOVI = 0x01,
        SYNC = 0x05,
        VINVALL = 0x2D,
        VMAPI = 0x2B,
        VMAPP = 0x29,
        VMAPTI = 0x2A,
        VMOVI = 0x21,
        VMOVP = 0x22,
        VSYNC = 0x25
    };

    ItsCommand(Gicv3Its &_its);
    ~ItsCommand();

  protected:
    /**
     * Dispatch entry is a metadata struct which contains information about
     * the command (like the name) and the function object implementing
     * the command.
     */
    struct DispatchEntry
    {
        using ExecFn =
            std::function<void(ItsCommand *, Yield &, CommandEntry &)>;

        DispatchEntry(std::string _name, ExecFn _exec)
            : name(_name), exec(_exec)
        {}

        std::string name;
        ExecFn exec;
    };

    using DispatchTable =
        std::unordered_map<std::underlying_type<enum CommandType>::type,
                           DispatchEntry>;

    static DispatchTable cmdDispatcher;

    static std::string commandName(uint32_t cmd);

    void main(Yield &yield) override;

    void readCommand(Yield &yield, CommandEntry &command);
    void processCommand(Yield &yield, CommandEntry &command);

    // Commands
    void clear(Yield &yield, CommandEntry &command);
    void discard(Yield &yield, CommandEntry &command);
    void mapc(Yield &yield, CommandEntry &command);
    void mapd(Yield &yield, CommandEntry &command);
    void mapi(Yield &yield, CommandEntry &command);
    void mapti(Yield &yield, CommandEntry &command);
    void movall(Yield &yield, CommandEntry &command);
    void movi(Yield &yield, CommandEntry &command);
    void sync(Yield &yield, CommandEntry &command);
    void doInt(Yield &yield, CommandEntry &command);
    void inv(Yield &yield, CommandEntry &command);
    void invall(Yield &yield, CommandEntry &command);
    void vinvall(Yield &yield, CommandEntry &command);
    void vmapi(Yield &yield, CommandEntry &command);
    void vmapp(Yield &yield, CommandEntry &command);
    void vmapti(Yield &yield, CommandEntry &command);
    void vmovi(Yield &yield, CommandEntry &command);
    void vmovp(Yield &yield, CommandEntry &command);
    void vsync(Yield &yield, CommandEntry &command);

  protected: // Helpers
    bool
    idOutOfRange(CommandEntry &command, DTE dte) const
    {
        return its.idOutOfRange(command.eventId, dte.ittRange);
    }

    bool
    deviceOutOfRange(CommandEntry &command) const
    {
        return its.deviceOutOfRange(command.deviceId);
    }

    bool
    sizeOutOfRange(CommandEntry &command) const
    {
        const auto size = bits(command.raw[1], 4, 0);
        const auto valid = bits(command.raw[2], 63);
        if (valid)
            return its.sizeOutOfRange(size);
        else
            return false;
    }

    bool
    collectionOutOfRange(CommandEntry &command) const
    {
        return its.collectionOutOfRange(bits(command.raw[2], 15, 0));
    }
};

} // namespace gem5

#endif
