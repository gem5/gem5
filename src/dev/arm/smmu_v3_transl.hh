/*
 * Copyright (c) 2013, 2018-2019, 2024 Arm Limited
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

#ifndef __DEV_ARM_SMMU_V3_TRANSL_HH__
#define __DEV_ARM_SMMU_V3_TRANSL_HH__

#include "base/compiler.hh"
#include "dev/arm/smmu_v3_deviceifc.hh"
#include "dev/arm/smmu_v3_proc.hh"
#include "mem/packet.hh"

namespace gem5
{

namespace ArmISA
{
struct PageTableOps;
}

struct SMMUTranslRequest
{
    Addr     addr;
    unsigned size;
    uint32_t sid;  // streamId
    uint32_t ssid; // substreamId
    bool     isWrite;
    bool     isPrefetch;
    bool     isAtsRequest;

    PacketPtr pkt;

    static SMMUTranslRequest fromPacket(PacketPtr pkt, bool ats = false);
    static SMMUTranslRequest prefetch(Addr addr, uint32_t sid, uint32_t ssid);
};

class SMMUTranslationProcess : public SMMUProcess
{
  private:
    struct TranslContext
    {
        bool stage1Enable;
        bool stage2Enable;
        Addr ttb0, ttb1, httb;
        uint16_t asid;
        uint16_t vmid;
        uint8_t stage1TranslGranule;
        uint8_t stage2TranslGranule;
        uint8_t t0sz;
        uint8_t s2t0sz;
    };

    enum FaultType : uint8_t
    {
        FAULT_NONE,
        FAULT_UUT = 0x1, // F_UUT = Unsupported Upstream Transaction
        FAULT_BAD_STREAMID = 0x2, // C_BAD_STREAMID = Transaction streamID out of range
        FAULT_STE_FETCH = 0x3, // F_STE_FETCH = Fetch of STE caused external abort
        FAULT_BAD_STE = 0x4, // C_BAD_STE = Invalid STE
        FAULT_BAD_ATS_TREQ = 0x5, // F_BAD_ATS_TREQ
        FAULT_STREAM_DISABLED = 0x6, // F_STREAM_DISABLED = Non-substream trans disabled
        FAULT_TRANSL_FORBIDDEN = 0x7, // F_TRANSL_FORBIDDEN = SMMU bypass not allowed
        FAULT_BAD_SUBSTREAMID = 0x8, // F_BAD_SUBSTREAMID = Bad substreamID
        FAULT_CD_FETCH = 0x9, // F_CD_FETCH = Fetch of CD caused external abort
        FAULT_BAD_CD = 0xa, // C_BAD_CD = Invalid CD
        FAULT_WALK_EABT = 0xb, // F_WALK_EABT = Table walk/update caused external abort
        FAULT_TRANSLATION = 0x10, // F_TRANSLATION = Translation Fault
        FAULT_ADDR_SIZE = 0x11, // F_ADDR_SIZE = Address Size fault
        FAULT_ACCESS = 0x12, // F_ACCESS = Access flag fault
        FAULT_PERMISSION = 0x13, // F_PERMISSION = Permission fault
        FAULT_TLB_CONFLICT = 0x20, // F_TLB_CONFLICT = TLB conflict
        FAULT_CFG_CONFLICT = 0x21, // F_CFG_CONFLICT = Config cache conflict
        FAULT_PAGE_REQUEST = 0x24, // E_PAGE_REQUEST
        FAULT_VMS_FETCH = 0x25, // F_VMS_FETCH
    };

    /* The class of the operation that caused the fault */
    enum FaultClass
    {
        CD = 0x0, // CD fetch
        TT = 0x1, // Stage1 translation table fetch
        IN = 0x2, // Input address caused fault
        RESERVED = 0x3
    };

    struct Fault
    {
        explicit Fault(FaultType _type,
                       FaultClass _clss=FaultClass::RESERVED,
                       bool _stage2=false, Addr _ipa=0)
          : type(_type), clss(_clss), stage2(_stage2), ipa(_ipa)
        {}

        Fault(const Fault &rhs) = default;
        Fault& operator=(const Fault &rhs) = default;

        bool isFaulting() const { return type != FAULT_NONE; }

        FaultType type;
        FaultClass clss;
        bool stage2;
        Addr ipa;
    };

    struct TranslResult
    {
        TranslResult()
          : fault(FaultType::FAULT_NONE),
            addr(0), addrMask(0), writable(false)
        {}

        TranslResult& operator=(const TranslResult &rhs) = default;

        bool isFaulting() const { return fault.isFaulting(); }

        Fault fault;
        Addr  addr;
        Addr  addrMask;
        bool  writable;
    };

    SMMUv3DeviceInterface &ifc;

    SMMUTranslRequest request;
    TranslContext context;

    Tick recvTick;
    GEM5_CLASS_VAR_USED Tick faultTick;

    virtual void main(Yield &yield);

    TranslResult bypass(Addr addr) const;
    TranslResult smmuTranslation(Yield &yield);

    bool microTLBLookup(Yield &yield, TranslResult &tr);
    bool ifcTLBLookup(Yield &yield, TranslResult &tr, bool &wasPrefetched);
    bool smmuTLBLookup(Yield &yield, TranslResult &tr);

    void microTLBUpdate(Yield &yield, const TranslResult &tr);
    void ifcTLBUpdate(Yield &yield, const TranslResult &tr);
    void smmuTLBUpdate(Yield &yield, const TranslResult &tr);

    bool configCacheLookup(Yield &yield, TranslContext &tc);
    void configCacheUpdate(Yield &yield, const TranslContext &tc);
    bool findConfig(Yield &yield, TranslContext &tc, TranslResult &tr);

    void walkCacheLookup(Yield &yield,
                         const WalkCache::Entry *&walkEntry,
                         Addr addr, uint16_t asid, uint16_t vmid,
                         unsigned stage, unsigned level);

    void walkCacheUpdate(Yield &yield, Addr va, Addr vaMask, Addr pa,
                         unsigned stage, unsigned level,
                         bool leaf, uint8_t permissions);

    TranslResult walkStage1And2(Yield &yield, Addr addr,
                                const ArmISA::PageTableOps *pt_ops,
                                unsigned level, Addr walkPtr);

    TranslResult walkStage2(Yield &yield, Addr addr, bool final_tr,
                            const ArmISA::PageTableOps *pt_ops,
                            unsigned level, Addr walkPtr);

    TranslResult translateStage1And2(Yield &yield, Addr addr);
    TranslResult translateStage2(Yield &yield, Addr addr, bool final_tr);

    TranslResult combineTranslations(const TranslResult &s1tr,
                                     const TranslResult &s2tr) const;

    /**
     * Used to force ordering on transactions with same
     * (SID, SSID, 4k page) to avoid multiple identical
     * page-table walks.
     */
    bool hazard4kCheck();
    void hazard4kRegister();
    void hazard4kHold(Yield &yield);
    void hazard4kRelease();

    /**
     * Used to force ordering on transactions with the same orderId.
     * This attempts to model AXI IDs.
     */
    void hazardIdRegister();
    void hazardIdHold(Yield &yield);
    void hazardIdRelease();

    void issuePrefetch(Addr addr);

    void completeTransaction(Yield &yield, const TranslResult &tr);
    void completePrefetch(Yield &yield);

    void sendEvent(Yield &yield, const SMMUEvent &ev);

    void doReadSTE(Yield &yield, StreamTableEntry &ste, uint32_t sid);
    TranslResult doReadCD(Yield &yield, ContextDescriptor &cd,
                          const StreamTableEntry &ste, uint32_t sid, uint32_t ssid);
    void doReadConfig(Yield &yield, Addr addr, void *ptr, size_t size,
                      uint32_t sid, uint32_t ssid);
    void doReadPTE(Yield &yield, Addr va, Addr addr, void *ptr,
                   unsigned stage, unsigned level);

  public:
    SMMUTranslationProcess(const std::string &name, SMMUv3 &_smmu,
        SMMUv3DeviceInterface &_ifc);

    virtual ~SMMUTranslationProcess();

    void beginTransaction(const SMMUTranslRequest &req);
    void resumeTransaction();
};

} // namespace gem5

#endif /* __DEV_ARM_SMMU_V3_TRANSL_HH__ */
