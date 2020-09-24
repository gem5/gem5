/*
 * Copyright 2019 Google, Inc.
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

#ifndef __ARCH_ARM_FASTMODEL_IRIS_THREAD_CONTEXT_HH__
#define __ARCH_ARM_FASTMODEL_IRIS_THREAD_CONTEXT_HH__

#include <list>
#include <map>
#include <memory>

#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "iris/IrisInstance.h"
#include "iris/detail/IrisErrorCode.h"
#include "iris/detail/IrisObjects.h"
#include "sim/system.hh"

namespace Iris
{

// This class is the base for ThreadContexts which read and write state using
// the Iris API.
class ThreadContext : public ::ThreadContext
{
  public:
    typedef std::map<std::string, iris::ResourceInfo> ResourceMap;

    typedef std::vector<iris::ResourceId> ResourceIds;
    typedef std::map<int, std::string> IdxNameMap;

  protected:
    ::BaseCPU *_cpu;
    int _threadId;
    ContextID _contextId;
    System *_system;
    ::BaseTLB *_dtb;
    ::BaseTLB *_itb;
    ::BaseISA *_isa;

    std::string _irisPath;
    iris::InstanceId _instId = iris::IRIS_UINT64_MAX;

    // Temporary holding places for the vector reg accessors to return.
    // These are not updated live, only when requested.
    mutable std::vector<ArmISA::VecRegContainer> vecRegs;
    mutable std::vector<ArmISA::VecPredRegContainer> vecPredRegs;

    Status _status = Active;
    Event *enableAfterPseudoEvent;

    virtual void initFromIrisInstance(const ResourceMap &resources);

    iris::ResourceId extractResourceId(
            const ResourceMap &resources, const std::string &name);
    void extractResourceMap(ResourceIds &ids,
            const ResourceMap &resources, const IdxNameMap &idx_names);


    ResourceIds miscRegIds;
    ResourceIds intReg32Ids;
    ResourceIds intReg64Ids;
    ResourceIds flattenedIntIds;
    ResourceIds ccRegIds;

    iris::ResourceId pcRscId = iris::IRIS_UINT64_MAX;
    iris::ResourceId icountRscId;

    ResourceIds vecRegIds;
    ResourceIds vecPredRegIds;

    std::vector<iris::MemorySpaceInfo> memorySpaces;
    std::vector<iris::MemorySupportedAddressTranslationResult> translations;

    std::unique_ptr<PortProxy> virtProxy = nullptr;
    std::unique_ptr<PortProxy> physProxy = nullptr;


    // A queue to keep track of instruction count based events.
    EventQueue comInstEventQueue;
    // A helper function to maintain the IRIS step count. This makes sure the
    // step count is correct even after IRIS resets it for us, and also handles
    // events which are supposed to happen at the current instruction count.
    void maintainStepping();


    using BpId = uint64_t;
    struct BpInfo
    {
        Addr pc;
        std::vector<BpId> ids;
        using EventList = std::list<PCEvent *>;
        std::shared_ptr<EventList> events;

        BpInfo(Addr _pc) : pc(_pc), events(new EventList) {}

        bool empty() const { return events->empty(); }
        bool validIds() const { return !ids.empty(); }
        void clearIds() { ids.clear(); }
    };

    using BpInfoPtr = std::unique_ptr<BpInfo>;
    using BpInfoMap = std::map<Addr, BpInfoPtr>;
    using BpInfoIt = BpInfoMap::iterator;

    BpInfoMap bps;

    BpInfoIt getOrAllocBp(Addr pc);

    void installBp(BpInfoIt it);
    void uninstallBp(BpInfoIt it);
    void delBp(BpInfoIt it);

    virtual const std::vector<iris::MemorySpaceId> &getBpSpaceIds() const = 0;


    iris::IrisErrorCode instanceRegistryChanged(
            uint64_t esId, const iris::IrisValueMap &fields, uint64_t time,
            uint64_t sInstId, bool syncEc, std::string &error_message_out);
    iris::IrisErrorCode phaseInitLeave(
            uint64_t esId, const iris::IrisValueMap &fields, uint64_t time,
            uint64_t sInstId, bool syncEc, std::string &error_message_out);
    iris::IrisErrorCode simulationTimeEvent(
            uint64_t esId, const iris::IrisValueMap &fields, uint64_t time,
            uint64_t sInstId, bool syncEc, std::string &error_message_out);
    iris::IrisErrorCode breakpointHit(
            uint64_t esId, const iris::IrisValueMap &fields, uint64_t time,
            uint64_t sInstId, bool syncEc, std::string &error_message_out);
    iris::IrisErrorCode semihostingEvent(
            uint64_t esId, const iris::IrisValueMap &fields, uint64_t time,
            uint64_t sInstId, bool syncEc, std::string &error_message_out);

    iris::EventStreamId regEventStreamId;
    iris::EventStreamId initEventStreamId;
    iris::EventStreamId timeEventStreamId;
    iris::EventStreamId breakpointEventStreamId;
    iris::EventStreamId semihostingEventStreamId;

    mutable iris::IrisInstance client;
    iris::IrisCppAdapter &call() const { return client.irisCall(); }
    iris::IrisCppAdapter &noThrow() const { return client.irisCallNoThrow(); }

    bool translateAddress(Addr &paddr, iris::MemorySpaceId p_space,
                          Addr vaddr, iris::MemorySpaceId v_space);

  public:
    ThreadContext(::BaseCPU *cpu, int id, System *system,
                  ::BaseTLB *dtb, ::BaseTLB *itb, ::BaseISA *isa,
                  iris::IrisConnectionInterface *iris_if,
                  const std::string &iris_path);
    virtual ~ThreadContext();

    virtual bool translateAddress(Addr &paddr, Addr vaddr) = 0;

    bool schedule(PCEvent *e) override;
    bool remove(PCEvent *e) override;

    void scheduleInstCountEvent(Event *event, Tick count) override;
    void descheduleInstCountEvent(Event *event) override;
    Tick getCurrentInstCount() override;

    ::BaseCPU *getCpuPtr() override { return _cpu; }
    int cpuId() const override { return _cpu->cpuId(); }
    uint32_t socketId() const override { return _cpu->socketId(); }

    int threadId() const override { return _threadId; }
    void setThreadId(int id) override { _threadId = id; }

    int contextId() const override { return _contextId; }
    void setContextId(int id) override { _contextId = id; }

    BaseTLB *
    getITBPtr() override
    {
        return _itb;
    }
    BaseTLB *
    getDTBPtr() override
    {
        return _dtb;
    }
    CheckerCPU *getCheckerCpuPtr() override { return nullptr; }
    ArmISA::Decoder *
    getDecoderPtr() override
    {
        panic("%s not implemented.", __FUNCTION__);
    }

    System *getSystemPtr() override { return _cpu->system; }

    BaseISA *
    getIsaPtr() override
    {
        return _isa;
    }

    PortProxy &getPhysProxy() override { return *physProxy; }
    PortProxy &getVirtProxy() override { return *virtProxy; }
    void initMemProxies(::ThreadContext *tc) override;

    Process *
    getProcessPtr() override
    {
        panic("%s not implemented.", __FUNCTION__);
    }
    void
    setProcessPtr(Process *p) override
    {
        panic("%s not implemented.", __FUNCTION__);
    }

    Status status() const override;
    void setStatus(Status new_status) override;
    void activate() override { setStatus(Active); }
    void suspend() override { setStatus(Suspended); }
    void halt() override { setStatus(Halted); }

    void
    takeOverFrom(::ThreadContext *old_context) override
    {
        panic("%s not implemented.", __FUNCTION__);
    }

    void regStats(const std::string &name) override {}

    // Not necessarily the best location for these...
    // Having an extra function just to read these is obnoxious
    Tick
    readLastActivate() override
    {
        panic("%s not implemented.", __FUNCTION__);
    }
    Tick readLastSuspend() override
    {
        panic("%s not implemented.", __FUNCTION__);
    }

    void
    copyArchRegs(::ThreadContext *tc) override
    {
        panic("%s not implemented.", __FUNCTION__);
    }

    void
    clearArchRegs() override
    {
        warn("Ignoring clearArchRegs()");
    }

    //
    // New accessors for new decoder.
    //
    RegVal readIntReg(RegIndex reg_idx) const override;

    RegVal
    readFloatReg(RegIndex reg_idx) const override
    {
        panic("%s not implemented.", __FUNCTION__);
    }

    const VecRegContainer &readVecReg(const RegId &reg) const override;
    VecRegContainer &
    getWritableVecReg(const RegId &reg) override
    {
        panic("%s not implemented.", __FUNCTION__);
    }

    /** Vector Register Lane Interfaces. */
    /** @{ */
    /** Reads source vector 8bit operand. */
    ConstVecLane8
    readVec8BitLaneReg(const RegId &reg) const override
    {
        panic("%s not implemented.", __FUNCTION__);
    }

    /** Reads source vector 16bit operand. */
    ConstVecLane16
    readVec16BitLaneReg(const RegId &reg) const override
    {
        panic("%s not implemented.", __FUNCTION__);
    }

    /** Reads source vector 32bit operand. */
    ConstVecLane32
    readVec32BitLaneReg(const RegId &reg) const override
    {
        panic("%s not implemented.", __FUNCTION__);
    }

    /** Reads source vector 64bit operand. */
    ConstVecLane64
    readVec64BitLaneReg(const RegId &reg) const override
    {
        panic("%s not implemented.", __FUNCTION__);
    }

    /** Write a lane of the destination vector register. */
    void
    setVecLane(const RegId &reg, const LaneData<LaneSize::Byte> &val) override
    {
        panic("%s not implemented.", __FUNCTION__);
    }
    void
    setVecLane(const RegId &reg,
               const LaneData<LaneSize::TwoByte> &val) override
    {
        panic("%s not implemented.", __FUNCTION__);
    }
    void
    setVecLane(const RegId &reg,
               const LaneData<LaneSize::FourByte> &val) override
    {
        panic("%s not implemented.", __FUNCTION__);
    }
    void
    setVecLane(const RegId &reg,
               const LaneData<LaneSize::EightByte> &val) override
    {
        panic("%s not implemented.", __FUNCTION__);
    }
    /** @} */

    const VecElem &
    readVecElem(const RegId &reg) const override
    {
        panic("%s not implemented.", __FUNCTION__);
    }

    const VecPredRegContainer &readVecPredReg(const RegId &reg) const override;
    VecPredRegContainer &
    getWritableVecPredReg(const RegId &reg) override
    {
        panic("%s not implemented.", __FUNCTION__);
    }

    RegVal
    readCCReg(RegIndex reg_idx) const override
    {
        return readCCRegFlat(reg_idx);
    }

    void setIntReg(RegIndex reg_idx, RegVal val) override;

    void
    setFloatReg(RegIndex reg_idx, RegVal val) override
    {
        panic("%s not implemented.", __FUNCTION__);
    }

    void
    setVecReg(const RegId &reg, const VecRegContainer &val) override
    {
        panic("%s not implemented.", __FUNCTION__);
    }

    void
    setVecElem(const RegId& reg, const VecElem& val) override
    {
        panic("%s not implemented.", __FUNCTION__);
    }

    void
    setVecPredReg(const RegId &reg,
                  const VecPredRegContainer &val) override
    {
        panic("%s not implemented.", __FUNCTION__);
    }

    void
    setCCReg(RegIndex reg_idx, RegVal val) override
    {
        setCCRegFlat(reg_idx, val);
    }

    void pcStateNoRecord(const ArmISA::PCState &val) override { pcState(val); }
    MicroPC microPC() const override { return 0; }

    ArmISA::PCState pcState() const override;
    void pcState(const ArmISA::PCState &val) override;
    Addr instAddr() const override;
    Addr nextInstAddr() const override;

    RegVal readMiscRegNoEffect(RegIndex misc_reg) const override;
    RegVal
    readMiscReg(RegIndex misc_reg) override
    {
        return readMiscRegNoEffect(misc_reg);
    }

    void setMiscRegNoEffect(RegIndex misc_reg, const RegVal val) override;
    void
    setMiscReg(RegIndex misc_reg, const RegVal val) override
    {
        setMiscRegNoEffect(misc_reg, val);
    }

    RegId
    flattenRegId(const RegId& regId) const override
    {
        panic("%s not implemented.", __FUNCTION__);
    }

    // Also not necessarily the best location for these two.  Hopefully will go
    // away once we decide upon where st cond failures goes.
    unsigned
    readStCondFailures() const override
    {
        panic("%s not implemented.", __FUNCTION__);
    }

    void
    setStCondFailures(unsigned sc_failures) override
    {
        panic("%s not implemented.", __FUNCTION__);
    }

    // Same with st cond failures.
    Counter
    readFuncExeInst() const override
    {
        panic("%s not implemented.", __FUNCTION__);
    }

    /** @{ */
    /**
     * Flat register interfaces
     *
     * Some architectures have different registers visible in
     * different modes. Such architectures "flatten" a register (see
     * flattenRegId()) to map it into the
     * gem5 register file. This interface provides a flat interface to
     * the underlying register file, which allows for example
     * serialization code to access all registers.
     */

    RegVal readIntRegFlat(RegIndex idx) const override;
    void setIntRegFlat(RegIndex idx, uint64_t val) override;

    RegVal
    readFloatRegFlat(RegIndex idx) const override
    {
        panic("%s not implemented.", __FUNCTION__);
    }
    void
    setFloatRegFlat(RegIndex idx, RegVal val) override
    {
        panic("%s not implemented.", __FUNCTION__);
    }

    const VecRegContainer &readVecRegFlat(RegIndex idx) const override;
    VecRegContainer &
    getWritableVecRegFlat(RegIndex idx) override
    {
        panic("%s not implemented.", __FUNCTION__);
    }
    void
    setVecRegFlat(RegIndex idx, const VecRegContainer &val) override
    {
        panic("%s not implemented.", __FUNCTION__);
    }

    const VecElem&
    readVecElemFlat(RegIndex idx, const ElemIndex& elemIdx) const override
    {
        panic("%s not implemented.", __FUNCTION__);
    }
    void
    setVecElemFlat(RegIndex idx, const ElemIndex &elemIdx,
                   const VecElem &val) override
    {
        panic("%s not implemented.", __FUNCTION__);
    }

    const VecPredRegContainer &readVecPredRegFlat(RegIndex idx) const override;
    VecPredRegContainer &
    getWritableVecPredRegFlat(RegIndex idx) override
    {
        panic("%s not implemented.", __FUNCTION__);
    }
    void
    setVecPredRegFlat(RegIndex idx, const VecPredRegContainer &val) override
    {
        panic("%s not implemented.", __FUNCTION__);
    }

    RegVal readCCRegFlat(RegIndex idx) const override;
    void setCCRegFlat(RegIndex idx, RegVal val) override;
    /** @} */

    // hardware transactional memory
    void
    htmAbortTransaction(uint64_t htm_uid, HtmFailureFaultCause cause) override
    {
        panic("%s not implemented.", __FUNCTION__);
    }

    BaseHTMCheckpointPtr &
    getHtmCheckpointPtr() override
    {
        panic("%s not implemented.", __FUNCTION__);
    }

    void
    setHtmCheckpointPtr(BaseHTMCheckpointPtr cpt) override
    {
        panic("%s not implemented.", __FUNCTION__);
    }
};

} // namespace Iris

#endif // __ARCH_ARM_FASTMODEL_IRIS_THREAD_CONTEXT_HH__
