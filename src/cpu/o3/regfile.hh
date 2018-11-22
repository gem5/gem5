/*
 * Copyright (c) 2016-2017 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder. You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
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
 * Authors: Kevin Lim
 *          Gabe Black
 */

#ifndef __CPU_O3_REGFILE_HH__
#define __CPU_O3_REGFILE_HH__

#include <vector>

#include "arch/isa_traits.hh"
#include "arch/kernel_stats.hh"
#include "arch/types.hh"
#include "base/trace.hh"
#include "config/the_isa.hh"
#include "cpu/o3/comm.hh"
#include "debug/IEW.hh"
#include "enums/VecRegRenameMode.hh"

class UnifiedFreeList;

/**
 * Simple physical register file class.
 */
class PhysRegFile
{
  private:

    using VecElem = TheISA::VecElem;
    using VecRegContainer = TheISA::VecRegContainer;
    using PhysIds = std::vector<PhysRegId>;
    using VecMode = Enums::VecRegRenameMode;
    using VecPredRegContainer = TheISA::VecPredRegContainer;
  public:
    using IdRange = std::pair<PhysIds::const_iterator,
                              PhysIds::const_iterator>;
  private:
    static constexpr auto NumVecElemPerVecReg = TheISA::NumVecElemPerVecReg;

    /** Integer register file. */
    std::vector<RegVal> intRegFile;
    std::vector<PhysRegId> intRegIds;

    /** Floating point register file. */
    std::vector<RegVal> floatRegFile;
    std::vector<PhysRegId> floatRegIds;

    /** Vector register file. */
    std::vector<VecRegContainer> vectorRegFile;
    std::vector<PhysRegId> vecRegIds;
    std::vector<PhysRegId> vecElemIds;

    /** Predicate register file. */
    std::vector<VecPredRegContainer> vecPredRegFile;
    std::vector<PhysRegId> vecPredRegIds;

    /** Condition-code register file. */
    std::vector<RegVal> ccRegFile;
    std::vector<PhysRegId> ccRegIds;

    /** Misc Reg Ids */
    std::vector<PhysRegId> miscRegIds;

    /**
     * Number of physical general purpose registers
     */
    unsigned numPhysicalIntRegs;

    /**
     * Number of physical floating point registers
     */
    unsigned numPhysicalFloatRegs;

    /**
     * Number of physical vector registers
     */
    unsigned numPhysicalVecRegs;

    /**
     * Number of physical vector element registers
     */
    unsigned numPhysicalVecElemRegs;

    /**
     * Number of physical predicate registers
     */
    unsigned numPhysicalVecPredRegs;

    /**
     * Number of physical CC registers
     */
    unsigned numPhysicalCCRegs;

    /** Total number of physical registers. */
    unsigned totalNumRegs;

    /** Mode in which vector registers are addressed. */
    VecMode vecMode;

  public:
    /**
     * Constructs a physical register file with the specified amount of
     * integer and floating point registers.
     */
    PhysRegFile(unsigned _numPhysicalIntRegs,
                unsigned _numPhysicalFloatRegs,
                unsigned _numPhysicalVecRegs,
                unsigned _numPhysicalVecPredRegs,
                unsigned _numPhysicalCCRegs,
                VecMode vmode
                );

    /**
     * Destructor to free resources
     */
    ~PhysRegFile() {}

    /** Initialize the free list */
    void initFreeList(UnifiedFreeList *freeList);

    /** @return the number of integer physical registers. */
    unsigned numIntPhysRegs() const { return numPhysicalIntRegs; }

    /** @return the number of floating-point physical registers. */
    unsigned numFloatPhysRegs() const { return numPhysicalFloatRegs; }
    /** @return the number of vector physical registers. */
    unsigned numVecPhysRegs() const { return numPhysicalVecRegs; }
    /** @return the number of predicate physical registers. */
    unsigned numPredPhysRegs() const { return numPhysicalVecPredRegs; }

    /** @return the number of vector physical registers. */
    unsigned numVecElemPhysRegs() const { return numPhysicalVecElemRegs; }

    /** @return the number of condition-code physical registers. */
    unsigned numCCPhysRegs() const { return numPhysicalCCRegs; }

    /** @return the total number of physical registers. */
    unsigned totalNumPhysRegs() const { return totalNumRegs; }

    /** Gets a misc register PhysRegIdPtr. */
    PhysRegIdPtr getMiscRegId(RegIndex reg_idx) {
        return &miscRegIds[reg_idx];
    }

    /** Reads an integer register. */
    RegVal
    readIntReg(PhysRegIdPtr phys_reg) const
    {
        assert(phys_reg->isIntPhysReg());

        DPRINTF(IEW, "RegFile: Access to int register %i, has data "
                "%#x\n", phys_reg->index(), intRegFile[phys_reg->index()]);
        return intRegFile[phys_reg->index()];
    }

    RegVal
    readFloatReg(PhysRegIdPtr phys_reg) const
    {
        assert(phys_reg->isFloatPhysReg());

        RegVal floatRegBits = floatRegFile[phys_reg->index()];

        DPRINTF(IEW, "RegFile: Access to float register %i as int, "
                "has data %#x\n", phys_reg->index(), floatRegBits);

        return floatRegBits;
    }

    /** Reads a vector register. */
    const VecRegContainer &
    readVecReg(PhysRegIdPtr phys_reg) const
    {
        assert(phys_reg->isVectorPhysReg());

        DPRINTF(IEW, "RegFile: Access to vector register %i, has "
                "data %s\n", int(phys_reg->index()),
                vectorRegFile[phys_reg->index()].print());

        return vectorRegFile[phys_reg->index()];
    }

    /** Reads a vector register for modification. */
    VecRegContainer &
    getWritableVecReg(PhysRegIdPtr phys_reg)
    {
        /* const_cast for not duplicating code above. */
        return const_cast<VecRegContainer&>(readVecReg(phys_reg));
    }

    /** Reads a vector register lane. */
    template <typename VecElem, int LaneIdx>
    VecLaneT<VecElem, true>
    readVecLane(PhysRegIdPtr phys_reg) const
    {
        return readVecReg(phys_reg).laneView<VecElem, LaneIdx>();
    }

    /** Reads a vector register lane. */
    template <typename VecElem>
    VecLaneT<VecElem, true>
    readVecLane(PhysRegIdPtr phys_reg) const
    {
        return readVecReg(phys_reg).laneView<VecElem>(phys_reg->elemIndex());
    }

    /** Get a vector register lane for modification. */
    template <typename LD>
    void
    setVecLane(PhysRegIdPtr phys_reg, const LD& val)
    {
        assert(phys_reg->isVectorPhysReg());

        DPRINTF(IEW, "RegFile: Setting vector register %i[%d] to %lx\n",
                int(phys_reg->index()), phys_reg->elemIndex(), val);

        vectorRegFile[phys_reg->index()].laneView<typename LD::UnderlyingType>(
                phys_reg->elemIndex()) = val;
    }

    /** Reads a vector element. */
    const VecElem &
    readVecElem(PhysRegIdPtr phys_reg) const
    {
        assert(phys_reg->isVectorPhysElem());
        auto ret = vectorRegFile[phys_reg->index()].as<VecElem>();
        const VecElem& val = ret[phys_reg->elemIndex()];
        DPRINTF(IEW, "RegFile: Access to element %d of vector register %i,"
                " has data %#x\n", phys_reg->elemIndex(),
                int(phys_reg->index()), val);

        return val;
    }

    /** Reads a predicate register. */
    const VecPredRegContainer& readVecPredReg(PhysRegIdPtr phys_reg) const
    {
        assert(phys_reg->isVecPredPhysReg());

        DPRINTF(IEW, "RegFile: Access to predicate register %i, has "
                "data %s\n", int(phys_reg->index()),
                vecPredRegFile[phys_reg->index()].print());

        return vecPredRegFile[phys_reg->index()];
    }

    VecPredRegContainer& getWritableVecPredReg(PhysRegIdPtr phys_reg)
    {
        /* const_cast for not duplicating code above. */
        return const_cast<VecPredRegContainer&>(readVecPredReg(phys_reg));
    }

    /** Reads a condition-code register. */
    RegVal
    readCCReg(PhysRegIdPtr phys_reg)
    {
        assert(phys_reg->isCCPhysReg());

        DPRINTF(IEW, "RegFile: Access to cc register %i, has "
                "data %#x\n", phys_reg->index(),
                ccRegFile[phys_reg->index()]);

        return ccRegFile[phys_reg->index()];
    }

    /** Sets an integer register to the given value. */
    void
    setIntReg(PhysRegIdPtr phys_reg, RegVal val)
    {
        assert(phys_reg->isIntPhysReg());

        DPRINTF(IEW, "RegFile: Setting int register %i to %#x\n",
                phys_reg->index(), val);

        if (!phys_reg->isZeroReg())
            intRegFile[phys_reg->index()] = val;
    }

    void
    setFloatReg(PhysRegIdPtr phys_reg, RegVal val)
    {
        assert(phys_reg->isFloatPhysReg());

        DPRINTF(IEW, "RegFile: Setting float register %i to %#x\n",
                phys_reg->index(), (uint64_t)val);

        if (!phys_reg->isZeroReg())
            floatRegFile[phys_reg->index()] = val;
    }

    /** Sets a vector register to the given value. */
    void
    setVecReg(PhysRegIdPtr phys_reg, const VecRegContainer& val)
    {
        assert(phys_reg->isVectorPhysReg());

        DPRINTF(IEW, "RegFile: Setting vector register %i to %s\n",
                int(phys_reg->index()), val.print());

        vectorRegFile[phys_reg->index()] = val;
    }

    /** Sets a vector register to the given value. */
    void
    setVecElem(PhysRegIdPtr phys_reg, const VecElem val)
    {
        assert(phys_reg->isVectorPhysElem());

        DPRINTF(IEW, "RegFile: Setting element %d of vector register %i to"
                " %#x\n", phys_reg->elemIndex(), int(phys_reg->index()), val);

        vectorRegFile[phys_reg->index()].as<VecElem>()[phys_reg->elemIndex()] =
                val;
    }

    /** Sets a predicate register to the given value. */
    void setVecPredReg(PhysRegIdPtr phys_reg, const VecPredRegContainer& val)
    {
        assert(phys_reg->isVecPredPhysReg());

        DPRINTF(IEW, "RegFile: Setting predicate register %i to %s\n",
                int(phys_reg->index()), val.print());

        vecPredRegFile[phys_reg->index()] = val;
    }

    /** Sets a condition-code register to the given value. */
    void
    setCCReg(PhysRegIdPtr phys_reg, RegVal val)
    {
        assert(phys_reg->isCCPhysReg());

        DPRINTF(IEW, "RegFile: Setting cc register %i to %#x\n",
                phys_reg->index(), (uint64_t)val);

        ccRegFile[phys_reg->index()] = val;
    }

    /** Get the PhysRegIds of the elems of a vector register.
     * Auxiliary function to transition from Full vector mode to Elem mode.
     */
    IdRange getRegElemIds(PhysRegIdPtr reg);

    /**
     * Get the PhysRegIds of the elems of all vector registers.
     * Auxiliary function to transition from Full vector mode to Elem mode
     * and to initialise the rename map.
     */
    IdRange getRegIds(RegClass cls);

    /**
     * Get the true physical register id.
     * As many parts work with PhysRegIdPtr, we need to be able to produce
     * the pointer out of just class and register idx.
     */
    PhysRegIdPtr getTrueId(PhysRegIdPtr reg);
};


#endif //__CPU_O3_REGFILE_HH__
