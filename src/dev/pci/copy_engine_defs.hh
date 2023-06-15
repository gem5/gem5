/*
 * Copyright (c) 2008 The Regents of The University of Michigan
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
 * Register and structure descriptions for Intel's I/O AT DMA Engine
 */
#include "base/bitfield.hh"
#include "base/compiler.hh"
#include "sim/serialize.hh"

namespace gem5
{

namespace copy_engine_reg
{

// General Channel independant registers, 128 bytes starting at 0x00
const uint32_t GEN_CHANCOUNT    = 0x00;
const uint32_t GEN_XFERCAP      = 0x01;
const uint32_t GEN_INTRCTRL     = 0x03;
const uint32_t GEN_ATTNSTATUS   = 0x04;


// Channel specific registers, each block is 128 bytes, starting at 0x80
const uint32_t CHAN_CONTROL         = 0x00;
const uint32_t CHAN_STATUS          = 0x04;
const uint32_t CHAN_CHAINADDR       = 0x0C;
const uint32_t CHAN_CHAINADDR_LOW   = 0x0C;
const uint32_t CHAN_CHAINADDR_HIGH  = 0x10;
const uint32_t CHAN_COMMAND         = 0x14;
const uint32_t CHAN_CMPLNADDR       = 0x18;
const uint32_t CHAN_CMPLNADDR_LOW   = 0x18;
const uint32_t CHAN_CMPLNADDR_HIGH  = 0x1C;
const uint32_t CHAN_ERROR           = 0x28;


const uint32_t DESC_CTRL_INT_GEN    = 0x00000001;
const uint32_t DESC_CTRL_SRC_SN     = 0x00000002;
const uint32_t DESC_CTRL_DST_SN     = 0x00000004;
const uint32_t DESC_CTRL_CP_STS     = 0x00000008;
const uint32_t DESC_CTRL_FRAME      = 0x00000010;
const uint32_t DESC_CTRL_NULL       = 0x00000020;

struct DmaDesc
{
    uint32_t len;
    uint32_t command;
    Addr src;
    Addr dest;
    Addr next;
    uint64_t reserved1;
    uint64_t reserved2;
    uint64_t user1;
    uint64_t user2;
};

#define ADD_FIELD8(NAME, OFFSET, BITS) \
    inline uint8_t NAME() { return bits(_data, OFFSET+BITS-1, OFFSET); } \
    inline void NAME(uint8_t d) { replaceBits(_data, OFFSET+BITS-1, OFFSET,d); }

#define ADD_FIELD16(NAME, OFFSET, BITS) \
    inline uint16_t NAME() { return bits(_data, OFFSET+BITS-1, OFFSET); } \
    inline void NAME(uint16_t d) { replaceBits(_data, OFFSET+BITS-1, OFFSET,d); }

#define ADD_FIELD32(NAME, OFFSET, BITS) \
    inline uint32_t NAME() { return bits(_data, OFFSET+BITS-1, OFFSET); } \
    inline void NAME(uint32_t d) { replaceBits(_data, OFFSET+BITS-1, OFFSET,d); }

#define ADD_FIELD64(NAME, OFFSET, BITS) \
    inline uint64_t NAME() { return bits(_data, OFFSET+BITS-1, OFFSET); } \
    inline void NAME(uint64_t d) { replaceBits(_data, OFFSET+BITS-1, OFFSET,d); }

template<class T>
struct Reg
{
    T _data;
    T operator()() { return _data; }
    const Reg<T> &operator=(T d) { _data = d; return *this;}
    bool operator==(T d) { return d == _data; }
    void operator()(T d) { _data = d; }
    Reg() { _data = 0; }
    void serialize(CheckpointOut &cp) const
    {
        SERIALIZE_SCALAR(_data);
    }
    void unserialize(CheckpointIn &cp)
    {
        UNSERIALIZE_SCALAR(_data);
    }
};


struct Regs : public Serializable
{
    uint8_t chanCount;
    uint8_t xferCap;

    struct INTRCTRL : public Reg<uint8_t>
    {
        // 0x03
        using Reg<uint8_t>::operator =;
        ADD_FIELD8(master_int_enable,0,1);
        ADD_FIELD8(interrupt_status,1,1);
        ADD_FIELD8(interrupt,2,1);
    };
    INTRCTRL intrctrl;

    uint32_t attnStatus; // Read clears

    void serialize(CheckpointOut &cp) const override
    {
        SERIALIZE_SCALAR(chanCount);
        SERIALIZE_SCALAR(xferCap);
        paramOut(cp, "intrctrl", intrctrl._data);
        SERIALIZE_SCALAR(attnStatus);
    }

    void unserialize(CheckpointIn &cp) override
    {
        UNSERIALIZE_SCALAR(chanCount);
        UNSERIALIZE_SCALAR(xferCap);
        paramIn(cp, "intrctrl", intrctrl._data);
        UNSERIALIZE_SCALAR(attnStatus);
    }

};

struct ChanRegs : public Serializable
{
    struct CHANCTRL : public Reg<uint16_t>
    {
        // channelX + 0x00
        using Reg<uint16_t>::operator =;
        ADD_FIELD16(interrupt_disable,0,1);
        ADD_FIELD16(error_completion_enable, 2,1);
        ADD_FIELD16(any_error_abort_enable,3,1);
        ADD_FIELD16(error_int_enable,4,1);
        ADD_FIELD16(desc_addr_snoop_control,5,1);
        ADD_FIELD16(in_use, 8,1);
    };
    CHANCTRL ctrl;

    struct CHANSTS : public Reg<uint64_t>
    {
        // channelX + 0x04
        ADD_FIELD64(dma_transfer_status, 0, 3);
        ADD_FIELD64(unaffiliated_error, 3, 1);
        ADD_FIELD64(soft_error, 4, 1);
        ADD_FIELD64(compl_desc_addr, 6, 58);
    };
    CHANSTS status;

    uint64_t descChainAddr;

    struct CHANCMD : public Reg<uint8_t>
    {
        // channelX + 0x14
        ADD_FIELD8(start_dma,0,1);
        ADD_FIELD8(append_dma,1,1);
        ADD_FIELD8(suspend_dma,2,1);
        ADD_FIELD8(abort_dma,3,1);
        ADD_FIELD8(resume_dma,4,1);
        ADD_FIELD8(reset_dma,5,1);
    };
    CHANCMD command;

    uint64_t completionAddr;

    struct CHANERR : public Reg<uint32_t>
    {
        // channel X + 0x28
        ADD_FIELD32(source_addr_error,0,1);
        ADD_FIELD32(dest_addr_error,1,1);
        ADD_FIELD32(ndesc_addr_error,2,1);
        ADD_FIELD32(desc_error,3,1);
        ADD_FIELD32(chain_addr_error,4,1);
        ADD_FIELD32(chain_cmd_error,5,1);
        ADD_FIELD32(chipset_parity_error,6,1);
        ADD_FIELD32(dma_parity_error,7,1);
        ADD_FIELD32(read_data_error,8,1);
        ADD_FIELD32(write_data_error,9,1);
        ADD_FIELD32(desc_control_error,10,1);
        ADD_FIELD32(desc_len_error,11,1);
        ADD_FIELD32(completion_addr_error,12,1);
        ADD_FIELD32(interrupt_config_error,13,1);
        ADD_FIELD32(soft_error,14,1);
        ADD_FIELD32(unaffiliated_error,15,1);
    };
    CHANERR error;

    void serialize(CheckpointOut &cp) const override
    {
        paramOut(cp, "ctrl", ctrl._data);
        paramOut(cp, "status", status._data);
        SERIALIZE_SCALAR(descChainAddr);
        paramOut(cp, "command", command._data);
        SERIALIZE_SCALAR(completionAddr);
        paramOut(cp, "error", error._data);
    }

    void unserialize(CheckpointIn &cp) override
    {
        paramIn(cp, "ctrl", ctrl._data);
        paramIn(cp, "status", status._data);
        UNSERIALIZE_SCALAR(descChainAddr);
        paramIn(cp, "command", command._data);
        UNSERIALIZE_SCALAR(completionAddr);
        paramIn(cp, "error", error._data);
    }


};

} // namespace copy_engine_reg
} // namespace gem5
