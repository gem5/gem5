/*
 * The Clear BSD License
 *
 * Copyright (c) 2015-2021 Arm Limited.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *      * Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARM_CHI_PHASE_H
#define ARM_CHI_PHASE_H

#include <stdint.h>

#include <ARM/TLM/arm_tlm_helpers.h>

namespace ARM
{
namespace CHI
{

/**
 * Communication phases for CHI.
 */

enum Channel
{
    /* Real CHI channels. */
    CHANNEL_REQ  = 0x0,
    CHANNEL_SNP  = 0x1,
    CHANNEL_RSP  = 0x2,
    CHANNEL_DAT  = 0x3
};

enum ReqOpcodeEnum
{
    REQ_OPCODE_REQ_LCRD_RETURN          = 0x00,
    REQ_OPCODE_READ_SHARED              = 0x01,
    REQ_OPCODE_READ_CLEAN               = 0x02,
    REQ_OPCODE_READ_ONCE                = 0x03,
    REQ_OPCODE_READ_NO_SNP              = 0x04,
    REQ_OPCODE_PCRD_RETURN              = 0x05,
    REQ_OPCODE_READ_UNIQUE              = 0x07,
    REQ_OPCODE_CLEAN_SHARED             = 0x08,
    REQ_OPCODE_CLEAN_INVALID            = 0x09,
    REQ_OPCODE_MAKE_INVALID             = 0x0A,
    REQ_OPCODE_CLEAN_UNIQUE             = 0x0B,
    REQ_OPCODE_MAKE_UNIQUE              = 0x0C,
    REQ_OPCODE_EVICT                    = 0x0D,
    REQ_OPCODE_EO_BARRIER               = 0x0E,
    REQ_OPCODE_EC_BARRIER               = 0x0F,
    REQ_OPCODE_READ_NO_SNP_SEP          = 0x11,
    REQ_OPCODE_CLEAN_SHARED_PERSIST_SEP = 0x13,
    REQ_OPCODE_DVM_OP                   = 0x14,
    REQ_OPCODE_WRITE_EVICT_FULL         = 0x15,
    REQ_OPCODE_WRITE_CLEAN_PTL          = 0x16,
    REQ_OPCODE_WRITE_CLEAN_FULL         = 0x17,
    REQ_OPCODE_WRITE_UNIQUE_PTL         = 0x18,
    REQ_OPCODE_WRITE_UNIQUE_FULL        = 0x19,
    REQ_OPCODE_WRITE_BACK_PTL           = 0x1A,
    REQ_OPCODE_WRITE_BACK_FULL          = 0x1B,
    REQ_OPCODE_WRITE_NO_SNP_PTL         = 0x1C,
    REQ_OPCODE_WRITE_NO_SNP_FULL        = 0x1D,

    REQ_OPCODE_WRITE_UNIQUE_FULL_STASH  = 0x20,
    REQ_OPCODE_WRITE_UNIQUE_PTL_STASH   = 0x21,
    REQ_OPCODE_STASH_ONCE_SHARED        = 0x22,
    REQ_OPCODE_STASH_ONCE_UNIQUE        = 0x23,

    REQ_OPCODE_READ_ONCE_CLEAN_INVALID  = 0x24,
    REQ_OPCODE_READ_ONCE_MAKE_INVALID   = 0x25,
    REQ_OPCODE_READ_NOT_SHARED_DIRTY    = 0x26,
    REQ_OPCODE_CLEAN_SHARED_PERSIST     = 0x27,

    REQ_OPCODE_ATOMIC_STORE_ADD         = 0x28,
    REQ_OPCODE_ATOMIC_STORE_CLR         = 0x29,
    REQ_OPCODE_ATOMIC_STORE_EOR         = 0x2A,
    REQ_OPCODE_ATOMIC_STORE_SET         = 0x2B,
    REQ_OPCODE_ATOMIC_STORE_SMAX        = 0x2C,
    REQ_OPCODE_ATOMIC_STORE_SMIN        = 0x2D,
    REQ_OPCODE_ATOMIC_STORE_UMAX        = 0x2E,
    REQ_OPCODE_ATOMIC_STORE_UMIN        = 0x2F,

    REQ_OPCODE_ATOMIC_LOAD_ADD          = 0x30,
    REQ_OPCODE_ATOMIC_LOAD_CLR          = 0x31,
    REQ_OPCODE_ATOMIC_LOAD_EOR          = 0x32,
    REQ_OPCODE_ATOMIC_LOAD_SET          = 0x33,
    REQ_OPCODE_ATOMIC_LOAD_SMAX         = 0x34,
    REQ_OPCODE_ATOMIC_LOAD_SMIN         = 0x35,
    REQ_OPCODE_ATOMIC_LOAD_UMAX         = 0x36,
    REQ_OPCODE_ATOMIC_LOAD_UMIN         = 0x37,

    REQ_OPCODE_ATOMIC_SWAP              = 0x38,
    REQ_OPCODE_ATOMIC_COMPARE           = 0x39,
    REQ_OPCODE_PREFETCH_TGT             = 0x3A,

    REQ_OPCODE_MAKE_READ_UNIQUE         = 0x41,
    REQ_OPCODE_WRITE_EVICT_OR_EVICT     = 0x42,
    REQ_OPCODE_WRITE_UNIQUE_ZERO        = 0x43,
    REQ_OPCODE_WRITE_NO_SNP_ZERO        = 0x44,
    REQ_OPCODE_STASH_ONCE_SEP_SHARED    = 0x47,
    REQ_OPCODE_STASH_ONCE_SEP_UNIQUE    = 0x48,
    REQ_OPCODE_READ_PREFER_UNIQUE       = 0x4C,

    REQ_OPCODE_WRITE_NO_SNP_FULL_CLEAN_SH           = 0x50,
    REQ_OPCODE_WRITE_NO_SNP_FULL_CLEAN_INV          = 0x51,
    REQ_OPCODE_WRITE_NO_SNP_FULL_CLEAN_SH_PER_SEP   = 0x52,

    REQ_OPCODE_WRITE_UNIQUE_FULL_CLEAN_SH           = 0x54,
    REQ_OPCODE_WRITE_UNIQUE_FULL_CLEAN_SH_PER_SEP   = 0x56,

    REQ_OPCODE_WRITE_BACK_FULL_CLEAN_SH             = 0x58,
    REQ_OPCODE_WRITE_BACK_FULL_CLEAN_INV            = 0x59,
    REQ_OPCODE_WRITE_BACK_FULL_CLEAN_SH_PER_SEP     = 0x5A,

    REQ_OPCODE_WRITE_CLEAN_FULL_CLEAN_SH            = 0x5C,
    REQ_OPCODE_WRITE_CLEAN_FULL_CLEAN_SH_PER_SEP    = 0x5E,

    REQ_OPCODE_WRITE_NO_SNP_PTL_CLEAN_SH            = 0x60,
    REQ_OPCODE_WRITE_NO_SNP_PTL_CLEAN_INV           = 0x61,
    REQ_OPCODE_WRITE_NO_SNP_PTL_CLEAN_SH_PER_SEP    = 0x62,

    REQ_OPCODE_WRITE_UNIQUE_PTL_CLEAN_SH            = 0x64,
    REQ_OPCODE_WRITE_UNIQUE_PTL_CLEAN_SH_PER_SEP    = 0x66,

    REQ_OPCODE_MASK = 0xFF
};

enum SnpOpcodeEnum
{
    SNP_OPCODE_SNP_LCRD_RETURN          = 0x00,
    SNP_OPCODE_SNP_SHARED               = 0x01,
    SNP_OPCODE_SNP_CLEAN                = 0x02,
    SNP_OPCODE_SNP_ONCE                 = 0x03,
    SNP_OPCODE_SNP_NOT_SHARED_DIRTY     = 0x04,
    SNP_OPCODE_SNP_UNIQUE_STASH         = 0x05,
    SNP_OPCODE_SNP_MAKE_INVALID_STASH   = 0x06,
    SNP_OPCODE_SNP_UNIQUE               = 0x07,
    SNP_OPCODE_SNP_CLEAN_SHARED         = 0x08,
    SNP_OPCODE_SNP_CLEAN_INVALID        = 0x09,
    SNP_OPCODE_SNP_MAKE_INVALID         = 0x0A,
    SNP_OPCODE_SNP_STASH_UNIQUE         = 0x0B,
    SNP_OPCODE_SNP_STASH_SHARED         = 0x0C,
    SNP_OPCODE_SNP_DVM_OP               = 0x0D,

    SNP_OPCODE_SNP_QUERY                = 0x10,
    SNP_OPCODE_SNP_SHARED_FWD           = 0x11,
    SNP_OPCODE_SNP_CLEAN_FWD            = 0x12,
    SNP_OPCODE_SNP_ONCE_FWD             = 0x13,
    SNP_OPCODE_SNP_NOT_SHARED_DIRTY_FWD = 0x14,
    SNP_OPCODE_SNP_PREFER_UNIQUE        = 0x15,
    SNP_OPCODE_SNP_PREFER_UNIQUE_FWD    = 0x16,
    SNP_OPCODE_SNP_UNIQUE_FWD           = 0x17,

    SNP_OPCODE_MASK = 0xFF
};

enum RspOpcodeEnum
{
    RSP_OPCODE_RSP_LCRD_RETURN          = 0x00,
    RSP_OPCODE_SNP_RESP                 = 0x01,
    RSP_OPCODE_COMP_ACK                 = 0x02,
    RSP_OPCODE_RETRY_ACK                = 0x03,
    RSP_OPCODE_COMP                     = 0x04,
    RSP_OPCODE_COMP_DBID_RESP           = 0x05,
    RSP_OPCODE_DBID_RESP                = 0x06,
    RSP_OPCODE_PCRD_GRANT               = 0x07,
    RSP_OPCODE_READ_RECEIPT             = 0x08,
    RSP_OPCODE_SNP_RESP_FWDED           = 0x09,
    RSP_OPCODE_TAG_MATCH                = 0x0A,
    RSP_OPCODE_RESP_SEP_DATA            = 0x0B,
    RSP_OPCODE_PERSIST                  = 0x0C,
    RSP_OPCODE_COMP_PERSIST             = 0x0D,
    RSP_OPCODE_DBID_RESP_ORD            = 0x0E,
    RSP_OPCODE_STASH_DONE               = 0x10,
    RSP_OPCODE_COMP_STASH_DONE          = 0x11,
    RSP_OPCODE_COMP_CMO                 = 0x14,

    RSP_OPCODE_MASK = 0xFF
};

enum DatOpcodeEnum
{
    DAT_OPCODE_DAT_LCRD_RETURN          = 0x00,
    DAT_OPCODE_SNP_RESP_DATA            = 0x01,
    DAT_OPCODE_COPY_BACK_WR_DATA        = 0x02,
    DAT_OPCODE_NON_COPY_BACK_WR_DATA    = 0x03,
    DAT_OPCODE_COMP_DATA                = 0x04,
    DAT_OPCODE_SNP_RESP_DATA_PTL        = 0x05,
    DAT_OPCODE_SNP_RESP_DATA_FWDED      = 0x06,
    DAT_OPCODE_WRITE_DATA_CANCEL        = 0x07,
    DAT_OPCODE_DATA_SEP_RESP            = 0x0B,
    DAT_OPCODE_NCB_WR_DATA_COMP_ACK     = 0x0C,

    DAT_OPCODE_MASK = 0xFF
};

typedef TLM::EnumWrapper<ReqOpcodeEnum, uint8_t> ReqOpcode;
typedef TLM::EnumWrapper<SnpOpcodeEnum, uint8_t> SnpOpcode;
typedef TLM::EnumWrapper<RspOpcodeEnum, uint8_t> RspOpcode;
typedef TLM::EnumWrapper<DatOpcodeEnum, uint8_t> DatOpcode;

/** CHI Resp field. */
enum Resp
{
    RESP_I     = 0,
    RESP_SC    = 1,
    RESP_UC    = 2,
    RESP_UD    = 2,
    RESP_SD    = 3,
    RESP_I_PD  = 4,
    RESP_SC_PD = 5,
    RESP_UC_PD = 6,
    RESP_UD_PD = 6,
    RESP_SD_PD = 7
};

/** CHI Order field. */
enum Order
{
    ORDER_NO_ORDER         = 0,
    ORDER_REQUEST_ACCEPTED = 1,
    ORDER_REQUEST_ORDER    = 2,
    ORDER_ENDPOINT_ORDER   = 3
};

/** CHI RespErr field. */
enum RespErr
{
    RESP_ERR_OK    = 0,
    RESP_ERR_EXOK  = 1,
    RESP_ERR_DERR  = 2,
    RESP_ERR_NDERR = 3
};

/** CHI TagOp field. */
enum TagOp
{
    TAG_OP_INVALID  = 0,
    TAG_OP_TRANSFER = 1,
    TAG_OP_UPDATE   = 2,
    TAG_OP_MATCH    = 3,
    TAG_OP_FETCH    = 3
};

class Phase
{
public:
    uint16_t src_id;
    uint16_t tgt_id;

    union
    {
        uint16_t stash_nid;
        uint16_t return_nid;
        uint16_t slc_rep_hint;
        uint16_t fwd_nid;
        uint16_t home_nid;
    };

    uint16_t txn_id;

    struct StashLPID
    {
        uint8_t value : 7;
        bool valid    : 1;
    };

    union
    {
        uint16_t dbid;
        uint16_t return_txn_id;
        uint16_t fwd_txn_id;
        uint8_t vmid_ext;
        StashLPID stash_lpid;
    };

    union
    {
        ReqOpcode req_opcode;
        SnpOpcode snp_opcode;
        RspOpcode rsp_opcode;
        DatOpcode dat_opcode;
        uint8_t raw_opcode;
    };

    Channel channel     : 2;
    uint8_t sub_channel : 1;
    bool lcrd           : 1;

    Order order         : 2;
    bool snp_attr       : 1;
    bool allow_retry    : 1;

    uint8_t data_id     : 2;
    Resp resp           : 3;
    Resp fwd_state      : 3;

    RespErr resp_err    : 2;
    uint8_t c_busy      : 3;
    TagOp tag_op        : 2;
    bool do_dwt         : 1;

    bool exp_comp_ack   : 1;

    uint8_t dummy1      : 7;

    uint8_t pcrd_type   : 4;
    uint8_t qos         : 4;

public:
    Phase():
        src_id(0),
        tgt_id(0),
        stash_nid(0),
        txn_id(0),
        dbid(0),
        raw_opcode(0),
        channel(CHANNEL_REQ),
        sub_channel(0),
        lcrd(false),
        order(ORDER_NO_ORDER),
        snp_attr(false),
        allow_retry(true),
        data_id(0),
        resp(RESP_I),
        fwd_state(RESP_I),
        resp_err(RESP_ERR_OK),
        c_busy(0),
        tag_op(TAG_OP_INVALID),
        do_dwt(false),
        exp_comp_ack(false),
        dummy1(0),
        pcrd_type(0),
        qos(0)
    {}
};

}
}

#endif /* ARM_CHI_PHASE_H */
