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

#ifndef ARM_AXI4_PAYLOAD_H
#define ARM_AXI4_PAYLOAD_H

#include <stdint.h>
#include <cstddef>
#include <iostream>

#include "arm_tlm_helpers.h"

namespace ARM
{
/** AMBA AXI protocol support. */
namespace AXI4
{

#define ARM_AXI4_TLM_API_VERSION ARM_AXI4_TLM_API_1
struct ARM_AXI4_TLM_API_VERSION{ARM_AXI4_TLM_API_VERSION();};
static ARM_AXI4_TLM_API_VERSION api_version_check;

#define ARM_AXI_TLM_PAYLOAD_HAS_ISSUE_H_FIELDS 1
struct ARM_AXI_TLM_PAYLOAD_SUPPORTS_ISSUE_H{ARM_AXI_TLM_PAYLOAD_SUPPORTS_ISSUE_H();};
static ARM_AXI_TLM_PAYLOAD_SUPPORTS_ISSUE_H arm_axi_tlm_payload_supports_issue_h;

class PayloadExtensionManager;

/**
 * Transaction command similar to tlm::tlm_command. For AXI4, the command
 * corresponds to the AXI4 channel on which a payload was first presented:
 * COMMAND_READ for AR, COMMAND_WRITE for AW or W, COMMAND_SNOOP for AC.
 */
enum CommandEnum
{
    COMMAND_READ  = 0,
    COMMAND_WRITE = 1,
    COMMAND_SNOOP = 2,

    COMMAND_MASK = 0xFF
};

/**
 * AXI4 SIZE field with the same element values as AXI4. The element names
 * correspond to the beat data size in bytes and the element values are the
 * log_2 of that size.
 */
enum SizeEnum
{
    SIZE_1   = 0,
    SIZE_2   = 1,
    SIZE_4   = 2,
    SIZE_8   = 3,
    SIZE_16  = 4,
    SIZE_32  = 5,
    SIZE_64  = 6,
    SIZE_128 = 7,

    SIZE_MASK = 0xFF
};

/** AXI4 BURST field with the same element values as AXI4. */
enum BurstEnum
{
    BURST_FIXED = 0,
    BURST_INCR  = 1,
    BURST_WRAP  = 2,

    BURST_MASK = 0xFF
};

/** AXI4 LOCK field with the same element values as AXI4. */
enum LockEnum
{
    LOCK_NORMAL    = 0,
    LOCK_EXCLUSIVE = 1,
    LOCK_LOCKED    = 2, /* AXI3 only. */

    LOCK_MASK = 0xFF
};

/**
 * AXI4 CACHE field with the same element values as AXI4. CacheEnum values can
 * also be built using the CacheBitEnum bit flags.
 */
enum CacheEnum
{
    /**
     * 'Canonical' CACHE values covering the whole allowable range. The names
     * have been chosen to match 'other allocate/modifiable' terminology rather
     * than older AXI 'write/read allocate/cacheable' terminology. See
     * CacheBitEnum for bitfield values.
     */

    /** For writes. */
    CACHE_AW_NA_NOA_NM_NB = 0x0,
    CACHE_AW_NA_NOA_NM_B  = 0x1,
    CACHE_AW_NA_NOA_M_NB  = 0x2,
    CACHE_AW_NA_NOA_M_B   = 0x3,
    CACHE_AW_NA_OA_NM_NB  = 0x4,
    CACHE_AW_NA_OA_NM_B   = 0x5,
    CACHE_AW_NA_OA_M_NB   = 0x6,
    CACHE_AW_NA_OA_M_B    = 0x7,
    CACHE_AW_A_NOA_NM_NB  = 0x8,
    CACHE_AW_A_NOA_NM_B   = 0x9,
    CACHE_AW_A_NOA_M_NB   = 0xA,
    CACHE_AW_A_NOA_M_B    = 0xB,
    CACHE_AW_A_OA_NM_NB   = 0xC,
    CACHE_AW_A_OA_NM_B    = 0xD,
    CACHE_AW_A_OA_M_NB    = 0xE,
    CACHE_AW_A_OA_M_B     = 0xF,

    /** For reads. */
    CACHE_AR_NOA_NA_NM_NB = 0x0,
    CACHE_AR_NOA_NA_NM_B  = 0x1,
    CACHE_AR_NOA_NA_M_NB  = 0x2,
    CACHE_AR_NOA_NA_M_B   = 0x3,
    CACHE_AR_NOA_A_NM_NB  = 0x4,
    CACHE_AR_NOA_A_NM_B   = 0x5,
    CACHE_AR_NOA_A_M_NB   = 0x6,
    CACHE_AR_NOA_A_M_B    = 0x7,
    CACHE_AR_OA_NA_NM_NB  = 0x8,
    CACHE_AR_OA_NA_NM_B   = 0x9,
    CACHE_AR_OA_NA_M_NB   = 0xA,
    CACHE_AR_OA_NA_M_B    = 0xB,
    CACHE_AR_OA_A_NM_NB   = 0xC,
    CACHE_AR_OA_A_NM_B    = 0xD,
    CACHE_AR_OA_A_M_NB    = 0xE,
    CACHE_AR_OA_A_M_B     = 0xF,

    /** For both. */
    CACHE_NA_NOA_NM_NB    = 0x0,
    CACHE_NA_NOA_NM_B     = 0x1,
    CACHE_NA_NOA_M_NB     = 0x2,
    CACHE_NA_NOA_M_B      = 0x3,
    CACHE_A_OA_NM_NB      = 0xC,
    CACHE_A_OA_NM_B       = 0xD,
    CACHE_A_OA_M_NB       = 0xE,
    CACHE_A_OA_M_B        = 0xF,

    CACHE_MASK = 0xFF,

    /** Cache values based on named memory types. */

    /** For writes. */
    CACHE_AW_DEVICE_NB         = CACHE_AW_NA_NOA_NM_NB, /* 0x0 */
    CACHE_AW_DEVICE_B          = CACHE_AW_NA_NOA_NM_B,  /* 0x1 */
    CACHE_AW_NORMAL_NC_NB      = CACHE_AW_NA_NOA_M_NB,  /* 0x2 */
    CACHE_AW_NORMAL_NC_B       = CACHE_AW_NA_NOA_M_B,   /* 0x3 */
    CACHE_AW_WRITE_THROUGH_NA  = CACHE_AW_NA_OA_M_NB,   /* 0x6 */
    CACHE_AW_WRITE_THROUGH_RA  = CACHE_AW_NA_OA_M_NB,   /* 0x6 */
    CACHE_AW_WRITE_THROUGH_WA  = CACHE_AW_A_OA_M_NB,    /* 0xE */
    CACHE_AW_WRITE_THROUGH_RWA = CACHE_AW_A_OA_M_NB,    /* 0xE */
    CACHE_AW_WRITE_BACK_NA     = CACHE_AW_NA_OA_M_B,    /* 0x7 */
    CACHE_AW_WRITE_BACK_RA     = CACHE_AW_NA_OA_M_B,    /* 0x7 */
    CACHE_AW_WRITE_BACK_WA     = CACHE_AW_A_OA_M_B,     /* 0xF */
    CACHE_AW_WRITE_BACK_RWA    = CACHE_AW_A_OA_M_B,     /* 0xF */

    /** For reads */
    CACHE_AR_DEVICE_NB         = CACHE_AR_NOA_NA_NM_NB, /* 0x0 */
    CACHE_AR_DEVICE_B          = CACHE_AR_NOA_NA_NM_B,  /* 0x1 */
    CACHE_AR_NORMAL_NC_NB      = CACHE_AR_NOA_NA_M_NB,  /* 0x2 */
    CACHE_AR_NORMAL_NC_B       = CACHE_AR_NOA_NA_M_B,   /* 0x3 */
    CACHE_AR_WRITE_THROUGH_NA  = CACHE_AR_OA_NA_M_NB,   /* 0xA */
    CACHE_AR_WRITE_THROUGH_RA  = CACHE_AR_OA_A_M_NB,    /* 0xE */
    CACHE_AR_WRITE_THROUGH_WA  = CACHE_AR_OA_NA_M_NB,   /* 0xA */
    CACHE_AR_WRITE_THROUGH_RWA = CACHE_AR_OA_A_M_NB,    /* 0xE */
    CACHE_AR_WRITE_BACK_NA     = CACHE_AR_OA_NA_M_B,    /* 0xB */
    CACHE_AR_WRITE_BACK_RA     = CACHE_AR_OA_A_M_B,     /* 0xF */
    CACHE_AR_WRITE_BACK_WA     = CACHE_AR_OA_NA_M_B,    /* 0xB */
    CACHE_AR_WRITE_BACK_RWA    = CACHE_AR_OA_A_M_B,     /* 0xF */

    /** For both. */
    CACHE_DEVICE_NB            = CACHE_NA_NOA_NM_NB,    /* 0x0 */
    CACHE_DEVICE_B             = CACHE_NA_NOA_NM_B,     /* 0x1 */
    CACHE_NORMAL_NC_NB         = CACHE_NA_NOA_M_NB,     /* 0x2 */
    CACHE_NORMAL_NC_B          = CACHE_NA_NOA_M_B       /* 0x3 */
};

/** Bit flags within CacheEnum. */
enum CacheBitEnum
{
    /** Write flags. */
    CACHE_AW_A  = 8, /* Write: write allocate */
    CACHE_AW_OA = 4, /* Write: 'other' (i.e. read) allocate */
    CACHE_AW_M  = 2, /* Write: modifiable */
    CACHE_AW_B  = 1, /* Write: bufferable */

    /** Read flags. */
    CACHE_AR_OA = 8, /* Read: 'other' (i.e. write) allocate */
    CACHE_AR_A  = 4, /* Read: read allocate */
    CACHE_AR_M  = 2, /* Read: modifiable */
    CACHE_AR_B  = 1, /* Read: bufferable */

    CACHE_BIT_MASK = 0xFF
};

/**
 * AXI4 PROT field with the same element values as AXI4. See ProtBitEnum for
 * bitfield values.
 */
enum ProtEnum
{
    PROT_D_S_UP  = 0,
    PROT_D_S_P   = 1,
    PROT_D_NS_UP = 2,
    PROT_D_NS_P  = 3,
    PROT_I_S_UP  = 4,
    PROT_I_S_P   = 5,
    PROT_I_NS_UP = 6,
    PROT_I_NS_P  = 7,

    PROT_MASK = 0xFF
};

/** Bit flags within ProtEnum. */
enum ProtBitEnum
{
    PROT_P  = 1, /* Privileged */
    PROT_NS = 2, /* Not secure */
    PROT_I  = 4, /* Instruction fetch */

    PROT_BIT_MASK = 0xFF
};

/**
 * AXI4 and ACE RESP with the same element values as AXI4 (except
 * RESP_INCONSISTENT).
 */
enum RespEnum
{
    RESP_OKAY       = 0,
    RESP_EXOKAY     = 1,
    RESP_SLVERR     = 2,
    RESP_DECERR     = 3,
    RESP_PREFETCHED = 4,
    RESP_TRANSFAULT = 5,

    /**
     * Extra element (not found in AXI) to mark transaction responses where the
     * individual beat responses are not consistent with each other.
     */
    RESP_INCONSISTENT = 0x80,

    RESP_MASK = 0xFF
};

/** Bit fields within RESP. */
enum RespBitEnum
{
    RESP_DATA_TRANSFER = 0x01,
    RESP_ERROR         = 0x02,
    RESP_DIRTY         = 0x04,
    RESP_SHARED        = 0x08,
    RESP_WAS_UNIQUE    = 0x10,

    RESP_BIT_MASK = 0xFF,

    /**
     * A mask useful for extracting (using Resp::mask) the non-ACE response
     * bit fields.
     */
    RESP_R_BASE_MASK = RESP_DATA_TRANSFER | RESP_ERROR
};

/** AXI4 DOMAIN field with the same element values as AXI4. */
enum DomainEnum
{
    DOMAIN_NSH = 0,
    DOMAIN_ISH = 1,
    DOMAIN_OSH = 2,
    DOMAIN_SYS = 3,

    DOMAIN_MASK = 0xFF
};

/** AXI4 BAR field with the same element values as AXI4. */
enum BarEnum
{
    BAR_NORM   = 0,
    BAR_MEMBAR = 1,
    BAR_IGNORE = 2,
    BAR_SYNC   = 3,

    BAR_MASK = 0xFF
};

/** AXI5 Issue G CMO field with the same element values as AXI5. */
enum CmoEnum
{
    CMO_CLEAN_INVALID             = 0,
    CMO_CLEAN_SHARED              = 1,
    CMO_CLEAN_SHARED_PERSIST      = 2,
    CMO_CLEAN_SHARED_DEEP_PERSIST = 3,

    CMO_MASK = 0xFF
};

/** AXI4 SNOOP field with the same element values as AXI4. */
enum SnoopEnum
{
    /** Write snoop requests. */
    SNOOP_AW_WRITE_NO_SNOOP          = 0x0,
    SNOOP_AW_WRITE_UNIQUE            = 0x0,
    SNOOP_AW_WRITE_UNIQUE_PTL        = 0x0,
    SNOOP_AW_BARRIER                 = 0x0,
    SNOOP_AW_ATOMIC                  = 0x0,
    SNOOP_AW_WRITE_LINE_UNIQUE       = 0x1,
    SNOOP_AW_WRITE_UNIQUE_FULL       = 0x1,
    SNOOP_AW_WRITE_CLEAN             = 0x2,
    SNOOP_AW_WRITE_BACK              = 0x3,
    SNOOP_AW_EVICT                   = 0x4,
    SNOOP_AW_WRITE_EVICT             = 0x5,
    SNOOP_AW_CMO                     = 0x6,
    SNOOP_AW_WRITE_ZERO              = 0x7,
    SNOOP_AW_WRITE_UNIQUE_PTL_STASH  = 0x8,
    SNOOP_AW_WRITE_UNIQUE_FULL_STASH = 0x9,
    SNOOP_AW_WRITE_PTL_CMO           = 0xA,
    SNOOP_AW_WRITE_FULL_CMO          = 0xB,
    SNOOP_AW_STASH_ONCE_SHARED       = 0xC,
    SNOOP_AW_STASH_ONCE_UNIQUE       = 0xD,
    SNOOP_AW_STASH_TRANSLATION       = 0xE,
    SNOOP_AW_PREFETCH                = 0xF,

    /** Read snoop requests. */
    SNOOP_AR_READ_NO_SNOOP           = 0x0,
    SNOOP_AR_READ_ONCE               = 0x0,
    SNOOP_AR_BARRIER                 = 0x0,
    SNOOP_AR_READ_SHARED             = 0x1,
    SNOOP_AR_READ_CLEAN              = 0x2,
    SNOOP_AR_READ_NOT_SHARED_DIRTY   = 0x3,
    SNOOP_AR_READ_ONCE_CLEAN_INVALID = 0x4,
    SNOOP_AR_READ_ONCE_MAKE_INVALID  = 0x5,
    SNOOP_AR_READ_UNIQUE             = 0x7,
    SNOOP_AR_CLEAN_SHARED            = 0x8,
    SNOOP_AR_CLEAN_INVALID           = 0x9,
    SNOOP_AR_CLEAN_SHARED_PERSIST    = 0xA,
    SNOOP_AR_CLEAN_UNIQUE            = 0xB,
    SNOOP_AR_MAKE_UNIQUE             = 0xC,
    SNOOP_AR_MAKE_INVALID            = 0xD,
    SNOOP_AR_DVM_COMPLETE            = 0xE,
    SNOOP_AR_DVM_MESSAGE             = 0xF,

    /** Snoop address snoop types.   */
    SNOOP_AC_READ_ONCE               = 0x0,
    SNOOP_AC_READ_SHARED             = 0x1,
    SNOOP_AC_READ_CLEAN              = 0x2,
    SNOOP_AC_READ_NOT_SHARED_DIRTY   = 0x3,
    SNOOP_AC_READ_UNIQUE             = 0x7,
    SNOOP_AC_CLEAN_SHARED            = 0x8,
    SNOOP_AC_CLEAN_INVALID           = 0x9,
    SNOOP_AC_MAKE_INVALID            = 0xD,
    SNOOP_AC_DVM_COMPLETE            = 0xE,
    SNOOP_AC_DVM_MESSAGE             = 0xF,

    SNOOP_MASK = 0xFF
};

/** AXI4 ATOP field with the same element values as AXI4. */
enum AtopEnum
{
    ATOP_NON_ATOMIC    = 0x0,

    ATOP_STORE_ADD     = 0x10,
    ATOP_STORE_CLR     = 0x11,
    ATOP_STORE_EOR     = 0x12,
    ATOP_STORE_SET     = 0x13,
    ATOP_STORE_SMAX    = 0x14,
    ATOP_STORE_SMIN    = 0x15,
    ATOP_STORE_UMAX    = 0x16,
    ATOP_STORE_UMIN    = 0x17,

    ATOP_STORE_ADD_BE  = 0x18,
    ATOP_STORE_CLR_BE  = 0x19,
    ATOP_STORE_EOR_BE  = 0x1A,
    ATOP_STORE_SET_BE  = 0x1B,
    ATOP_STORE_SMAX_BE = 0x1C,
    ATOP_STORE_SMIN_BE = 0x1D,
    ATOP_STORE_UMAX_BE = 0x1E,
    ATOP_STORE_UMIN_BE = 0x1F,

    ATOP_LOAD_ADD      = 0x20,
    ATOP_LOAD_CLR      = 0x21,
    ATOP_LOAD_EOR      = 0x22,
    ATOP_LOAD_SET      = 0x23,
    ATOP_LOAD_SMAX     = 0x24,
    ATOP_LOAD_SMIN     = 0x25,
    ATOP_LOAD_UMAX     = 0x26,
    ATOP_LOAD_UMIN     = 0x27,

    ATOP_LOAD_ADD_BE   = 0x28,
    ATOP_LOAD_CLR_BE   = 0x29,
    ATOP_LOAD_EOR_BE   = 0x2A,
    ATOP_LOAD_SET_BE   = 0x2B,
    ATOP_LOAD_SMAX_BE  = 0x2C,
    ATOP_LOAD_SMIN_BE  = 0x2D,
    ATOP_LOAD_UMAX_BE  = 0x2E,
    ATOP_LOAD_UMIN_BE  = 0x2F,

    ATOP_SWAP          = 0x30,
    ATOP_COMPARE       = 0x31,

    ATOP_MASK = 0xFF
};

/** Bit fields within ATOP. */
enum AtopBitEnum
{
    ATOP_ENDIAN = 0x08,
    ATOP_STORE  = 0x10,
    ATOP_LOAD   = 0x20,

    ATOP_BIT_MASK = 0xFF
};

/** AXI4 MMUFLOW field with the same element values as AXI4. */
enum MmuFlowEnum
{
    MMU_FLOW_STALL    = 0,
    MMU_FLOW_ATST     = 1,
    MMU_FLOW_NO_STALL = 2,
    MMU_FLOW_PRI      = 3,

    MMU_FLOW_MASK = 0xFF
};

/** AXI4 AWTAGOP and ARTAGOP field with the same element values as AXI4. */
enum TagOpEnum
{
    TAG_OP_INVALID  = 0,
    TAG_OP_TRANSFER = 1,
    TAG_OP_UPDATE   = 2,
    TAG_OP_MATCH    = 3,

    TAP_OP_MASK = 0xFF
};

/**
 * Typedefs wrapping enumeration types for Payload data members in EnumWrapper
 * and BitEnumWrapper.
 */
typedef TLM::EnumWrapper<CommandEnum, uint8_t> Command;
typedef TLM::EnumWrapper<SizeEnum, uint8_t> Size;
typedef TLM::EnumWrapper<BurstEnum, uint8_t> Burst;
typedef TLM::EnumWrapper<LockEnum, uint8_t> Lock;
typedef TLM::BitEnumWrapper<CacheEnum, CacheBitEnum, uint8_t> Cache;
typedef TLM::BitEnumWrapper<ProtEnum, ProtBitEnum, uint8_t> Prot;
typedef TLM::BitEnumWrapper<RespEnum, RespBitEnum, uint8_t> Resp;
typedef TLM::EnumWrapper<DomainEnum, uint8_t> Domain;
typedef TLM::EnumWrapper<BarEnum, uint8_t> Bar;
typedef TLM::EnumWrapper<SnoopEnum, uint8_t> Snoop;
typedef TLM::EnumWrapper<CmoEnum, uint8_t> Cmo;
typedef TLM::BitEnumWrapper<AtopEnum, AtopBitEnum, uint8_t> Atop;
typedef TLM::EnumWrapper<MmuFlowEnum, uint8_t> MmuFlow;
typedef TLM::EnumWrapper<TagOpEnum, uint8_t> TagOp;

class Mpam
{
public:
    uint8_t mpam_ns;
    uint16_t part_id;
    uint8_t perf_mon_group;

    Mpam() :
        mpam_ns(0),
        part_id(0),
        perf_mon_group(0)
    {}

    Mpam(uint8_t mpam_ns_, uint16_t part_id_, uint8_t perf_mon_group_) :
        mpam_ns(mpam_ns_),
        part_id(part_id_),
        perf_mon_group(perf_mon_group_)
    {}
};

class MteTag
{
public:
    uint8_t tag     : 4;
    bool tag_update : 1;

    MteTag() :
        tag(0),
        tag_update(false)
    {}

    MteTag(uint8_t tag_, bool tag_update_) :
        tag(tag_),
        tag_update(tag_update_)
    {}
};

class PayloadData;
class PayloadPool;

/** AXI4/ACE transaction payload. */
class Payload
{
    friend class PayloadPool;
private:
    /**
     * Payload reference count. New payloads have a reference count of 1. The
     * reference count can be incremented/decremented with ref/unref. When
     * unref is called on a payload with refcount == 1, the payload will be
     * returned to the payload pool.
     */
    mutable unsigned refcount;

public:
    /**
     * Unique ID of this payload. Unique IDs are set when a payload is created
     * (using new_payload, clone, or descend) and are guarenteed to be unique
     * within a simulation regardless of reuse of a payload's memory.
     */
    uint64_t const uid;

    /**
     * 'Parent' payload from which this payload is descended or cloned. The
     * parent/child relationship can be used to chain payloads which form part
     * of the same logical transaction but which must exist as separate Payload
     * objects
     */
    Payload* const parent;

private:
    /**
     * Pointer to a possibly-shared data object. All access to payload data are
     * handled by member functions on the payload.
     */
    PayloadData* const payload_data;

private:
    /**
     * AXI4 address field with low address bits set appropriately for wrapping
     * burst transactions. Address is private as modifying the address could
     * invalidate the payload's understanding of its data's organization.
     */
    uint64_t address;

public:
    /** AXI4 ID field. */
    uint32_t id;

    /**
     * AXI4 LOCK, CACHE, PROT, QOS, REGION, and USER fields. All have the same
     * interpretation as in AXI4.
     */
    Lock lock;
    Cache cache;
    Prot prot;
    uint8_t qos;
    uint8_t region;
    uint64_t user;

    /**
     * ACE SNOOP, DOMAIN, and BAR, UNIQUE fields. All have the same
     * interpretation as in ACE.
     */
    Snoop snoop;
    Domain domain;
    Bar bar;
    Atop atop;
    uint8_t vmid_ext;

    uint16_t stash_nid;
    uint8_t stash_lpid;

    bool unique           : 1;
    bool stash_nid_valid  : 1; /* STASHNIDEN. */
    bool stash_lpid_valid : 1; /* STASHLPIDEN. */

    /* All data members past this point are part of the issue H release (or later). */

    bool idunq            : 1;
    bool chunk_en         : 1;
    bool mmu_sec_sid      : 1;
    bool mmu_ssid_v       : 1;

    /**
     * BTAGMATCH[0].  After a TAGMATCH phase has been seen, BTAGMATCH = { 0b1, tag_match }.
     */
    bool tag_match        : 1;

    TagOp tag_op;
    Cmo cmo;
    Mpam mpam;
    MmuFlow mmu_flow;
    uint8_t nsaid;

    uint32_t mmu_sid;
    uint32_t mmu_ssid;
    uint64_t loop;

private:
    /**
     * Create a new Payload with new payload data.
     * Used by descend and new_payload but cannot be publicly called.
     */
    Payload(PayloadData* payload_data, uint64_t address, uint64_t uid);

    /**
     * Create a new payload sharing payload data with the parent. Used by clone
     * but cannot be publicly called.
     */
    Payload(PayloadData* payload_data_, uint64_t address_,
        Payload* parent_, uint64_t uid);

    /** Forbid copy construction. */
    Payload(const Payload&);

    /** Forbid assignment. */
    Payload& operator= (const Payload&) { return *this; }

    /** Forbid public delete as Payloads are reference counted. */
    void operator delete (void* p);

    /** Forbid public stack-based objects. */
    ~Payload();

public:
    /** Increment reference count. */
    void ref() const;

    /** Decrement reference count. */
    void unref() const;

    /**
     * Create a new payload. Payload has no parent.
     * size, len, and burst have the values of the AXI4 fields SIZE, LEN, and
     * BURST for the transaction.
     * All payload data members not set using the arguments are reset to 0.
     */
    static Payload* new_payload(Command command, uint64_t address, Size size,
        uint8_t len, Burst burst = BURST_INCR);

    /**
     * Create a copy of a payload but share payload data with the parent 'this'.
     * All other data members copied from the parent.
     */
    Payload* clone();

    /**
     * Create a new Payload with new payload data with its parent set to 'this'.
     * size, len, and burst have the values of the AXI4 fields SIZE, LEN, and
     * BURST for the transaction. All data members not set directly from the
     * arguments are copied from the parent.
     */
    Payload* descend(Command command, uint64_t address_, Size size,
        uint8_t len, Burst burst = BURST_INCR);

    /**
     * Get a dummy payload for non payload TLM calls (QOSACCEPT etc).
     */
    static Payload* get_dummy();

    /** Get the length of a single beat's data in bytes. */
    std::size_t get_beat_data_length() const;

    /** Get the length of the transaction's data in bytes. */
    std::size_t get_data_length() const;

    /** Get the transaction's address including all of the low address bits. */
    uint64_t get_address() const;

    /**
     * Set the transaction's address. This function will call assert if the
     * alignment of the new address does not match that of the existing address.
     * This is to avoid invalidating the transaction's understanding of the
     * organization of its beat data (which may be stared with other Payload
     * objects).
     */
    void set_address(uint64_t new_address);

    /**
     * Get the address of the lowest address beat in this transaction ignoring
     * any wrapping.
     */
    uint64_t get_base_address() const;

    /**
     * Get the response value. This value can be set to RESP_INCONSISTENT if a
     * beat within the transaction did not have the same response as other
     * beats.
     */
    Resp get_resp() const;

    /** Set the response value. */
    void set_resp(Resp resp);

    /** Get the command value. */
    Command get_command() const;

    /** Get the AXI4 LEN burst length value (== number of beats - 1). */
    uint8_t get_len() const;

    /** Get the AXI4 SIZE beat element size. */
    Size get_size() const;

    /** Get the AXI4 BURST burst type. */
    Burst get_burst() const;

    /** Get the beat count. (== AXI4 LEN + 1). */
    unsigned get_beat_count() const;

    /** Get the completed beat count. */
    unsigned get_beats_complete() const;

    /** Get the atomic response length. */
    std::size_t get_atomic_response_length() const;

    /** Get the atmoic response beat count.*/
    unsigned get_atomic_response_beat_count() const;

    /** Get the atmoic response beat length.*/
    std::size_t get_atomic_response_beat_length() const;

    /**
     * Get the number of skipped chunks at the start of the transaction due
     * to unalignment.
     */
    unsigned get_unaligned_skipped_chunks() const;

    /**
     * Copy in the data for a whole read transaction from an array of
     * get_data_length() bytes. 'resp' is an optional array of per-beat
     * responses get_beat_count() Resps long.
     */
    void read_in(const uint8_t* data, Resp* resp = nullptr);

    /**
     * Copy out the data for a whole read transaction to an array of
     * get_data_length() bytes.
     */
    void read_out(uint8_t* data) const;

    /**
     * Copy out the beat response values for a whole read transaction to an
     * array get_beat_count() Resps long.
     */
    void read_out_resps(Resp* resp) const;

    /**
     * Copy in the data for a whole write transaction from an array
     * get_data_length() bytes long. An optional array of byte strobes
     * ceil(get_data_length() / 8.0) bytes long can be passed to select which
     * bytes to write. The strobes are organized as one bit per data byte with
     * the lowest bit of byte strobe[0] corresponding to the lowest byte of
     * data. If 'strobe' is null, all bytes are written.
     */
    void write_in(const uint8_t* data, const uint8_t* strobe = nullptr);

    /**
     * Copy out the data for a whole write transaction into an array
     * get_data_length() bytes long.
     */
    void write_out(uint8_t* data) const;

    /**
     * Copy out the strobes for a whole write transaction into an array
     * ceil(get_data_length() / 8.0) bytes long.
     */
    void write_out_strobes(uint8_t* strobe) const;

    /**
     * Copy in the data for a whole snoop transaction from an array
     * get_data_length() bytes long.
     */
    void snoop_in(const uint8_t* data);

    /**
     * Copy out the data for a whole snoop transaction into an array
     * get_data_length() bytes long.
     */
    void snoop_out(uint8_t* data) const;

    /**
     * Copy in the data for one beat of a read transaction from an array
     * get_beat_data_length() bytes long. The beat response will be set to 'resp'.
     */
    void read_in_beat(const uint8_t* data, Resp resp = RESP_OKAY);

    /**
     * Copy out the data for one beat of a read transaction into an array
     * get_beat_data_length() bytes long. The first beat of a transaction has
     * index 0. Only beats which have already been written into a payload can
     * be read out.
     */
    void read_out_beat(unsigned beat_index, uint8_t* data) const;

    /**
     * Copy in the data for one chunk of a read transaction from an array
     * 16 bytes long. The chunk response will be set to 'resp'.
     */
    void read_in_chunk(unsigned chunk_number, const uint8_t* data, Resp resp = RESP_OKAY);

    /**
     * Copy out the data for one chunk of a read transaction into an array
     * 16 bytes long. Only chunks which have already been written into a payload
     * can be read out.
     */
    void read_out_chunk(unsigned chunk_number, uint8_t* data) const;

    /**
     * Get the response for one beat of a read transaction. The first beat of a
     * transaction has index 0. Only beats which have already been written into
     * a payload can be copied out.
     */
    Resp read_out_beat_resp(unsigned beat_index) const;

    /**
     * Get the response for one chunk of a read transaction. Only chunks which
     * have already been written into a payload can be copied out.
     */
    Resp read_out_chunk_resp(unsigned chunk_number) const;

    /**
     * Copy in the data for one beat of a write transaction where the beat is
     * shorter than 64 bytes long (get_size() <= SIZE_64). 'strobe' encodes the
     * byte strobes for the beat with the lowest bit of strobe being the strobe
     * for data[0].
     */
    void write_in_beat(const uint8_t* data, uint64_t strobe);

    /**
     * Copy in the data for one beat of a write transaction. The write strobe is
     * passed as an array ceil(get_beat_data_length() / 8.0) bytes long with the same
     * strobe organization as for write_in but with the lowest strobe bit
     * corresponding to the *beat* data[0] rather than the whole transaction's
     * data.
     */
    void write_in_beat(const uint8_t* data, const uint8_t* strobe = nullptr);

    /**
     * Copy out the data for one beat of a write transaction to an array
     * get_beat_data_length() bytes long. The first beat of a transaction has
     * index 0. Only beats which have already been written into a payload can
     * be read out.
     */
    void write_out_beat(unsigned beat_index, uint8_t* data) const;

    /**
     * Get the strobe for one beat of a write transaction where the beat is
     * shorter than 64 bytes long (get_size() <= SIZE_64). The returned strobe
     * will be organized with the lowest bit corresponding to the lowest address
     * byte in the beat.
     */
    uint64_t write_out_beat_strobe(unsigned beat_index) const;

    /**
     * Copy out the strobes for one beat of a write transaction into an array
     * ceil(get_beat_data_length() / 8.0) bytes long. The first beat of a transaction
     * has index 0. Only beats which have already been written into a payload
     * can have their strobe copied out. The strobes are organized as one bit
     * per data byte with the lowest bit of byte strobe[0] corresponding to the
     * lowest byte of data.
     */
    void write_out_beat_strobe(unsigned beat_index, uint8_t* strobe) const;

    /**
     * Copy in the data for one beat of a snoop transaction from an array
     * get_beat_data_length() bytes long.
     */
    void snoop_in_beat(const uint8_t* data);

    /**
     * Copy out the data for one beat of a snoop transaction into an array
     * get_beat_data_length() bytes long.
     * The first beat of a transaction has index 0. Only beats which have
     * already been written into a payload can have their strobe copied out.
     */
    void snoop_out_beat(unsigned beat_index, uint8_t* data) const;

    /**
     * Copy in the data for a whole atomic transaction response from an array
     * get_data_length() bytes long (unless it is a AtomicCompare when the
     * response is get_data_length() / 2 bytes long ).
     */
    void read_in_atomic_response(const uint8_t* data);

    /**
     * Copy out the data for a whole atomic transaction response into an array
     * get_data_length() bytes long (unless it is a AtomicCompare when the
     * response is get_data_length() / 2 bytes long).
     */
    void read_out_atomic_response(uint8_t* data) const;

    /**
     * Copy in the data for a whole atomic transaction response from an array
     * get_data_length() bytes long (unless it is a AtomicCompare when the
     * response is get_data_length() / 2 bytes long ).
     */
    void read_in_atomic_response_beat(const uint8_t* data);

    /**
     * Copy out the data for a whole atomic transaction response into an array
     * get_data_length() bytes long (unless it is a AtomicCompare when the
     * response is get_data_length() / 2 bytes long).
     */
    void read_out_atomic_response_beat(unsigned beat_index, uint8_t* data) const;

    /**
     * Copy in the data for one beat of a read transaction supplied in raw
     * signal level format. 'width' is the width of the data channel in the
     * signal level implementation of the AXI4 port encoded as Size. 'data' is
     * an array (1 << 'width') bytes long.
     */
    void read_in_beat_raw(Size width, const uint8_t* data,
        Resp resp = RESP_OKAY);

    /**
     * Copy out the data for one beat of a read transaction supplied in raw
     * signal level format. 'width' is the width of the data channel in the
     * signal level implementation of the AXI4 port encoded as Size. 'data' is
     * an array (1 << 'width') bytes long. The first beat of a transaction has
     * index 0.  Only beats which have already been written into a payload can
     * be read out.
     */
    void read_out_beat_raw(Size width, unsigned beat_index,
        uint8_t* data) const;

    /**
     * Copy in the data for one beat of a read transaction supplied in raw
     * signal level format. 'width' is the width of the data channel in the
     * signal level implementation of the AXI4 port encoded as Size. 'data' is
     * an array (1 << 'width') bytes long.
     */
    void read_in_chunk_beat_raw(Size width, const uint8_t* data,
        unsigned chunk_number, unsigned chunk_strobe, Resp resp = RESP_OKAY);

    /**
     * Copy out the data for one beat of a read transaction supplied in raw
     * signal level format. 'width' is the width of the data channel in the
     * signal level implementation of the AXI4 port encoded as Size. 'data' is
     * an array (1 << 'width') bytes long. The first beat of a transaction has
     * index 0.  Only beats which have already been written into a payload can
     * be read out.
     */
    void read_out_chunk_beat_raw(Size width, unsigned chunk_number,
        unsigned chunk_strobe, uint8_t* data) const;

    /**
     * Copy in the data for one beat of a write transaction supplied in raw
     * signal level format where 'width' <= SIZE_64. 'width' is the width of the
     * data channel in the signal level implementation of the AXI4 port encoded
     * as Size. 'data' is an array (1 << 'width') bytes long. Strobes are
     * organized with the lowest bit being the strobe for byte data[0].
     */
    void write_in_beat_raw(Size width, const uint8_t* data, uint64_t strobe);

    /**
     * Copy in the data for one beat of a write transaction supplied in raw
     * signal level format. 'width' is the width of the data channel in the
     * signal level implementation of the AXI4 port encoded as Size. 'data' is
     * an array (1 << 'width') bytes long. The write strobe is passed as an
     * array ceil((1 << 'width') / 8.0) bytes long with the same strobe
     * organization as for write_in but with the lowest strobe bit corresponding
     * to the signal level beat data[0] rather than the whole transaction's
     * data.
     */
    void write_in_beat_raw(Size width, const uint8_t* data,
        const uint8_t* strobe);

    /**
     * Copy out the data for one beat of a write transaction supplied in raw
     * signal level format. 'width' is the width of the data channel in the
     * signal level implementation of the AXI4 port encoded as Size. 'data' is
     * an array (1 << 'width') bytes long. The first beat of a transaction has
     * index 0. Only beats which have already been written into a payload can
     * have their strobe copied out.
     */
    void write_out_beat_raw(Size width, unsigned beat_index,
        uint8_t* data) const;

    /**
     * Get the strobe for one beat of a write transaction supplied in raw signal
     * level format where 'width' <= SIZE_64. 'width' is the width of the data
     * channel in the signal level implementation of the AXI4 port encoded as
     * Size.  Strobes are organized with the lowest bit being the strobe for
     * byte data[0]. The first beat of a transaction has index 0. Only beats
     * which have already been written into a payload can have their strobe
     * copied out.
     */
    uint64_t write_out_beat_raw_strobe(Size width, unsigned beat_index) const;

    /**
     * Get the strobe for one beat of a write transaction. 'width' is the width
     * of the data channel in the signal level implementation of the AXI4 port
     * encoded as Size. The write strobe is passed as an array ceil('width' /
     * 8.0) bytes long with the same strobe organization as for write_in but
     * with the lowest strobe bit corresponding to the signal level beat's
     * lowest data byte rather than the whole transaction's data.
     */
    void write_out_beat_raw_strobe(Size width, unsigned beat_index,
        uint8_t* strobe) const;

    /** Get the number of MTE tags needed to be sent with the transaction. */
    unsigned get_mte_tag_count() const;

    /** Set the MTE tag at a given index. */
    void set_mte_tag(unsigned chunk_index, MteTag tag);

    /** Get the MTE tag at a given index */
    MteTag get_mte_tag(unsigned chunk_index) const;

    /**
     * Get the offset from the start of a Payload of the named extension.
     * Returns 0 if the extension has not been registered.
     * This function will typically only be called by PayloadExtension<>'s
     * constructor.
     */
    static std::size_t get_extension_offset(const char* name);

    /**
     * Register a new named extension
     * The manager provides the extension type's construct, copy and destruct
     * functionality as well as the size.
     * This function will typically only be called by PayloadExtension<>'s
     * constructor.
     */
    static std::size_t register_extension(const char* name,
        PayloadExtensionManager* manager);

    /**
     * LEGACY
     * Get the offset from the start of a Payload of the named extension.
     * Legacy get_extension_offset has the effect of registering an extension
     * if no extension of name 'name' has previously been the subject of a
     * get_extension_offset call. 'size' provides the size of the new/existing
     * extension.
     * This function will typically only be called by PayloadExtension<>'s
     * constructor.
     */
    static std::size_t get_extension_offset(unsigned size, const char* name);

    /**
     * Print debugging information for the payload pool. Useful for debugging
     * memory problems.
     */
    static void debug_payload_pool(std::ostream& stream);
};

/**
 * Base class of a PayloadExtensionManageer. This provides the functionality
 * of creating, copying and destroying extensions on payloads. The get_size
 * function should return the size in memory the extension occupies.
 */
class PayloadExtensionManager
{
public:
    virtual void create(void* /* ext */) {}
    virtual void destroy(void* /* ext */) {}
    virtual void copy(void* /* dst */, const void* /* src */) {}
    virtual std::size_t get_size() = 0;
};

/**
 * A default PayloadExtensionManager which calls the extension type's
 * constructor, copy constructor and destructor. If an extension manager is not
 * specified, this will be used.
 */
template <typename Type>
class PayloadExtensionManagerTyped :
    public PayloadExtensionManager
{
public:
    void create(void* vext)
    {
        Type* ext = reinterpret_cast<Type*>(vext);
        new (ext) Type();
    }

    void destroy(void* vext)
    {
        Type* ext = reinterpret_cast<Type*>(vext);
        ext->~Type();
    }

    void copy(void* vdst, const void* vsrc)
    {
        const Type* src = reinterpret_cast<const Type*>(vsrc);
        Type* dst = reinterpret_cast<Type*>(vdst);
        new(dst) Type(*src);
    }

    std::size_t get_size()
    {
        return sizeof(Type);
    }
};

/**
 * Extensions to AXI::Payload. Extensions must be registered by (creating a
 * PayloadExtension object on the extension's type, or calling
 * get_extension_offset) before any AXI::Payloads are made in a system.
 */
template <typename Type, typename ManagerType = PayloadExtensionManagerTyped<Type> >
class PayloadExtension
{
private:
    /** The offset into all Payloads where the extension can be found. */
    std::size_t offset;

public:
    /**
     * Constructor which registers an extension with the extension map managed
     * by the payload pool.
     */
    PayloadExtension(const char* name)
    {
        offset = Payload::get_extension_offset(name);
        if (offset == 0)
            offset = Payload::register_extension(name, new ManagerType());
    }

    /** Get an extension from a Payload object. */
    Type& get(Payload* payload)
    {
        return *reinterpret_cast<Type*>(
            reinterpret_cast<char*>(payload) + offset);
    }

    /** Set an extension (using operator=). */
    void set(Payload* payload, const Type& value)
    {
        get(payload) = value;
    }
};

}

/** Allow namespace AXI as ARM::AXI4 also covers AXI5. */
namespace AXI = AXI4;

}

#endif /* ARM_AXI4_PAYLOAD_H */
