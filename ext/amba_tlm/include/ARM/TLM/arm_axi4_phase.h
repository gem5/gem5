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

#ifndef ARM_AXI4_PHASE_H
#define ARM_AXI4_PHASE_H

#include <stdint.h>

namespace ARM
{
/** AMBA AXI protocol support. */
namespace AXI4
{

/* Enumeration of the channel portion of phases. */
enum Channel
{
    /* Pseudo channel to encode the uninitialized phase value. */
    CHANNEL_UNINITIALIZED = 0,

    /* Real AXI/ACE channels and acks. */
    CHANNEL_AW   = 0x1,
    CHANNEL_W    = 0x2,
    CHANNEL_B    = 0x3,
    CHANNEL_AR   = 0x4,
    CHANNEL_R    = 0x5,
    CHANNEL_AC   = 0x6,
    CHANNEL_CR   = 0x7,
    CHANNEL_CD   = 0x8,
    CHANNEL_WACK = 0x9,
    CHANNEL_RACK = 0xA,
    CHANNEL_WQOSACCEPT = 0xB,
    CHANNEL_RQOSACCEPT = 0xC

};

/**
 * Communication phases for AXI4.
 *
 * Phases correspond to events on AXI4 channels.
 *
 * VALID phases are used when offering a Payload to another agent. READY is
 * used as a response (either immediately/in the same cycle with: `phase =
 * ...READY; return TLM_UPDATED;`, or later with another communicating function
 * call) to a VALID phase.
 *
 * \par Read data phases (bw)
 *
 * Protocol |RLAST| Note                  | Phase
 * -------- | --: | --------------------- | -----
 * All      | `0` | Non-final data beat.  | `R_VALID`
 * ^        | `1` | Final data beat.      | `R_VALID_LAST`
 *
 * \par Write response phases (bw)
 *
 * Protocol| Transaction Type        |BCOMP|BPERSIST|BTAGMATCH| Note                                         || Phase
 * ---- | -------------------------- | --: | -----: | ------: | -------------------------------------------- || -----
 * AXI4 |                            | n/a |    n/a |     n/a | Single beat response.                        || `B_VALID` <sup>a</sup>
 * AXI5 | Not persist, not Tag Match | `1` |    `0` |  `0b00` | Single beat response.                        || `B_VALID_COMP` <sup>a</sup>
 * ^    | Persist                    | `1` |    `0` |  `0b00` | Completion response.  | 2-part response.      | `B_VALID_COMP`
 * ^    | ^                          | `0` |    `1` |  `0b00` | Persist response.     | ^                     | `B_VALID_PERSIST`
 * ^    | ^                          | `1` |    `1` |  `0b00` | Combined response.    | Single beat response. | `B_VALID_COMP_PERSIST`
 * ^    | Tag Match                  | `1` |    `0` |  `0b01` | Completion response.  | 2-part response.      | `B_VALID_COMP`
 * ^    | ^                          | `0` |    `0` |  `0b1x` | Match response.       | ^                     | `B_VALID_TAGMATCH`
 * ^    | ^                          | `1` |    `0` |  `0b1x` | Combined response.    | Single beat response. | `B_VALID_COMP_TAGMATCH`
 *
 * <sup>a</sup> `B_VALID` and `V_VALID_COMP` have the same value and may be used
 * interchangeably. `B_VALID` is recommended for AXI4 and `B_VALID_COMP` is
 * recommended for AXI5.
 *
 * Payload field validity:
 *  * The `resp` field is transported on all AXI4 phases and AXI5 phases when
 *    `BCOMP=1`, that is single-beat responses and 2-part completion responses.
 *  * The `tag_match` field is transported on AXI5 phases when `BTAGMATCH=0b1x`,
 *    that is single-beat combined responses and 2-part match responses.
 */
enum Phase
{
    /*
     * Bit position meaning:
     * [0] == 1:   READY
     * [7:4]:      Channel number
     * [7:4] == 0: UNINITIALIZED
     * [7:4] == 1: AW
     * [7:4] == 2: W
     * [7:4] == 3: B
     * [7:4] == 4: AR
     * [7:4] == 5: R
     * [7:4] == 6: AC
     * [7:4] == 7: CR
     * [7:4] == 8: CD
     * [7:4] == 9: WACK
     * [7:4] == A: RACK
     * [8] == 1:        LAST (on channels: R, W, CD)
     * [8] == ~BCOMP:   NOT COMP (on channel: B)
     * [9] == BPERSIST: PERSIST (on channel: B)
     * [10]:       TAGMATCH (on channel: B)
     * [19:12]:    QOSACCEPT value (on channel: WQOSACCEPT, RQOSACCEPT)
     * [19:12]:    RCHUNKNUM value (on channel: R)
     * [27:20]:    RCHUNKSTRB value (one channel: R)
     */

    PHASE_UNINITIALIZED = 0x000,
    PHASE_MASK = 0xFFFFFFFF,

    AW_VALID              = 0x010,
    AW_READY              = 0x011,

    W_VALID               = 0x020,
    W_VALID_LAST          = 0x120,
    W_READY               = 0x021,

    B_VALID               = 0x030,
    B_VALID_COMP          = 0x030,
    B_VALID_PERSIST       = 0x330,
    B_VALID_COMP_PERSIST  = 0x230,
    B_VALID_TAGMATCH      = 0x530,
    B_VALID_COMP_TAGMATCH = 0x430,
    B_READY               = 0x031,

    AR_VALID              = 0x040,
    AR_READY              = 0x041,

    R_VALID               = 0x050,
    R_VALID_LAST          = 0x150,
    R_READY               = 0x051,

    AC_VALID              = 0x060,
    AC_READY              = 0x061,

    CR_VALID              = 0x070,
    CR_READY              = 0x071,

    CD_VALID              = 0x080,
    CD_VALID_LAST         = 0x180,
    CD_READY              = 0x081,

    WACK                  = 0x090,
    RACK                  = 0x0A0,

    WQOSACCEPT            = 0x0B0,
    RQOSACCEPT            = 0x0C0
};

/* Get Channel from Phase */
inline Channel phase_get_channel(Phase phase)
{ return static_cast<Channel>((phase >> 4) & 0xf); }

/* Remove the chunk number and strobe from phase. */
inline Phase phase_strip(Phase phase)
{ return Phase(phase & 0xff1); }

/* Extract the QOSACCEPT from Phase. */
inline uint8_t phase_get_qos_accept(Phase phase)
{ return static_cast<uint8_t>((phase >> 12) & 0xff); }

/* Set the QOSACCEPT in Phase. */
inline void phase_set_qos_accept(Phase &phase, uint8_t qos_accept)
{ phase = Phase((uint32_t(phase) & ~(0xff << 12)) | uint32_t(qos_accept & 0xff) << 12); }

/* Extract the RCHUNKNUM from Phase. */
inline uint8_t phase_get_chunk_number(Phase phase)
{ return static_cast<uint8_t>((phase >> 12) & 0xff); }

/* Set the RCHUNKNUM in Phase. */
inline void phase_set_chunk_number(Phase &phase, uint8_t chunk_number)
{ phase = Phase((uint32_t(phase) & ~(0xff << 12)) | uint32_t(chunk_number & 0xff) << 12); }

/* Extract the RCHUNKSTRB from Phase. */
inline uint8_t phase_get_chunk_strobe(Phase phase)
{ return static_cast<uint8_t>((phase >> 20) & 0xff); }

/* Set the RCHUNKSTRB in Phase. */
inline void phase_set_chunk_strobe(Phase &phase, uint8_t chunk_strobe)
{ phase = Phase((uint32_t(phase) & ~(0xff << 20)) | uint32_t(chunk_strobe & 0xff) << 20); }

}

/** Allow namespace AXI as ARM::AXI4 also covers AXI5. */
namespace AXI = AXI4;

}

#endif /* ARM_AXI4_PHASE_H */
