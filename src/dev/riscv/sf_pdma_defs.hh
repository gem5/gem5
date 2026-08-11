/*
 * Copyright (c) 2025 Nikita Proshkin
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

#ifndef _SF_PDMA_DEFS_HH_
#define _SF_PDMA_DEFS_HH_

#include <stdint.h>

#include "base/bitunion.hh"

// Channel Memory Map
// Each channel mapping starts from the new page (4 KiB).
//
// Offset   | Size  | Name              | Description
// 0x0      | 4     | Control           | Channel Control Register
// 0x004    | 4     | NextConfig        | Next transfer type
// 0x008    | 8     | NextBytes         | Number of bytes to move
// 0x010    | 8     | NextDestination   | Destination start address
// 0x018    | 8     | NextSource        | Source start address
// 0x104    | 4     | ExecConfig        | Active transfer type
// 0x108    | 8     | ExecBytes         | Number of bytes remaining
// 0x110    | 8     | ExecDestination   | Destination current address
// 0x118    | 8     | ExecSource        | Source current address
//
// 'Next' registers are RW and used by the driver to configure next transfer.
// 'Exec' registers are RO and associated with the transfer in progress.
// When driver sets 'run' bit in Control register, PDMA copies 'Next' registers
// to the 'Exec' set and starts the transfer.

// Control Register
BitUnion32(PDMACtrl)
    // Indicates that the channel is in use. Setting this clears all of the
    // channel’s Next registers. This bit can only be cleared when run is low.
    Bitfield<0> claim;
    // Setting this bit starts a DMA transfer by copying the Next registers
    // into their Exec counterparts.
    Bitfield<1> run;
    // Enable transfer done interrupts.
    Bitfield<14> done_ie;
    // Enable bus error interrupts.
    Bitfield<15> error_ie;
    // Indicates that a transfer has completed since the channel was claimed.
    Bitfield<30> done;
    // Indicates that a transfer error has occurred since the channel was
    // claimed.
    Bitfield<31> error;
EndBitUnion(PDMACtrl)

// Next/ExecConfig Register
BitUnion32(PDMAConf)
    // If set, the Exec registers are reloaded from the Next registers once a
    // transfer is complete. The repeat bit must be cleared by software for the
    // sequence to stop.
    Bitfield<2> repeat;
    // Enforces strict ordering by only allowing one of each transfer type
    // in-flight at a time.
    Bitfield<3> order;
    // The wsize and rsize fields are used to determine the size and alignment
    // of individual PDMA transactions, as a single PDMA transfer might require
    // multiple transactions. They contain Base 2 Logarithm of PDMA transaction
    // sizes.
    Bitfield<27, 24> wsize;
    Bitfield<31, 28> rsize;
EndBitUnion(PDMAConf)

// Set of 'Next' or 'Exec' registers
union PDMARegs
{
    uint8_t data[32];

    struct
    {
        // Keep 0x004 offset requirement
        uint32_t reserved;
        // Transfer config (PDMAConf)
        uint32_t config;
        // Transfer size
        uint64_t size;
        // Transfer destination address
        uint64_t dest;
        // Transfer source address
        uint64_t src;
    };
};

#define PDMA_CTRL_OFFSET    0x0
#define PDMA_CONF_OFFSET    0x4
#define PDMA_SIZE_OFFSET    0x8
#define PDMA_DEST_OFFSET    0x10
#define PDMA_SRC_OFFSET     0x18

#endif
