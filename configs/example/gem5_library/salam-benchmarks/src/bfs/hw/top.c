/*
 * Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
 * (University of Wisconsin-Madison)
 * All rights reserved.
 *
 * This file contains modifications and/or code derived from:
 * gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
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

#include "hw_defines.h"

void
top(uint64_t nodes_addr, uint64_t edges_addr, uint64_t levels_addr,
    uint64_t level_counts_addr, node_index_t starting_node)
{
    // Define Device MMRs
    volatile uint8_t *BFSFlags = (uint8_t *)(BFS);
    volatile uint8_t *BFSConfig = (uint8_t *)(BFS + 1);
    volatile uint8_t *DmaFlags = (uint8_t *)(DMA_Flags);
    volatile uint64_t *DmaRdAddr = (uint64_t *)(DMA_RdAddr);
    volatile uint64_t *DmaWrAddr = (uint64_t *)(DMA_WrAddr);
    volatile uint32_t *DmaCopyLen = (uint32_t *)(DMA_CopyLen);

    // Transfer Input Matrices
    // Transfer Nodes
    *DmaRdAddr = nodes_addr;
    *DmaWrAddr = NODES;
    *DmaCopyLen = NODESSIZE;
    *DmaFlags = DEV_INIT;
    // Poll DMA for finish
    while ((*DmaFlags & DEV_INTR) != DEV_INTR)
        ;
    // Transfer Edges
    *DmaRdAddr = edges_addr;
    *DmaWrAddr = EDGES;
    *DmaCopyLen = EDGESSIZE;
    *DmaFlags = DEV_INIT;
    // Poll DMA for finish
    while ((*DmaFlags & DEV_INTR) != DEV_INTR)
        ;
    // Transfer Levels
    *DmaRdAddr = levels_addr;
    *DmaWrAddr = LEVELS;
    *DmaCopyLen = LEVELSIZE;
    *DmaFlags = DEV_INIT;
    // Poll DMA for finish
    while ((*DmaFlags & DEV_INTR) != DEV_INTR)
        ;

    // Start the accelerated function
    *(node_index_t *)BFSConfig = starting_node;
    *BFSFlags = DEV_INIT;
    // Poll function for finish
    while ((*BFSFlags & DEV_INTR) != DEV_INTR)
        ;

    // Transfer level_counts
    *DmaRdAddr = LEVELCOUNTS;
    *DmaWrAddr = level_counts_addr;
    *DmaCopyLen = LVLCNTSIZE;
    *DmaFlags = DEV_INIT;
    // Poll DMA for finish
    while ((*DmaFlags & DEV_INTR) != DEV_INTR)
        ;
    return;
}
