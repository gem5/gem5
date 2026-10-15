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

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <gem5/m5ops.h>

#include "../bfs_clstr_hw_defines.h"
#include "bench.h"

bfs_struct bfs;

#define BASE 0x80c00000

#define NODES_OFFSET 0
#define EDGES_OFFSET NODES_OFFSET + sizeof(node_index_t) * N_NODES * 2
#define LEVEL_OFFSET EDGES_OFFSET + sizeof(edge_index_t) * N_EDGES
#define COUNT_OFFSET LEVEL_OFFSET + sizeof(level_t) * N_NODES
#define CHECK_OFFSET COUNT_OFFSET + sizeof(edge_index_t) * N_LEVELS

volatile uint8_t *top = (uint8_t *)(TOP);
volatile uint32_t *NODES_ADDR = (uint32_t *)(TOP + 1);
volatile uint32_t *EDGES_ADDR = (uint32_t *)(TOP + 9);
volatile uint32_t *LEVEL_ADDR = (uint32_t *)(TOP + 17);
volatile uint32_t *COUNT_ADDR = (uint32_t *)(TOP + 25);
volatile node_index_t *START_ADDR = (node_index_t *)(TOP + 33);

volatile int stage = 0;

int
main(void)
{
    node_index_t *nodes = (node_index_t *)(BASE + NODES_OFFSET);
    edge_index_t *edges = (edge_index_t *)(BASE + EDGES_OFFSET);
    level_t *level = (level_t *)(BASE + LEVEL_OFFSET);
    edge_index_t *level_counts = (edge_index_t *)(BASE + COUNT_OFFSET);
    edge_index_t *check = (edge_index_t *)(BASE + CHECK_OFFSET);
    stage = 0;
    volatile int count = 0;
    bfs.nodes = nodes;
    bfs.edges = edges;
    bfs.level = level;
    bfs.level_counts = level_counts;
    bfs.check = check;

    node_index_t starting_node = 38;

    printf("Generating data\n");
    genData(&bfs);
    printf("Data generated\n");

    *NODES_ADDR = (uint32_t)(void *)nodes;
    *EDGES_ADDR = (uint32_t)(void *)edges;
    *LEVEL_ADDR = (uint32_t)(void *)level;
    *COUNT_ADDR = (uint32_t)(void *)level_counts;
    printf("Starting node: %d\n", starting_node);
    *START_ADDR = starting_node;

    printf("Starting job\n");

    *top = 0x01;
    while (stage < 1) {
        count++;
    }

    printf("Job complete\n");

#ifdef CHECK
    bool fail = false;

    for (int i = 0; i < N_LEVELS; i++) {
        if (level_counts[i] != check[i]) {
            fail = true;
            printf("Mismatch: %d found, %d expected\n", level_counts[i],
                   check[i]);
        }
    }
    if (fail) {
        printf("Check Failed\n");
    } else {
        printf("Check Passed\n");
    }
#endif
    m5_dump_stats(0, 0);
    m5_exit(0);
}
