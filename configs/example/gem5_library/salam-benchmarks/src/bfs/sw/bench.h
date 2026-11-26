/*
 * Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
 * (University of Wisconsin-Madison)
 * All rights reserved.
 *
 * This file contains code derived from gem5-SALAM:
 * https://github.com/TeCSAR-UNCC/gem5-SALAM
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

#ifndef __BFS_BENCH_H__
#define __BFS_BENCH_H__

#include "../defines.h"

extern volatile int stage;

#include "data.h"

typedef struct
{
    node_index_t *nodes;
    edge_index_t *edges;
    level_t *level;
    edge_index_t *level_counts;
    node_index_t starting_node;
    edge_index_t *check;
} bfs_struct;

void
genData(bfs_struct *bfs)
{
    int i;

    bfs->starting_node = start;
    for (i = 0; i < 2 * N_NODES; i++) {
        bfs->nodes[i] = nod[i];
    }
    for (i = 0; i < N_EDGES; i++) {
        bfs->edges[i] = edg[i];
    }
    for (i = 0; i < N_LEVELS; i++) {
        bfs->check[i] = chk[i];
    }
    for (i = 0; i < N_NODES; i++) {
        bfs->level[i] = MAX_LEVEL;
    }
}

#endif // __BFS_BENCH_H__
