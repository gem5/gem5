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
