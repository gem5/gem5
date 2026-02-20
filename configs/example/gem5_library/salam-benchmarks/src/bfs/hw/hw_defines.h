/*
Implementations based on:
Harish and Narayanan.
"Accelerating large graph algorithms on the GPU using CUDA." HiPC, 2007.
Hong, Oguntebi, Olukotun.
"Efficient Parallel Graph Exploration on Multi-Core CPU and GPU." PACT, 2011.
*/

#include "../bfs_clstr_hw_defines.h"
#include "../defines.h"

typedef struct edge_t_struct
{
    node_index_t dst;
} edge_t;

typedef struct node_t_struct
{
    edge_index_t edge_begin;
    edge_index_t edge_end;
} node_t;

#define DEV_INIT 0x01
#define DEV_INTR 0x04

#define NODESSIZE N_NODES * sizeof(node_t)
#define EDGESSIZE N_EDGES * sizeof(edge_t)
#define LEVELSIZE N_NODES * sizeof(level_t)
#define LVLCNTSIZE N_LEVELS * sizeof(edge_index_t)
