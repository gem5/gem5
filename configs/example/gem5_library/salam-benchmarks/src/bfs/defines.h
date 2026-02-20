#ifndef __DEFINES_H__
#define __DEFINES_H__

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// #define CHECK

#define MAX_LEVEL INT8_MAX

// Terminology (but not values) from graph500 spec
//   graph density = 2^-(2*SCALE - EDGE_FACTOR)
#define SCALE 8
#define EDGE_FACTOR 16

#define N_NODES (1LL << SCALE)
#define N_EDGES (N_NODES * EDGE_FACTOR)

// upper limit
#define N_LEVELS 10

typedef uint32_t TYPE;

// Larger than necessary for small graphs, but appropriate for large ones
typedef TYPE edge_index_t;
typedef TYPE node_index_t;
typedef int8_t level_t;

#endif
