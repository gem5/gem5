/*****************************************************************************

  Licensed to Accellera Systems Initiative Inc. (Accellera) under one or
  more contributor license agreements.  See the NOTICE file distributed
  with this work for additional information regarding copyright ownership.
  Accellera licenses this file to you under the Apache License, Version 2.0
  (the "License"); you may not use this file except in compliance with the
  License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
  implied.  See the License for the specific language governing
  permissions and limitations under the License.

 *****************************************************************************/

/*****************************************************************************

  sc_mempool.cpp - Memory pools for small objects.

  Original Author: Stan Y. Liao, Synopsys, Inc.

  CHANGE LOG AT END OF FILE
 *****************************************************************************/




//  <sc_mempool> is a class that manages the memory for small objects,
//  of sizes <increment>, 2 * <increment>, ..., <num_pools> *
//  <increment>.  When a memory request of <k> bytes is made through
//  the memory pool, the smallest pool <j> such that <j> * <increment>
//  >= <k> is used.  The default values of <increment> and <num_pools>
//  are 8 and 8, respectively.  Each pool has an allocator, that
//  simply keeps a free list of cells, and allocate new blocks
//  whenever necessary.  We are relying on malloc() to return a
//  properly aligned memory blocks.  Note that the memory blocks
//  allocated by the mempool are never freed.  Thus, if purify is
//  used, we may get MIU (memory-in-use) warnings.  To disable this,
//  set the environment variable SYSTEMC_MEMPOOL_DONT_USE to 1.


static const char* dont_use_envstring = "SYSTEMC_MEMPOOL_DONT_USE";
static bool use_default_new = false;


#include <stdio.h>
#include <stdlib.h> // duplicate (c)stdlib.h headers for Solaris
#include <cstdlib>
#include "sysc/utils/sc_mempool.h"

namespace sc_core {

//  An allocator is one that handles a particular size.  It keeps a
//  <free_list> from which a cell may be allocated quickly if there
//  is one available.  If no cell is available from <free_list>, then
//  the allocator tries to find whether space is available from the
//  most-recently-allocated block, as pointed to by <next_avail>.  If
//  so, then the cell pointed to by <next_avail> is returned, while
//  <next_avail> is advanced.  If <next_avail> now points beyond
//  the current block, then it's reset to 0.  On the other hand,
//  if <next_avail> was 0 when a request to the block is made, then
//  a new block is allocated by calling system malloc(), and the new
//  block becomes the head of <block_list>.


class sc_allocator {
    friend class sc_mempool;

public:
    sc_allocator( int blksz, int cellsz );
    ~sc_allocator();
    void* allocate();
    void release(void* p);
    
    void display_statistics();

private:
    union link {
        link* next;
        double align;          // alignment required.
    };

    int block_size;            // size of each block in bytes,
                               // including the link
    int cell_size;             // size of each cell in bytes

    char* block_list;
    link* free_list;
    char* next_avail;

    int total_alloc;
    int total_freed;
    int free_list_alloc;
};

sc_allocator::sc_allocator( int blksz, int cellsz )
  : block_size(sizeof(link) + (((blksz - 1) / cellsz) + 1) * cellsz),
    cell_size(cellsz), block_list(0), free_list(0), next_avail(0),
    total_alloc(0), total_freed(0), free_list_alloc(0)
{}

sc_allocator::~sc_allocator()
{
    // Shouldn't free the block_list, since global objects that use
    // the memory pool may not have been destroyed yet ...
    // Let it leak, let it leak, let it leak ...
}

void*
sc_allocator::allocate()
{
    void* result = 0;
    total_alloc++;
    if (free_list != 0) {
        free_list_alloc++;
        result = free_list;
        free_list = free_list->next;
        return result;
    }
    else if (next_avail != 0) {
        result = next_avail;
        next_avail += cell_size;
        // next_avail goes beyond the block
        if (next_avail >= block_list + block_size)
            next_avail = 0;
        return result;
    }
    else {  // (next_avail == 0)
        link* new_block = (link*) malloc(block_size);  // need alignment?
        new_block->next = (link*) block_list;
        block_list = (char*) new_block;
        result = (block_list + sizeof(link));
        // Assume that the block will hold more than one cell ... why
        // wouldn't it?
        next_avail = ((char*) result) + cell_size;
        return result;
    }
}

void
sc_allocator::release(void* p)
{
    total_freed++;
    ((link*) p)->next = free_list;
    free_list = (link*) p;
}

void
sc_allocator::display_statistics()
{
    int nblocks = 0;
    for (link* b = (link*) block_list; b != 0; b = b->next)
        nblocks++;
    printf("size %3d: %2d block(s), %3d requests (%3d from free list), %3d freed.\n",
           cell_size, nblocks, total_alloc, free_list_alloc, total_freed);
}


static const int cell_sizes[] = {
/* 0 */   0,
/* 1 */   8,
/* 2 */  16,
/* 3 */  24,
/* 4 */  32,
/* 5 */  48,
/* 6 */  64,
/* 7 */  80,
/* 8 */  96,
/* 9 */ 128
};

static const int cell_size_to_allocator[] = {
/*  0 */    0,
/*  1 */    1,
/*  2 */    2,
/*  3 */    3,
/*  4 */    4,
/*  5 */    5,
/*  6 */    5,
/*  7 */    6,
/*  8 */    6,
/*  9 */    7,
/* 10 */    7,
/* 11 */    8,
/* 12 */    8,
/* 13 */    9,
/* 14 */    9,
/* 15 */    9,
/* 16 */    9
};


class sc_mempool_int {
    friend class sc_mempool;

public:
    sc_mempool_int(int blksz, int npools, int incr);
    ~sc_mempool_int();
    void* do_allocate(std::size_t);
    void  do_release(void*, std::size_t);

    void display_statistics();
    
private:
    sc_allocator** allocators;
    int num_pools;
    int increment;
    int max_size;
};


static bool
compute_use_default_new()
{
    const char* e = getenv(dont_use_envstring);
    return (e != 0) && (atoi(e) != 0);
}

sc_mempool_int::sc_mempool_int(int blksz, int npools, int incr) :
    allocators(0), num_pools(0), increment(0), max_size(0)
{
    use_default_new = compute_use_default_new();
    if (! use_default_new) {
        num_pools = npools;
        increment = incr;
        max_size = cell_sizes[sizeof(cell_sizes)/sizeof(cell_sizes[0]) - 1];
        allocators = new sc_allocator*[npools + 1];
        for (int i = 1; i <= npools; ++i)
            allocators[i] = new sc_allocator(blksz, cell_sizes[i]);
        allocators[0] = allocators[1];
    }
}

sc_mempool_int::~sc_mempool_int()
{
    for (int i = 1; i <= num_pools; ++i)
        delete allocators[i];
    delete[] allocators;
}

static sc_mempool_int* the_mempool = 0;

void*
sc_mempool_int::do_allocate(std::size_t sz)
{
    int which_allocator = cell_size_to_allocator[(sz - 1) / increment + 1];
    void* p = allocators[which_allocator]->allocate();
    return p;
}

void
sc_mempool_int::do_release(void* p, std::size_t sz)
{
    int which_allocator = cell_size_to_allocator[(sz - 1) / increment + 1];
    allocators[which_allocator]->release(p);
}

void
sc_mempool_int::display_statistics()
{
    printf("*** Memory Pool Statistics ***\n");
    for (int i = 1; i <= num_pools; ++i)
        allocators[i]->display_statistics();
}

/****************************************************************************/

void*
sc_mempool::allocate(std::size_t sz)
{
    if (use_default_new)
        return ::operator new(sz);

    if (the_mempool == 0) {
        use_default_new = compute_use_default_new();
        if (use_default_new)
            return ::operator new(sz);

        // Note that the_mempool is never freed.  This is going to cause
        // memory leaks when the program exits.
        the_mempool = new sc_mempool_int( 1984, sizeof(cell_sizes)/sizeof(cell_sizes[0]) - 1, 8 );
    }

    if (sz > (unsigned) the_mempool->max_size)
        return ::operator new(sz);

    return the_mempool->do_allocate(sz);
}

void
sc_mempool::release(void* p, std::size_t sz)
{
    if (p) {
        
        if (use_default_new || sz > (unsigned) the_mempool->max_size) {
            ::operator delete(p);
            return;
        }

        the_mempool->do_release(p, sz);
    }
}

void
sc_mempool::display_statistics()
{
    if (the_mempool && !use_default_new) {
        the_mempool->display_statistics();
    } else {
        printf("SystemC info: no memory allocation was done through the memory pool.\n");
    }
}

} // namespace sc_core

// $Log: sc_mempool.cpp,v $
// Revision 1.4  2011/08/26 20:46:18  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.3  2011/08/24 22:05:56  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.2  2011/02/18 20:38:44  acg
//  Andy Goodrich: Updated Copyright notice.
//
// Revision 1.1.1.1  2006/12/15 20:20:06  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:53:10  acg
// Andy Goodrich: Added $Log command so that CVS comments are reproduced in
// the source.

// taf
