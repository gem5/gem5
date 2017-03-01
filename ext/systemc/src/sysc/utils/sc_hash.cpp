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

  sc_hash.cpp -- Implementation of a chained hash table with MTF
                 (move-to-front).

  Original Author: Stan Y. Liao, Synopsys, Inc.

  CHANGE LOG AT END OF FILE
 *****************************************************************************/

#include <assert.h>
#include <stdlib.h> // duplicate (c)stdlib.h headers for Solaris
#include <cstdlib>
#include <cstddef>
#include <string.h>

#include "sysc/kernel/sc_cmnhdr.h"
#include "sysc/utils/sc_hash.h"
#include "sysc/utils/sc_mempool.h"

namespace sc_core {

// we can't assume global availability of uintptr_t,
// approximate it by size_t
typedef std::size_t uintptr_t;

const double PHASH_DEFAULT_GROW_FACTOR     = 2.0;

class sc_phash_elem {
    friend class sc_phash_base;
    friend class sc_phash_base_iter;

private:
    void* key;
    void* contents;
    sc_phash_elem* next;

    sc_phash_elem( void* k, void* c, sc_phash_elem* n )
        : key(k), contents(c), next(n) { }
    sc_phash_elem() : key(0), contents(0), next(0) { }
    ~sc_phash_elem() { }

    static void* operator new(std::size_t sz) 
        { return sc_mempool::allocate(sz); }
    static void operator delete(void* p, std::size_t sz) 
        { sc_mempool::release(p, sz);      }
};


sc_phash_base::sc_phash_base(
    void* def,
    int size,
    int density,
    double grow,
    bool reorder,
    unsigned (*hash_fn)(const void*),
    int (*cmp_fn)(const void*, const void*)
) :
    default_value(def), num_bins(0), num_entries(0), max_density(density),
    reorder_flag(reorder), grow_factor(grow), bins(0), hash(hash_fn), 
    cmpr(cmp_fn)
{
    if (size <= 0)
        size = PHASH_DEFAULT_INIT_TABLE_SIZE;
    else if ((size % 2) == 0)
        size += 1;
    num_bins = size;
    bins = new sc_phash_elem*[size];
    for (int i = 0; i < size; ++i)
        bins[i] = 0;
}

void
sc_phash_base::set_cmpr_fn(cmpr_fn_t c)
{
    cmpr = c;
}

void
sc_phash_base::set_hash_fn(hash_fn_t h)
{
    hash = h;
}

sc_phash_base::~sc_phash_base()
{
    sc_phash_elem* ptr;
    sc_phash_elem* next;

    for (int i = 0; i < num_bins; ++i) {
        ptr = bins[i];
        while (ptr != 0) {
            next = ptr->next;
            delete ptr;
            ptr = next;
        }
    }
    delete[] bins;
}

void
sc_phash_base::rehash()
{
    sc_phash_elem* ptr;
    sc_phash_elem* next;
    sc_phash_elem** old_bins = bins;

    int old_num_bins = num_bins;
    unsigned hash_val;

    num_bins = (int) (grow_factor * old_num_bins);
    if (num_bins % 2 == 0)
        ++num_bins;

    num_entries = 0;
    bins = new sc_phash_elem*[num_bins];
    memset( bins, 0, sizeof(sc_phash_elem*) * num_bins );

    for (int i = 0; i < old_num_bins; ++i) {
        ptr = old_bins[i];
        while (ptr != 0) {
            next = ptr->next;
            hash_val = do_hash(ptr->key);
            ptr->next = bins[hash_val];
            bins[hash_val] = ptr;
            ++num_entries;
            ptr = next;
        }
    }
    delete[] old_bins;
}

sc_phash_elem*
sc_phash_base::find_entry_q( unsigned hash_val, const void* key, sc_phash_elem*** plast )
{
    sc_phash_elem** last = &(bins[hash_val]);
    sc_phash_elem*  ptr  = *last;

    /* The (ptr->key != key) here is meant by the "q" */
    while ((ptr != 0) && (ptr->key != key)) {
        /*                         ^^ right here */
        last = &(ptr->next);
        ptr  = *last;
    }
    if ((ptr != 0) && reorder_flag) {
        *last = ptr->next;
        ptr->next = bins[hash_val];
        bins[hash_val] = ptr;
        last = &(bins[hash_val]);
    }
    if (plast) *plast = last;
    return ptr;
}

sc_phash_elem*
sc_phash_base::find_entry_c( unsigned hash_val, const void* key, sc_phash_elem*** plast )
{
    sc_phash_elem** last = &(bins[hash_val]);
    sc_phash_elem*  ptr  = *last;

    while ((ptr != 0) && ((*cmpr)(ptr->key, key) != 0)) {
        last = &(ptr->next);
        ptr = *last;
    }
        /* Bring to front */
    if ((ptr != 0) && reorder_flag) {
        *last = ptr->next;
        ptr->next = bins[hash_val];
        bins[hash_val] = ptr;
        last = &(bins[hash_val]);
    }
    if (plast) *plast = last;
    return ptr;
}

sc_phash_elem*
sc_phash_base::add_direct( void* key, void* contents, unsigned hash_val )
{
    if (num_entries / num_bins >= max_density) {
        rehash();
        hash_val = do_hash(key);
    }

    sc_phash_elem* new_entry = new sc_phash_elem(key, contents, bins[hash_val]);
    bins[hash_val] = new_entry;
    ++num_entries;
    return new_entry;
}

void
sc_phash_base::erase()
{
    for (int i = 0; i < num_bins; ++i) {
        sc_phash_elem* ptr = bins[i];
        while (ptr != 0) {
            sc_phash_elem* next = ptr->next;
            delete ptr;
            ptr = next;
            --num_entries;
        }
        bins[i] = 0;
    }
    assert(num_entries == 0);
}

void
sc_phash_base::erase(void (*kfree)(void*))
{
    for (int i = 0; i < num_bins; ++i) {
        sc_phash_elem* ptr = bins[i];
        while (ptr != 0) {
            sc_phash_elem* next = ptr->next;
            (*kfree)(ptr->key);
            delete ptr;
            ptr = next;
            --num_entries;
        }
        bins[i] = 0;
    }
    assert(num_entries == 0);
}

void
sc_phash_base::copy( const sc_phash_base* b )
{
    erase();
    iterator iter((sc_phash_base*) b);  /* cast away the const */
    for ( ; ! iter.empty(); iter++)
        insert( iter.key(), iter.contents() );
}

void
sc_phash_base::copy(const sc_phash_base& b, void* (*kdup)(const void*), void (*kfree)(void*))
{
    erase(kfree);
    iterator iter((sc_phash_base&) b);
    for ( ; ! iter.empty(); iter++)
        insert( (*kdup)(iter.key()), iter.contents() );
}

int
sc_phash_base::insert( void* k, void* c )
{
    unsigned hash_val = do_hash(k);
    sc_phash_elem* ptr = find_entry( hash_val, k );
    if (ptr == 0) {
        (void) add_direct(k, c, hash_val);
        return 0;
    }
    else {
        ptr->contents = c;
        return 1;
    }
}

int
sc_phash_base::insert( void* k, void* c, void* (*kdup)(const void*) )
{
    unsigned hash_val = do_hash(k);
    sc_phash_elem* ptr = find_entry( hash_val, k );
    if (ptr == 0) {
        (void) add_direct((*kdup)(k), c, hash_val);
        return 0;
    }
    else {
        ptr->contents = c;
        return 1;
    }
}

int
sc_phash_base::insert_if_not_exists( void* k, void* c )
{
    unsigned hash_val = do_hash(k);
    sc_phash_elem* ptr = find_entry( hash_val, k );
    if (ptr == 0) {
        (void) add_direct( k, c, hash_val );
        return 0;
    }
    else
        return 1;
}

int
sc_phash_base::insert_if_not_exists( void* k, void* c, void* (*kdup)(const void*) )
{
    unsigned hash_val = do_hash(k);
    sc_phash_elem* ptr = find_entry( hash_val, k );
    if (ptr == 0) {
        (void) add_direct( (*kdup)(k), c, hash_val );
        return 0;
    }
    else
        return 1;
}

int
sc_phash_base::remove( const void* k )
{
    unsigned hash_val = do_hash(k);
    sc_phash_elem** last;
    sc_phash_elem*  ptr = find_entry( hash_val, k, &last );

    if (ptr == 0)
        return 0;

    assert(*last == ptr);
    *last = ptr->next;
    delete ptr;
    --num_entries;
    return 1;
}

int
sc_phash_base::remove( const void* k, void** pk, void** pc )
{
    unsigned hash_val = do_hash(k);
    sc_phash_elem** last;
    sc_phash_elem*  ptr = find_entry( hash_val, k, &last );

    if (ptr == 0) {
        *pk = 0;
        *pc = 0;
        return 0;
    }
    else {
        *pk = ptr->key;
        *pc = ptr->contents;
    }

    assert(*last == ptr);
    *last = ptr->next;
    delete ptr;
    --num_entries;
    return 1;
}

int
sc_phash_base::remove(const void* k, void (*kfree)(void*))
{
    void* rk;
    void* rc;
    if (remove(k, &rk, &rc)) {
        (*kfree)(rk);
        return 1;
    }
    else
        return 0;
}

int
sc_phash_base::remove_by_contents( const void* c )
{
    sc_phash_elem** last;
    sc_phash_elem*  ptr;

    int num_removed = 0;
    for (int i = 0; i < num_bins; ++i) {
        last = &(bins[i]);
        ptr = *last;
        while (ptr != 0) {
            if (ptr->contents != c) {
                last = &(ptr->next);
                ptr  = *last;
            }
            else {
                *last = ptr->next;
                delete ptr;
                ptr = *last;
                --num_entries;
                ++num_removed;
            }
        }
    }
    return num_removed;
}

int
sc_phash_base::remove_by_contents( bool (*predicate)(const void* c, void* arg), void* arg )
{
    sc_phash_elem** last;
    sc_phash_elem*  ptr;

    int num_removed = 0;
    for (int i = 0; i < num_bins; ++i) {
        last = &(bins[i]);
        ptr = *last;
        while (ptr != 0) {
            if (! (*predicate)(ptr->contents, arg)) {
                last = &(ptr->next);
                ptr  = *last;
            }
            else {
                *last = ptr->next;
                delete ptr;
                ptr = *last;
                --num_entries;
                ++num_removed;
            }
        }
    }
    return num_removed;
}

int
sc_phash_base::remove_by_contents( const void* c, void (*kfree)(void*) )
{
    sc_phash_elem** last;
    sc_phash_elem*  ptr;

    int num_removed = 0;
    for (int i = 0; i < num_bins; ++i) {
        last = &(bins[i]);
        ptr = *last;
        while (ptr != 0) {
            if (ptr->contents != c) {
                last = &(ptr->next);
                ptr  = *last;
            }
            else {
                *last = ptr->next;
                (*kfree)(ptr->key);
                delete ptr;
                ptr = *last;
                --num_entries;
                ++num_removed;
            }
        }
    }
    return num_removed;
}

int
sc_phash_base::remove_by_contents( bool (*predicate)(const void*, void*), void* arg, void (*kfree)(void*))
{
    sc_phash_elem** last;
    sc_phash_elem*  ptr;

    int num_removed = 0;
    for (int i = 0; i < num_bins; ++i) {
        last = &(bins[i]);
        ptr = *last;
        while (ptr != 0) {
            if (! (*predicate)(ptr->contents, arg)) {
                last = &(ptr->next);
                ptr  = *last;
            }
            else {
                *last = ptr->next;
                (*kfree)(ptr->key);
                delete ptr;
                ptr = *last;
                --num_entries;
                ++num_removed;
            }
        }
    }
    return num_removed;
}

int
sc_phash_base::lookup( const void* k, void** c_ptr ) const
{
    unsigned hash_val = do_hash(k);
    sc_phash_elem* ptr = find_entry( hash_val, k );
    if (ptr == 0) {
        if (c_ptr != 0) *c_ptr = default_value;
        return 0;
    }
    else {
        if (c_ptr != 0) *c_ptr = ptr->contents;
        return 1;
    }
}

void*
sc_phash_base::operator[]( const void* key ) const
{
    void* contents;
    lookup( key, &contents );
    return contents;
}

/***************************************************************************/

void
sc_phash_base_iter::reset( sc_phash_base* t )
{
    table = t;
    index = 0;
    entry = 0;
    next  = 0;

    for (int i = index; i < table->num_bins; ++i) {
        if (table->bins[i] != 0) {
            index = i + 1;
            last  = &(table->bins[i]);
            entry = *last;
            next  = entry->next;
            break;
        }
    }
}

bool
sc_phash_base_iter::empty() const
{
    return (entry == 0);
}

void
sc_phash_base_iter::step()
{
    if (entry) {
        last = &(entry->next);
    }
    entry = next;
    if (! entry) {
        for (int i = index; i < table->num_bins; ++i) {
            if (table->bins[i] != 0) {
                index = i + 1;
                last = &(table->bins[i]);
                entry = *last;
                next = entry->next;
                break;
            }
        }
    }
    else {
        next = entry->next;
    }
}

void
sc_phash_base_iter::remove()
{
    delete entry;
    *last = next;
    entry = 0;
    --table->num_entries;
    step();
}

void
sc_phash_base_iter::remove(void (*kfree)(void*))
{
    (*kfree)(entry->key);
    delete entry;
    *last = next;
    entry = 0;
    --table->num_entries;
    step();
}

void*
sc_phash_base_iter::key() const
{
    return entry->key;
}

void*
sc_phash_base_iter::contents() const
{
    return entry->contents;
}

void*
sc_phash_base_iter::set_contents( void* c )
{
    return entry->contents = c;
}

/****************************************************************************/

unsigned 
default_ptr_hash_fn(const void* p)
{
    return static_cast<unsigned>(((uintptr_t)(p) >> 2) * 2654435789U);

}

unsigned
default_int_hash_fn(const void* p)
{
    return static_cast<unsigned>((uintptr_t)(p) * 3141592661U);
}


unsigned
default_str_hash_fn(const void* p)
{
    if (!p) return 0;

    const char* x = (const char*) p;
    unsigned int h = 0;
    unsigned int g;

    while (*x != 0) {
        h = (h << 4) + *x++;
        if ((g = h & 0xf0000000) != 0)
            h = (h ^ (g >> 24)) ^ g;
    }
    return h;
}

int
sc_strhash_cmp( const void* a, const void* b )
{
    return strcmp( (const char*) a, (const char*) b );
}

void*
sc_strhash_kdup(const void* k)
{
    char* result = (char*) malloc( strlen((const char*)k)+1 );
    strcpy(result, (const char*) k);
    return result;
}

void
sc_strhash_kfree(void* k)
{
    if (k) free((char*) k);
}
 } // namespace sc_core

// $Log: sc_hash.cpp,v $
// Revision 1.5  2011/08/26 20:42:30  acg
//  Andy Goodrich:
//    (1) Replaced strdup with new and strcpy to eliminate issue with the
//        Greenhills compiler.
//    (2) Moved modification log to the end of the file to eliminate line
//        skew when check-ins are done.
//
// Revision 1.4  2011/08/24 22:05:56  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.3  2011/05/05 17:46:04  acg
//  Philip A. Hartmann: changes in "swap" support.
//
// Revision 1.2  2011/02/18 20:38:43  acg
//  Andy Goodrich: Updated Copyright notice.
//
// Revision 1.1.1.1  2006/12/15 20:20:06  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:53:10  acg
// Andy Goodrich: Added $Log command so that CVS comments are reproduced in
// the source.

// taf
