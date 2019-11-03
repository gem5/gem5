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

#ifndef SC_HASH_H
#define SC_HASH_H


namespace sc_core {

extern unsigned default_int_hash_fn(const void*);
extern unsigned default_ptr_hash_fn(const void*);
extern unsigned default_str_hash_fn(const void*);

class sc_phash_elem;
class sc_phash_base_iter;
template<class K, class C>  //template class 
class sc_pdhash_iter;       //decl. -- Amit

const int    PHASH_DEFAULT_MAX_DENSITY     = 5;
const int    PHASH_DEFAULT_INIT_TABLE_SIZE = 11;
extern const double PHASH_DEFAULT_GROW_FACTOR;
const bool   PHASH_DEFAULT_REORDER_FLAG    = true;

class sc_phash_base {
    friend class sc_phash_base_iter;

    typedef sc_phash_base_iter iterator;

public:
    typedef unsigned (*hash_fn_t)(const void*);
    typedef int (*cmpr_fn_t)(const void*, const void*);

protected:
    void*    default_value;
    int      num_bins;
    int      num_entries;
    int      max_density;
    int      reorder_flag;
    double   grow_factor;

    sc_phash_elem** bins;

    hash_fn_t hash;
    cmpr_fn_t cmpr;

    void rehash();
    unsigned do_hash(const void* key) const { return (*hash)(key) % num_bins; }

    sc_phash_elem* add_direct(void* key, void* contents, unsigned hash_val);
    sc_phash_elem* find_entry_c(unsigned hv, const void* k, sc_phash_elem*** plast);
    sc_phash_elem* find_entry_q(unsigned hv, const void* k, sc_phash_elem*** plast);
    sc_phash_elem* find_entry(unsigned hv, const void* k, sc_phash_elem*** plast=0) const
    {
      /* Got rid of member func. pointer and replaced with if-else  */
      /* Amit (5/14/99)                                             */
      if( cmpr == 0 )
        return ((sc_phash_base*)this)->find_entry_q( hv, k, plast );
      else 
	return ((sc_phash_base*)this)->find_entry_c( hv, k, plast );
    }

public:
    sc_phash_base( void* def       = 0,
                   int    size     = PHASH_DEFAULT_INIT_TABLE_SIZE,
                   int    density  = PHASH_DEFAULT_MAX_DENSITY,
                   double grow     = PHASH_DEFAULT_GROW_FACTOR,
                   bool   reorder  = PHASH_DEFAULT_REORDER_FLAG,    
                   hash_fn_t hash_fn = default_ptr_hash_fn,
                   cmpr_fn_t cmpr_fn = 0                             );
    ~sc_phash_base();

    void set_cmpr_fn(cmpr_fn_t);
    void set_hash_fn(hash_fn_t);

    bool empty() const { return (num_entries == 0); }
    unsigned count() const { return num_entries; }

    void erase();
    void erase(void (*kfree)(void*));
    void copy( const sc_phash_base* );
    void copy( const sc_phash_base& b ) { copy(&b); }
    void copy( const sc_phash_base& b, void* (*kdup)(const void*), void (*kfree)(void*));
    int insert( void* k, void* c );
    int insert( void* k ) { return insert(k, default_value); }
    int insert( void* k, void* c, void* (*kdup)(const void*) );
    int insert_if_not_exists(void* k, void* c);
    int insert_if_not_exists(void* k) { return insert_if_not_exists(k, default_value); }
    int insert_if_not_exists(void* k, void* c, void* (*kdup)(const void*));
    int remove(const void* k);
    int remove(const void* k, void** pk, void** pc);
    int remove(const void* k, void (*kfree)(void*));
    int remove_by_contents(const void* c);
    int remove_by_contents(bool (*predicate)(const void*, void*), void* arg);
    int remove_by_contents(const void* c, void (*kfree)(void*));
    int remove_by_contents(bool (*predicate)(const void*, void*), void* arg, void (*kfree)(void*));
    int lookup(const void* k, void** pc) const;
    bool contains(const void* k) const { return (lookup(k, 0) != 0); }
    void* operator[](const void* key) const;
};

class sc_phash_base_iter {
protected:
    sc_phash_base*  table;
    sc_phash_elem*  entry;
    sc_phash_elem*  next;
    sc_phash_elem** last;
    int             index;

public:
    void reset(sc_phash_base* t);
    void reset(sc_phash_base& t) { reset(&t); }

    sc_phash_base_iter(sc_phash_base* t)
    : table(t), entry(0), next(0), last(0), index(0)
        { reset(t); }
    sc_phash_base_iter(sc_phash_base& t)
    : table(&t), entry(0), next(0), last(0), index(0)
        { reset(t); }
    ~sc_phash_base_iter() { }

    bool empty() const;
    void step();
    void operator++(int) { step(); }
    void remove();
    void remove(void (*kfree)(void*));
    void* key() const;
    void* contents() const;
    void* set_contents(void* c);
};

template< class K, class C >
class sc_phash_iter;

template< class K, class C >
class sc_phash : public sc_phash_base {
    friend class sc_phash_iter<K,C>;

public:
    typedef sc_phash_iter<K,C> iterator;

    sc_phash( C def = (C) 0,
              int size    = PHASH_DEFAULT_INIT_TABLE_SIZE,
              int density = PHASH_DEFAULT_MAX_DENSITY,
              double grow = PHASH_DEFAULT_GROW_FACTOR,
              bool reorder = PHASH_DEFAULT_REORDER_FLAG,
              hash_fn_t hash_fn = default_ptr_hash_fn,
              cmpr_fn_t cmpr_fn = 0                          )
        : sc_phash_base((void*) def, size, density, grow, reorder, hash_fn, cmpr_fn) { }
    ~sc_phash() { }

    void copy(const sc_phash<K,C>* b) { sc_phash_base::copy(b); }
    void copy(const sc_phash<K,C>& b) { sc_phash_base::copy(b); }
    void copy(const sc_phash<K,C>& b, void* (*kdup)(const void*), void (*kfree)(void*)) { sc_phash_base::copy(b, kdup, kfree); }

    int insert(K k, C c) { return sc_phash_base::insert((void*) k, (void*) c); }
    int insert(K k) { return sc_phash_base::insert((void*) k, default_value); }
    int insert(K k, C c, void* (*kdup)(const void*)) { return sc_phash_base::insert((void*) k, (void*) c, kdup); }
    int insert_if_not_exists(K k, C c)
    {
        return sc_phash_base::insert_if_not_exists((void*) k, (void*) c);
    }
    int insert_if_not_exists(K k)
    {
        return sc_phash_base::insert_if_not_exists((void*) k, default_value);
    }
    int insert_if_not_exists(K k, C c, void* (*kdup)(const void*))
    {
        return sc_phash_base::insert_if_not_exists((void*) k, (void*) c, kdup);
    }
    int remove(K k) { return sc_phash_base::remove((const void*) k); }
    int remove(K k, K* pk, C* pc)
    {
        return sc_phash_base::remove((const void*) k, (void**) pk, (void**) pc);
    }
    int remove(K k, void (*kfree)(void*))
    {
        return sc_phash_base::remove((const void*) k, kfree);
    }
    int remove_by_contents(C c)
    {
        return sc_phash_base::remove_by_contents((const void*) c);
    }
    int remove_by_contents(bool (*predicate)(const void*, void*), void* arg)
    {
        return sc_phash_base::remove_by_contents(predicate, arg);
    }
    int remove_by_contents(const void* c, void (*kfree)(void*))
    {
        return sc_phash_base::remove_by_contents(c, kfree);
    }
    int remove_by_contents(bool (*predicate)(const void*, void*), void* arg, void (*kfree)(void*))
    {
        return sc_phash_base::remove_by_contents(predicate, arg, kfree);
    }
    int lookup(K k, C* pc) const
    {
        return sc_phash_base::lookup((const void*) k, (void**) pc);
    }
    bool contains(K k) const
    {
        return sc_phash_base::contains((const void*) k);
    }
    C operator[](K k) const
    {
        return (C) sc_phash_base::operator[]((const void*) k);
    }
};


template< class K, class C >
class sc_phash_iter : public sc_phash_base_iter {
public:
    sc_phash_iter(sc_phash<K,C>* t) : sc_phash_base_iter(t) { }
    sc_phash_iter(sc_phash<K,C>& t) : sc_phash_base_iter(t) { }
    ~sc_phash_iter() { }

    void reset(sc_phash<K,C>* t) { sc_phash_base_iter::reset(t); }
    void reset(sc_phash<K,C>& t) { sc_phash_base_iter::reset(t); }

    K key()      const { return (K) sc_phash_base_iter::key();      }
    C contents() const { return (C) sc_phash_base_iter::contents(); }
    C set_contents(C c)
    {
        return (C) sc_phash_base_iter::set_contents((void*) c);
    }
};





template< class K, class C >
class sc_pdhash : public sc_phash_base {
    friend class sc_pdhash_iter<K,C>;

private:
    void* (*kdup)(const void*); //eliminated braces around void* -- Amit
    void (*kfree)(void*);

public:
    typedef sc_pdhash_iter<K,C> iterator;
    sc_pdhash( C def = (C) 0,
              int size    = PHASH_DEFAULT_INIT_TABLE_SIZE,
              int density = PHASH_DEFAULT_MAX_DENSITY,
              double grow = PHASH_DEFAULT_GROW_FACTOR,
              bool reorder = PHASH_DEFAULT_REORDER_FLAG,
              hash_fn_t hash_fn = (hash_fn_t) 0, // defaults added --
              cmpr_fn_t cmpr_fn = (cmpr_fn_t) 0, // Amit
              void* (*kdup_fn)(const void*) = 0,
              void (*kfree_fn)(void*)  = 0 )
        : sc_phash_base((void*) def, size, density, grow, reorder, hash_fn, cmpr_fn)
    {
        kdup = kdup_fn;
        kfree = kfree_fn;
    }
    ~sc_pdhash()
    {
        erase();
    }
    void erase()
    {
        sc_phash_base::erase(kfree);
    }
    void copy(const sc_pdhash<K,C>& b) { sc_phash_base::copy(b, kdup, kfree); }
    int insert(K k, C c) { return sc_phash_base::insert((void*) k, (void*) c, kdup); }
    int insert(K k) { return sc_phash_base::insert((void*) k, default_value, kdup); }
    int insert_if_not_exists(K k, C c)
    {
        return sc_phash_base::insert_if_not_exists((void*) k, (void*) c, kdup);
    }
    int insert_if_not_exists(K k)
    {
        return sc_phash_base::insert_if_not_exists((void*) k, default_value, kdup);
    }
    int remove(K k) { return sc_phash_base::remove((const void*) k, kfree); }
    int remove(K k, K* pk, C* pc)
    {
        return sc_phash_base::remove((const void*) k, (void**) pk, (void**) pc);
    }
    int remove_by_contents(C c)
    {
        return sc_phash_base::remove_by_contents((const void*) c, kfree);
    }
    int remove_by_contents(bool (*predicate)(const void*, void*), void* arg)
    {
        return sc_phash_base::remove_by_contents(predicate, arg, kfree);
    }
    int lookup(K k, C* pc) const
    {
        return sc_phash_base::lookup((const void*) k, (void**) pc);
    }
    bool contains(K k) const
    {
        return sc_phash_base::contains((const void*) k);
    }
    C operator[](K k) const
    {
        return (C) sc_phash_base::operator[]((const void*) k);
    }
};

template< class K, class C >
class sc_pdhash_iter : public sc_phash_base_iter {
public:
    sc_pdhash_iter(sc_pdhash<K,C>* t) : sc_phash_base_iter(t) { }
    sc_pdhash_iter(sc_pdhash<K,C>& t) : sc_phash_base_iter(t) { }
    ~sc_pdhash_iter() { }

    void reset(sc_pdhash<K,C>* t) { sc_phash_base_iter::reset(t); }
    void reset(sc_pdhash<K,C>& t) { sc_phash_base_iter::reset(t); }

    void remove() { sc_phash_base_iter::remove(((sc_pdhash<K,C>*) table)->kfree); }
    K key()      const { return (K) sc_phash_base_iter::key();      }
    C contents() const { return (C) sc_phash_base_iter::contents(); }
    C set_contents(C c)
    {
        return (C) sc_phash_base_iter::set_contents((void*) c);
    }
};

extern int sc_strhash_cmp( const void*, const void* );
extern void sc_strhash_kfree(void*);
extern void* sc_strhash_kdup(const void*);

template< class C >      // template class decl.
class sc_strhash_iter;   // --Amit

template< class C >
class sc_strhash : public sc_phash_base {
    friend class sc_strhash_iter<C>;

public:
    typedef sc_strhash_iter<C> iterator;

    sc_strhash( C def = (C) 0,
                int size    = PHASH_DEFAULT_INIT_TABLE_SIZE,
                int density = PHASH_DEFAULT_MAX_DENSITY,
                double grow = PHASH_DEFAULT_GROW_FACTOR,
                bool reorder = PHASH_DEFAULT_REORDER_FLAG,
                unsigned (*hash_fn)(const void*) = default_str_hash_fn,
                int (*cmpr_fn)(const void*, const void*) = sc_strhash_cmp )
        : sc_phash_base((void*) def, size, density, grow, reorder, hash_fn, cmpr_fn)
    {

    }
    ~sc_strhash()
    {
        erase();
    }

    void erase() { sc_phash_base::erase(sc_strhash_kfree); }
    void copy(const sc_strhash<C>* b) { sc_phash_base::copy(*b, sc_strhash_kdup, sc_strhash_kfree); }
    void copy(const sc_strhash<C>& b) { sc_phash_base::copy(b, sc_strhash_kdup, sc_strhash_kfree); }

    int insert(char* k, C c) { return sc_phash_base::insert((void*) k, (void*) c, sc_strhash_kdup); }
    int insert(char* k) { return sc_phash_base::insert((void*) k, default_value, sc_strhash_kdup); }
    int insert_if_not_exists(char* k, C c)
    {
        return sc_phash_base::insert_if_not_exists((void*) k, (void*) c, sc_strhash_kdup);
    }
    int insert_if_not_exists(char* k)
    {
        return sc_phash_base::insert_if_not_exists((void*) k, default_value, sc_strhash_kdup);
    }
    int remove(const char* k) { return sc_phash_base::remove((const void*) k, sc_strhash_kfree); }
    int remove(const char* k, char** pk, C* pc)
    {
        return sc_phash_base::remove((const void*) k, (void**) pk, (void**) pc);
    }
    int remove_by_contents(C c)
    {
        return sc_phash_base::remove_by_contents((const void*) c, sc_strhash_kfree);
    }
    int remove_by_contents(bool (*predicate)(const void*, void*), void* arg)
    {
        return sc_phash_base::remove_by_contents(predicate, arg, sc_strhash_kfree);
    }
    int lookup(const char* k, C* pc) const
    {
        return sc_phash_base::lookup((const void*) k, (void** )pc);
    }
    bool contains(const char* k) const
    {
        return sc_phash_base::contains((const void*) k);
    }
    C operator[](const char* k) const
    {
        return (C) sc_phash_base::operator[]((const void*) k);
    }
};

template<class C>
class sc_strhash_iter : public sc_phash_base_iter {
public:
    sc_strhash_iter ( sc_strhash<C>* t ) : sc_phash_base_iter(t) { }
    sc_strhash_iter ( sc_strhash<C>& t ) : sc_phash_base_iter(t) { }
    ~sc_strhash_iter() { }

    void reset ( sc_strhash<C>* t ) { sc_phash_base_iter::reset(t); }
    void reset ( sc_strhash<C>& t ) { sc_phash_base_iter::reset(t); }

    void remove() { sc_phash_base_iter::remove(sc_strhash_kfree); }
    const char* key()   { return (const char*) sc_phash_base_iter::key(); }
    C contents()  { return (C) sc_phash_base_iter::contents(); }
    C set_contents(C c)
    {
        return (C) sc_phash_base_iter::set_contents((void*) c);
    }
};

} // namespace sc_core

// $Log: sc_hash.h,v $
// Revision 1.5  2011/08/29 18:04:32  acg
//  Philipp A. Hartmann: miscellaneous clean ups.
//
// Revision 1.4  2011/08/26 20:46:16  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.3  2011/08/24 22:05:56  acg
//  Torsten Maehne: initialization changes to remove warnings.
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

#endif
