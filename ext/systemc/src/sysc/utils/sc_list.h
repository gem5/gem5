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

  sc_list.h -- Simple implementation of a doubly linked list.

  Original Author: Stan Y. Liao, Synopsys, Inc.

  CHANGE LOG AT END OF FILE
 *****************************************************************************/


#ifndef SC_LIST_H
#define SC_LIST_H

namespace sc_core {

//Some forward declarations
class sc_plist_elem;
template<class T> class sc_plist_iter; 

typedef void (*sc_plist_map_fn)( void* data, void* arg );

class sc_plist_base {
    friend class sc_plist_base_iter;

public:
    sc_plist_base();
    ~sc_plist_base();
    
    typedef sc_plist_elem* handle_t;

    handle_t push_back(void* d);
    handle_t push_front(void* d);
    void* pop_back();
    void* pop_front();
    handle_t insert_before(handle_t h, void* d);
    handle_t insert_after(handle_t h, void* d);
    void* remove(handle_t h);
    void* get(handle_t h) const;
    void set(handle_t h, void* d);
    void mapcar( sc_plist_map_fn f, void* arg );

    void* front() const;
    void* back() const;

    void erase_all();
    bool empty() const { return (head == 0); }
    int size() const;

private:
    handle_t head;
    handle_t tail;
};


class sc_plist_base_iter {
public:
    typedef sc_plist_elem* handle_t;
    
    sc_plist_base_iter( sc_plist_base* l, bool from_tail = false );
    ~sc_plist_base_iter();

    void reset( sc_plist_base* l, bool from_tail = false );
    bool empty() const;
    void operator++(int);
    void operator--(int);
    void* get() const;
    void  set(void* d);
    void remove();
    void remove(int direction);

    void set_handle(handle_t h);
    handle_t get_handle() const { return ptr; }
    
private:
    sc_plist_base* lst;
    sc_plist_elem* ptr;
};

/*---------------------------------------------------------------------------*/

template< class T >
class sc_plist : public sc_plist_base {
    friend class sc_plist_iter <T>;

public:
    typedef sc_plist_iter<T> iterator;

    sc_plist() { } 
    ~sc_plist() { } 

    handle_t push_back(T d)  { return sc_plist_base::push_back((void*)d);  }
    handle_t push_front(T d) { return sc_plist_base::push_front((void*)d); }
    T pop_back()           { return (T) sc_plist_base::pop_back(); }
    T pop_front()          { return (T) sc_plist_base::pop_front(); }
    handle_t insert_before(handle_t h, T d) 
    {
        return sc_plist_base::insert_before(h, (void*) d);
    }
    handle_t insert_after(handle_t h, T d)
    {
        return sc_plist_base::insert_after(h, (void*) d);
    }
    T remove(handle_t h)
    {
        return (T)sc_plist_base::remove(h);
    }
    T get(handle_t h) const { return (T)sc_plist_base::get(h); }
    void set(handle_t h, T d) { sc_plist_base::set(h, (void*)d); }

    T front() const { return (T)sc_plist_base::front(); }
    T back() const { return (T)sc_plist_base::back(); }
};

template< class T >
class sc_plist_iter : public sc_plist_base_iter {
public:
    sc_plist_iter( sc_plist<T>* l, bool from_tail = false )
        : sc_plist_base_iter( l, from_tail )
    {

    }
    sc_plist_iter( sc_plist<T>& l, bool from_tail = false )
        : sc_plist_base_iter( &l, from_tail )
    {

    }
    ~sc_plist_iter()
    {

    }

    void reset( sc_plist<T>* l, bool from_tail = false )
    {
        sc_plist_base_iter::reset( l, from_tail );
    }
    void reset( sc_plist<T>& l, bool from_tail = false )
    {
        sc_plist_base_iter::reset( &l, from_tail );
    }

    T operator*() const { return (T) sc_plist_base_iter::get(); }
    T get() const     { return (T) sc_plist_base_iter::get(); }
    void set(T d)     { sc_plist_base_iter::set((void*) d); }
};

} // namespace sc_core

// $Log: sc_list.h,v $
// Revision 1.5  2011/09/01 15:16:50  acg
//  Philipp A. Hartmann: revert unnecessary virtual destructors.
//
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

#endif
