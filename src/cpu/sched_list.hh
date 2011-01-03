/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * All rights reserved.
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
 *
 * Authors: Steve Raasch
 */

#ifndef SCHED_LIST_HH
#define SCHED_LIST_HH

#include <list>
#include "base/intmath.hh"
#include "base/misc.hh"


//  Any types you use this class for must be covered here...
namespace {
    void ClearEntry(int &i) { i = 0; };
    void ClearEntry(unsigned &i) { i = 0; };
    void ClearEntry(double &i) { i = 0; };
    template <class T> void ClearEntry(std::list<T> &l) { l.clear(); };
};


//
//  this is a special list type that allows the user to insert elements at a
//  specified positive offset from the "current" element, but only allow them
//  be extracted from the "current" element
//


template <class T>
class SchedList
{
    T *data_array;
    unsigned position;
    unsigned size;
    unsigned mask;

  public:
    SchedList(unsigned size);
    SchedList(void);

    void init(unsigned size);

    T &operator[](unsigned offset);

    void advance(void);

    void clear(void);
};



//
//  Constructor
//
template<class T>
SchedList<T>::SchedList(unsigned _size)
{
    size = _size;

    //  size must be a power of two
    if (!isPowerOf2(size)) {
        panic("SchedList: size must be a power of two");
    }

    if (size < 2) {
        panic("SchedList: you don't want a list that small");
    }

    //  calculate the bit mask for the modulo operation
    mask = size - 1;

    data_array = new T[size];

    if (!data_array) {
        panic("SchedList: could not allocate memory");
    }

    clear();
}

template<class T>
SchedList<T>::SchedList(void)
{
    data_array = 0;
    size = 0;
}


template<class T> void
SchedList<T>::init(unsigned _size)
{
    size = _size;

    if (!data_array) {
        //  size must be a power of two
        if (size & (size-1)) {
            panic("SchedList: size must be a power of two");
        }

        if (size < 2) {
            panic("SchedList: you don't want a list that small");
        }

        //  calculate the bit mask for the modulo operation
        mask = size - 1;

        data_array = new T[size];

        if (!data_array) {
            panic("SchedList: could not allocate memory");
        }

        clear();
    }
}


template<class T> void
SchedList<T>::advance(void)
{
    ClearEntry(data_array[position]);

    //    position = (++position % size);
    position = ++position & mask;
}


template<class T> void
SchedList<T>::clear(void)
{
    for (unsigned i=0; i<size; ++i) {
        ClearEntry(data_array[i]);
    }

    position = 0;
}


template<class T>  T&
SchedList<T>::operator[](unsigned offset)
{
    if (offset >= size) {
        panic("SchedList: can't access element beyond current pointer");
    }

    //    unsigned p = (position + offset) % size;
    unsigned p = (position + offset) & mask;

    return data_array[p];
}



#endif
