/*
 * Copyright (c) 2002-2003 The Regents of The University of Michigan
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
 */

#ifndef __FIFO_BUFFER_HH__
#define __FIFO_BUFFER_HH__

#include "base/res_list.hh"


//
//  The FifoBuffer requires only that the objects to be used have a default
//  constructor and a dump() method
//
template<class T>
class FifoBuffer
{
  public:
    typedef typename res_list<T>::iterator iterator;

  private:
    res_list<T> *buffer;

    unsigned size;

  public:
    FifoBuffer(unsigned sz)
    {
        buffer = new res_list<T>(sz, true, 0);
        size = sz;
    }

    void add(T &item)
    {
        assert(buffer->num_free() > 0);
        buffer->add_head(item);
    }

    iterator head() { return buffer->head(); }
    iterator tail() { return buffer->tail(); }

    unsigned count() {return buffer->count();}
    unsigned free_slots() {return buffer->num_free();}

    T *peek() { return (count() > 0) ? tail().data_ptr() : 0; }

    T remove()
    {
        assert(buffer->count() > 0);
        T rval = *buffer->tail();
        buffer->remove_tail();
        return rval;
    }

    void dump();

    ~FifoBuffer() { delete buffer; }
};


#endif

