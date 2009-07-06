/*
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
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

/*
 * flitBuffer_d.hh
 *
 * Niket Agarwal, Princeton University
 *
 * */

#ifndef FLIT_BUFFER_D_H
#define FLIT_BUFFER_D_H

#include "mem/ruby/network/garnet-fixed-pipeline/NetworkHeader.hh"
#include "mem/gems_common/PrioHeap.hh"
#include "mem/ruby/network/garnet-fixed-pipeline/flit_d.hh"

class flitBuffer_d {
public:
        flitBuffer_d();
        flitBuffer_d(int maximum_size);

        bool isReady();
        bool isReadyForNext();
        bool isEmpty();
        void print(ostream& out) const;
        bool isFull();
        void setMaxSize(int maximum);

        inline flit_d* getTopFlit()
        {
                return m_buffer.extractMin();
        }
        inline flit_d* peekTopFlit()
        {
                return m_buffer.peekMin();
        }
        inline void insert(flit_d *flt)
        {
                m_buffer.insert(flt);
        }
        /**********Data Members*********/
private:
        PrioHeap <flit_d *> m_buffer;
        int size, max_size;
};

ostream& operator<<(ostream& out, const flitBuffer_d& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline
ostream& operator<<(ostream& out, const flitBuffer_d& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif

