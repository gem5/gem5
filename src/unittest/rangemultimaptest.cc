/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 * Authors: Ali Saidi
 */

#include <cassert>
#include <iostream>

#include "base/range_map.hh"
#include "base/types.hh"

using namespace std;

int
main()
{
    typedef range_multimap<Addr, int> multimap_t;

    multimap_t r;
    multimap_t::iterator i;
    std::pair<multimap_t::iterator, multimap_t::iterator> jk;

    i = r.insert(RangeIn<Addr>(10,40),5);
    assert(i != r.end());
    i = r.insert(RangeIn<Addr>(10,40),6);
    assert(i != r.end());
    i = r.insert(RangeIn<Addr>(60,90),3);
    assert(i != r.end());

    jk = r.find(RangeIn(20,30));
    assert(jk.first != r.end());
    cout << jk.first->first << " " << jk.first->second << endl;
    cout << jk.second->first << " " << jk.second->second << endl;

    i = r.insert(RangeIn<Addr>(0,3),5);
    assert(i != r.end());

    for( i = r.begin(); i != r.end(); i++)
        cout << i->first << " " << i->second << endl;

    jk = r.find(RangeIn(20,30));
    assert(jk.first != r.end());
    cout << jk.first->first << " " << jk.first->second << endl;
    cout << jk.second->first << " " << jk.second->second << endl;


}








