/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 * Authors: Nathan Binkert
 */

#ifndef __HASHMAP_HH__
#define __HASHMAP_HH__

#if defined(__GNUC__) && __GNUC__ >= 3
#include <ext/hash_map>
#else
#include <hash_map>
#endif

#include <string>

#include "sim/host.hh"

#if defined(__GNUC__) && __GNUC__ >= 3
    #define __hash_namespace __gnu_cxx
#else
    #define __hash_namespace std
#endif

namespace m5 {
    using ::__hash_namespace::hash_multimap;
    using ::__hash_namespace::hash_map;
    using ::__hash_namespace::hash;
}


///////////////////////////////////
// Some default Hashing Functions
//

namespace __hash_namespace {
#if !defined(__LP64__) && !defined(__alpha__) && !defined(__SUNPRO_CC)
    template<>
    struct hash<uint64_t> {
        size_t operator()(uint64_t r) const {
            return r;
        }
    };

    template<>
    struct hash<int64_t> {
        size_t operator()(int64_t r) const {
            return r;
        };
    };
#endif

    template<>
    struct hash<std::string> {
        size_t operator()(const std::string &s) const {
            return(__stl_hash_string(s.c_str()));
        }
    };
}


#endif // __HASHMAP_HH__
