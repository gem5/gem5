/*
 * Copyright (c) 2012 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
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
 *          Andreas Hansson
 */

#ifndef __HASHMAP_HH__
#define __HASHMAP_HH__

#if defined(__GNUC__)

// for compilers that deprecate ext/hash_map, i.e. gcc >= 4.3 and
// clang, use unordered_map

// we need to determine what is available, as in the non-c++0x case,
// e.g. gcc >= 4.3 and <= 4.5, the containers are in the std::tr1
// namespace, and only gcc >= 4.6 (with -std=c++0x) adds the final
// container implementation in the std namespace

#if defined(__clang__)
// align with -std=c++0x only for clang >= 3.0 in CCFLAGS and also
// check if the header is present as this depends on what clang was
// built against, using XCode clang 3.1, for example, the header is
// not present without adding -stdlib=libc++
#if (__clang_major__ >= 3 && __has_include(<unordered_map>))
#define HAVE_STD_UNORDERED_MAP 1
#else
// we only support clang versions above 2.9 and these all have the tr1
// unordered_map
#define HAVE_STD_TR1_UNORDERED_MAP 1
#endif
#else
// align with -std=c++0x only for gcc >= 4.6 in CCFLAGS, contrary to
// clang we can rely entirely on the compiler version
#if ((__GNUC__ == 4 && __GNUC_MINOR__ >= 6) || __GNUC__ > 4)
#define HAVE_STD_UNORDERED_MAP 1
#else
#define HAVE_STD_TR1_UNORDERED_MAP 1
#endif
#endif

// set a default value of 0
#ifndef HAVE_STD_UNORDERED_MAP
#define HAVE_STD_UNORDERED_MAP 0
#endif

// set a default value of 0
#ifndef HAVE_STD_TR1_UNORDERED_MAP
#define HAVE_STD_TR1_UNORDERED_MAP 0
#endif

// now we are ready to deal with the actual includes based on what is
// available
#if (HAVE_STD_UNORDERED_MAP || HAVE_STD_TR1_UNORDERED_MAP)

#define hash_map unordered_map
#define hash_multimap unordered_multimap
#define hash_set unordered_set
#define hash_multiset unordered_multiset

// these versions also have an existing hash function for strings and
// 64-bit integer types
#define HAVE_HASH_FUNCTIONS 1

#if HAVE_STD_UNORDERED_MAP

// clang or gcc >= 4.6
#include <unordered_map>
#include <unordered_set>
// note that this assumes that -std=c++0x is added to the command line
// which is done in the SConstruct CXXFLAGS for gcc >= 4.6 and clang
// >= 3.0
#define __hash_namespace std
#define __hash_namespace_begin namespace std {
#define __hash_namespace_end }
#else
// clang <= 3.0, gcc >= 4.3 and < 4.6
#include <tr1/unordered_map>
#include <tr1/unordered_set>
#define __hash_namespace std::tr1
#define __hash_namespace_begin namespace std { namespace tr1 {
#define __hash_namespace_end } }
#endif
#else
// gcc < 4.3
#include <ext/hash_map>
#include <ext/hash_set>
#define __hash_namespace __gnu_cxx
#define __hash_namespace_begin namespace __gnu_cxx {
#define __hash_namespace_end }
#endif
#else
// non GNU compiler
#include <hash_map>
#include <hash_set>
#define __hash_namsepace std
#define __hash_namespace_begin namespace std {
#define __hash_namespace_end }
#endif

#include <string>

#include "base/types.hh"

namespace m5 {
    using ::__hash_namespace::hash_multimap;
    using ::__hash_namespace::hash_multiset;
    using ::__hash_namespace::hash_map;
    using ::__hash_namespace::hash_set;
    using ::__hash_namespace::hash;
}


///////////////////////////////////
// Some default Hashing Functions
//

__hash_namespace_begin

// if the hash functions for 64-bit integer types and strings are not
// already available, then declare them here
#if !defined(HAVE_HASH_FUNCTIONS)

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

    template <>
    struct hash<std::pair<std::string, uint64_t> > {
        size_t operator() (std::pair<std::string, uint64_t> r) const {
            return (__stl_hash_string(r.first.c_str())) ^ r.second;
        }
    };
#endif
__hash_namespace_end

#endif // __HASHMAP_HH__
