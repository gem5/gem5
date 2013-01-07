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

#if defined(__clang__)
// check if the header is present, which requires -stdlib=libc++, and
// that in turn causes problems with incomplete template parameters
#if (__has_include(<unordered_map>))
#define HAVE_STD_UNORDERED_MAP 1
#endif
#else
// we only support gcc >= 4.4 as the other option
#define HAVE_STD_UNORDERED_MAP 1
#endif

// set a default value of 0 clang with the header in the tr1 namespace
#ifndef HAVE_STD_UNORDERED_MAP
#define HAVE_STD_UNORDERED_MAP 0
#endif

#define hash_map unordered_map
#define hash_multimap unordered_multimap
#define hash_set unordered_set
#define hash_multiset unordered_multiset

#if HAVE_STD_UNORDERED_MAP
// gcc or clang with libc++
#include <unordered_map>
#include <unordered_set>
#define __hash_namespace std
#define __hash_namespace_begin namespace std {
#define __hash_namespace_end }
#else
// clang with libstdc++
#include <tr1/unordered_map>
#include <tr1/unordered_set>
#define __hash_namespace std::tr1
#define __hash_namespace_begin namespace std { namespace tr1 {
#define __hash_namespace_end } }
#endif

namespace m5 {
    using ::__hash_namespace::hash_multimap;
    using ::__hash_namespace::hash_multiset;
    using ::__hash_namespace::hash_map;
    using ::__hash_namespace::hash_set;
    using ::__hash_namespace::hash;
}

#endif // __HASHMAP_HH__
