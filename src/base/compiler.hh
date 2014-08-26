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

#ifndef __BASE_COMPILER_HH__
#define __BASE_COMPILER_HH__

// gcc C++11 status: http://gcc.gnu.org/projects/cxx0x.html
// clang C++11 status: http://clang.llvm.org/cxx_status.html
// http://gcc.gnu.org/onlinedocs/gcc/Function-Attributes.html

/* Support for override control (final/override) */
#undef M5_COMP_HAS_OVERRIDE_CONTROL

#if defined(__GNUC__) && !defined(__clang__) /* Check for gcc */

#  define M5_GCC_VERSION(maj, min) \
    (__GNUC__ > (maj) || (__GNUC__ == (maj) && __GNUC_MINOR__ >= (min)))

#  define M5_COMP_HAS_OVERRIDE_CONTROL M5_GCC_VERSION(4, 7)

#elif defined(__clang__) /* Check for clang */

#  define M5_COMP_HAS_OVERRIDE_CONTROL __has_feature(cxx_override_control)

#else
#  error "Need to define compiler options in base/compiler.hh"
#endif


#if M5_COMP_HAS_OVERRIDE_CONTROL
#  define M5_ATTR_FINAL final
#  define M5_ATTR_OVERRIDE override
#else
#  define M5_ATTR_FINAL
#  define M5_ATTR_OVERRIDE
#endif

#if defined(__GNUC__) // clang or gcc
#  define M5_ATTR_NORETURN  __attribute__((noreturn))
#  define M5_DUMMY_RETURN
#  define M5_VAR_USED __attribute__((unused))
#  define M5_ATTR_PACKED __attribute__ ((__packed__))
#  define M5_NO_INLINE __attribute__ ((__noinline__))
#endif

#if defined(__clang__)
#  define M5_CLASS_VAR_USED M5_VAR_USED
#else
#  define M5_CLASS_VAR_USED
#endif

#endif // __BASE_COMPILER_HH__
