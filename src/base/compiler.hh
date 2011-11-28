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

#ifndef __BASE_COMPILER_HH__
#define __BASE_COMPILER_HH__

//http://msdn2.microsoft.com/en-us/library/ms937669.aspx
//http://msdn2.microsoft.com/en-us/library/aa448724.aspx
//http://docs.sun.com/source/819-3688/sun.specific.html#marker-998278
//http://gcc.gnu.org/onlinedocs/gcc-3.3.1/gcc/Function-Attributes.html#Function%20Attributes

#if defined(__GNUC__)
#define M5_ATTR_NORETURN  __attribute__((noreturn))
#define M5_PRAGMA_NORETURN(x)
#define M5_DUMMY_RETURN
#define M5_VAR_USED __attribute__((unused))
#define M5_ATTR_PACKED __attribute__ ((__packed__))
#define M5_NO_INLINE __attribute__ ((__noinline__))
#elif defined(__SUNPRO_CC)
// this doesn't do anything with sun cc, but why not
#define M5_ATTR_NORETURN  __sun_attr__((__noreturn__))
#define M5_DUMMY_RETURN return (0);
#define DO_PRAGMA(x) _Pragma(#x)
#define M5_VAR_USED
#define M5_PRAGMA_NORETURN(x) DO_PRAGMA(does_not_return(x))
#define M5_ATTR_PACKED __attribute__ ((__packed__))
#define M5_NO_INLINE __attribute__ ((__noinline__))
#else
#error "Need to define compiler options in base/compiler.hh"
#endif

#endif // __BASE_COMPILER_HH__
