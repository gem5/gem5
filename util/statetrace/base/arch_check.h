/*
 * Copyright (c) 2006-2007 The Regents of The University of Michigan
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
 * Authors: Gabe Black
 */

#if defined __STATETRACE_ALPHA__
    #if !defined __alpha__
        #error "Alpha toolchain required."
    #endif
#elif defined __STATETRACE_AMD64__
    #if !defined __amd64__
        #error "Amd64 toolchain required."
    #endif
#elif defined __STATETRACE_ARM__
    #if !defined __arm__
        #error "Arm toolchain required."
    #endif
#elif defined __STATETRACE_HPPA__
    #if !defined __hppa__
        #error "Hppa toolchain required."
    #endif
#elif defined __STATETRACE_I686__
    #if !(defined __i386__ || defined __i486__ || \
            defined __i586__ || defined __i686__)
        #error "I686 toolchain required."
    #endif
#elif defined __STATETRACE_IA64__
    #if !defined __ia64__
        #error "IA64 toolchain required."
    #endif
#elif defined __STATETRACE_MIPS__
    #if !defined __mips__
        #error "Mips toolchain required."
    #endif
#elif defined __STATETRACE_POWERPC__
    #if !defined __powerpc__
        #error "PowerPC toolchain required."
    #endif
#elif defined __STATETRACE_SPARC__
    #if !defined __sparc__
        #error "Sparc toolchain required."
    #endif
#elif defined __STATETRACE_SH__
    #if !defined __sh__
        #error "SuperH toolchain required."
    #endif
#elif defined __STATETRACE__S390__
    #if !defined __s390__
        #error "System/390 toolchain required."
    #endif
#else
    #error "Couldn't determine architecture."
#endif
