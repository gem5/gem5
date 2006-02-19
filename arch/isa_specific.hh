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
 */

#ifndef __ARCH_ISA_SPECIFIC_HH__
#define __ARCH_ISA_SPECIFIC_HH__

//This file provides a mechanism for other source code to bring in
//files from the ISA being compiled with

//These are constants so you can selective compile code based on the isa
//To use them, do something like
//
//#if THE_ISA == YOUR_FAVORITE_ISA
//	conditional_code
//#endif
//
//Note that this is how this file sets up the other isa "hooks"

//These macros have numerical values because otherwise the preprocessor
//would treat them as 0 in comparisons.
#define ALPHA_ISA 21064
#define SPARC_ISA 42
#define MIPS_ISA 1337

//These tell the preprocessor where to find the files of a particular
//ISA, and set the "TheISA" macro for use elsewhere.
#if THE_ISA == ALPHA_ISA
    #define ISA_PATH arch/alpha/
    #define TheISA AlphaISA
#elif THE_ISA == SPARC_ISA
    #define ISA_PATH arch/sparc/
    #define TheISA SparcISA
#elif THE_ISA == MIPS_ISA
    #define ISA_PATH arch/mips/
    #define TheISA MipsISA
#else
    #error "THE_ISA not set"
#endif
//The following is some preprocessor voodoo to allow redirectable includes
//The end result is the ISA_INCLUDE() macro which is used inside stub
//include files in arch and which redirect to the isa in use.
#define STRINGIFY(token) #token
#define EXPAND(token) token
#define STICK_TOGETHER(firstpart, secondpart) \
    EXPAND(firstpart)EXPAND(secondpart)
#define EXPAND_AND_STRINGIFY(pathAndFile) \
    STRINGIFY(pathAndFile)
#define ISA_INCLUDE(filename) \
    EXPAND_AND_STRINGIFY(STICK_TOGETHER(ISA_PATH, filename))

#endif
