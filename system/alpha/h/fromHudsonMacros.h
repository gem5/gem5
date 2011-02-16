/*
 * Copyright (c) 1993-1994 The Hewlett-Packard Development Company
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

#ifndef HUDSON_MACROS_LOADED
#define	HUDSON_MACROS_LOADED	    1

#define	STALL \
    mfpr    r31, pt0

#define	NOP \
    bis	    $31, $31, $31

/*
** Align code on an 8K byte page boundary.
*/

#define	ALIGN_PAGE \
    .align  13

/*
** Align code on a 32 byte block boundary.
*/

#define	ALIGN_BLOCK \
    .align  5

/*
** Align code on a quadword boundary.
*/

#define ALIGN_BRANCH \
    .align  3

/*
** Hardware vectors go in .text 0 sub-segment.
*/

#define	HDW_VECTOR(offset) \
    . = offset

/*
** Privileged CALL_PAL functions are in .text 1 sub-segment.
*/

#define	CALL_PAL_PRIV(vector) \
    . = (PAL_CALL_PAL_PRIV_ENTRY+(vector<<6))

/*
** Unprivileged CALL_PAL functions are in .text 1 sub-segment,
** the privileged bit is removed from these vectors.
*/

#define CALL_PAL_UNPRIV(vector) \
    . = (PAL_CALL_PAL_UNPRIV_ENTRY+((vector&0x3F)<<6))

/*
** Implements a load "immediate" longword function
*/
#define LDLI(reg,val) \
        ldah	reg, ((val+0x8000) >> 16)(zero); \
        lda	reg, (val&0xffff)(reg)

#endif
