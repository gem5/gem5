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
 * Authors: Steve Reinhardt
 *          Nathan Binkert
 */

#ifndef __AOUT_MACHDEP_H__
#define __AOUT_MACHDEP_H__

///
/// Funky Alpha 64-bit a.out header used for PAL code.
///
struct aout_exechdr {
    uint16_t    magic;          ///< magic number
    uint16_t    vstamp;         ///< version stamp?
    uint16_t    bldrev;         ///< ???
    uint16_t    padcell;        ///< padding
    uint64_t    tsize;          ///< text segment size
    uint64_t    dsize;          ///< data segment size
    uint64_t    bsize;          ///< bss segment size
    uint64_t    entry;          ///< entry point
    uint64_t    text_start;     ///< text base address
    uint64_t    data_start;     ///< data base address
    uint64_t    bss_start;      ///< bss base address
    uint32_t    gprmask;        ///< GPR mask (unused, AFAIK)
    uint32_t    fprmask;        ///< FPR mask (unused, AFAIK)
    uint64_t    gp_value;       ///< global pointer reg value
};

#define AOUT_LDPGSZ     8192

#define N_GETMAGIC(ex)  ((ex).magic)

#define N_BADMAX

#define N_TXTADDR(ex)   ((ex).text_start)
#define N_DATADDR(ex)   ((ex).data_start)
#define N_BSSADDR(ex)   ((ex).bss_start)

#define N_TXTOFF(ex)    \
        (N_GETMAGIC(ex) == ZMAGIC ? 0 : sizeof(struct aout_exechdr))

#define N_DATOFF(ex)    N_ALIGN(ex, N_TXTOFF(ex) + (ex).tsize)

#endif /* !__AOUT_MACHDEP_H__*/
