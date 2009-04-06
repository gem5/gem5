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

#ifndef __ARCH_ALPHA_OSFPAL_HH__
#define __ARCH_ALPHA_OSFPAL_HH__

struct PAL
{
    enum {
        // Privileged PAL functions
        halt = 0x00,
        cflush = 0x01,
        draina = 0x02,
        cserve = 0x09,
        swppal = 0x0a,
        wripir = 0x0d,
        rdmces = 0x10,
        wrmces = 0x11,
        wrfen = 0x2b,
        wrvptptr = 0x2d,
        swpctx = 0x30,
        wrval = 0x31,
        rdval = 0x32,
        tbi = 0x33,
        wrent = 0x34,
        swpipl = 0x35,
        rdps = 0x36,
        wrkgp = 0x37,
        wrusp = 0x38,
        wrperfmon = 0x39,
        rdusp = 0x3a,
        whami = 0x3c,
        retsys = 0x3d,
        wtint = 0x3e,
        rti = 0x3f,

        // unprivileged pal functions
        bpt = 0x80,
        bugchk = 0x81,
        callsys = 0x83,
        imb = 0x86,
        urti = 0x92,
        rdunique = 0x9e,
        wrunique = 0x9f,
        gentrap = 0xaa,
        clrfen = 0xae,
        nphalt = 0xbe,
        copypal = 0xbf,
        NumCodes
    };

    static const char *name(int index);
};

#endif // __ARCH_ALPHA_OSFPAL_HH__
