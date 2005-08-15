/*
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
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

/* @file
 * Device register definitions for a device's PCI config space
 */

#ifndef __PITREG_H__
#define __PITREG_H__

#include <sys/types.h>

// Control Word Format

#define PIT_SEL_SHFT  0x6
#define PIT_RW_SHFT   0x4
#define PIT_MODE_SHFT 0x1
#define PIT_BCD_SHFT  0x0

#define PIT_SEL_MASK  0x3
#define PIT_RW_MASK   0x3
#define PIT_MODE_MASK 0x7
#define PIT_BCD_MASK  0x1

#define GET_CTRL_FIELD(x, s, m) (((x) >> s) & m)
#define GET_CTRL_SEL(x) GET_CTRL_FIELD(x, PIT_SEL_SHFT, PIT_SEL_MASK)
#define GET_CTRL_RW(x) GET_CTRL_FIELD(x, PIT_RW_SHFT, PIT_RW_MASK)
#define GET_CTRL_MODE(x) GET_CTRL_FIELD(x, PIT_MODE_SHFT, PIT_MODE_MASK)
#define GET_CTRL_BCD(x) GET_CTRL_FIELD(x, PIT_BCD_SHFT, PIT_BCD_MASK)

#define PIT_READ_BACK 0x3

#define PIT_RW_LATCH_COMMAND 0x0
#define PIT_RW_LSB_ONLY      0x1
#define PIT_RW_MSB_ONLY      0x2
#define PIT_RW_16BIT         0x3

#define PIT_MODE_INTTC    0x0
#define PIT_MODE_ONESHOT  0x1
#define PIT_MODE_RATEGEN  0x2
#define PIT_MODE_SQWAVE   0x3
#define PIT_MODE_SWSTROBE 0x4
#define PIT_MODE_HWSTROBE 0x5

#define PIT_BCD_FALSE 0x0
#define PIT_BCD_TRUE  0x1

#endif // __PITREG_H__
