/*
 * Copyright (c) 2002-2004 The Regents of The University of Michigan
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

#ifndef __VTOPHYS_H__
#define __VTOPHYS_H__

#include "targetarch/isa_traits.hh"
#include "targetarch/pmap.h"

inline bool entry_valid(uint64_t entry)
{ return (entry & ALPHA_PTE_VALID) != 0; }

class ExecContext;
class PhysicalMemory;

Addr kernel_pte_lookup(PhysicalMemory *pmem, Addr ptbr, Addr vaddr);
Addr vtophys(PhysicalMemory *xc, Addr vaddr);
Addr vtophys(ExecContext *xc, Addr vaddr);
uint8_t *vtomem(ExecContext *xc, Addr vaddr, size_t len);
uint8_t *ptomem(ExecContext *xc, Addr paddr, size_t len);

void CopyOut(ExecContext *xc, void *dst, Addr src, size_t len);
void CopyIn(ExecContext *xc, Addr dst, void *src, size_t len);
void CopyString(ExecContext *xc, char *dst, Addr vaddr, size_t maxlen);

#endif // __VTOPHYS_H__

