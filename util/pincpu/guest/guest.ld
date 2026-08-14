/*
 * Copyright (c) 2026 The Board of Trustees of the Leland Stanford
 * Junior University
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

/*
 * Split the image into separate segments so that no LOAD segment ends up
 * writable and executable at once. Without explicit PHDRS the linker merges
 * everything into one RWX segment, which modern binutils warns about.
 */
PHDRS
{
    text   PT_LOAD FLAGS(5); /* R E */
    rodata PT_LOAD FLAGS(4); /* R   */
    data   PT_LOAD FLAGS(6); /* RW  */
}

SECTIONS
{
    . = 0xdead0000000;

    .text : { *(.text .text.*) } :text

    /* Segments need their own pages to carry distinct permissions. */
    . = ALIGN(CONSTANT(MAXPAGESIZE));
    .rodata            : { *(.rodata .rodata.*) }  :rodata
    .eh_frame          : { *(.eh_frame) }          :rodata
    .note.gnu.property : { *(.note.gnu.property) } :rodata
    .note.gnu.build-id : { *(.note.gnu.build-id) } :rodata

    . = ALIGN(CONSTANT(MAXPAGESIZE));
    .got     : { *(.got) }                 :data
    .got.plt : { *(.got.plt) }             :data
    .data    : { *(.data .data.*) }        :data
    .bss     : { *(.bss .bss.*) *(COMMON) } :data
}
