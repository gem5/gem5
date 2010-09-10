/*
 * Taken from NetBSD sys/exec_ecoff.h
 */

/*	$NetBSD: exec_ecoff.h,v 1.13 2003/01/18 09:53:18 thorpej Exp $	*/

/*
 * Copyright (c) 1994 Adam Glass
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed by Adam Glass.
 * 4. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef	_SYS_EXEC_ECOFF_H_
#define	_SYS_EXEC_ECOFF_H_

struct ecoff_filehdr {
        coff_ushort f_magic;	/* magic number */
        coff_ushort f_nscns;	/* # of sections */
        coff_uint   f_timdat;	/* time and date stamp */
        coff_ulong  f_symptr;	/* file offset of symbol table */
        coff_uint   f_nsyms;	/* # of symbol table entries */
        coff_ushort f_opthdr;	/* sizeof the optional header */
        coff_ushort f_flags;	/* flags??? */
};

struct ecoff_aouthdr {
        coff_ushort magic;
        coff_ushort vstamp;
        ECOFF_PAD
        coff_ulong  tsize;
        coff_ulong  dsize;
        coff_ulong  bsize;
        coff_ulong  entry;
        coff_ulong  text_start;
        coff_ulong  data_start;
        coff_ulong  bss_start;
        ECOFF_MACHDEP;
};

struct ecoff_scnhdr {		/* needed for size info */
        char	s_name[8];	/* name */
        coff_ulong  s_paddr;	/* physical addr? for ROMing?*/
        coff_ulong  s_vaddr;	/* virtual addr? */
        coff_ulong  s_size;		/* size */
        coff_ulong  s_scnptr;	/* file offset of raw data */
        coff_ulong  s_relptr;	/* file offset of reloc data */
        coff_ulong  s_lnnoptr;	/* file offset of line data */
        coff_ushort s_nreloc;	/* # of relocation entries */
        coff_ushort s_nlnno;	/* # of line entries */
        coff_uint   s_flags;	/* flags */
};

struct ecoff_exechdr {
        struct ecoff_filehdr f;
        struct ecoff_aouthdr a;
};

#define ECOFF_HDR_SIZE (sizeof(struct ecoff_exechdr))

#define ECOFF_OMAGIC 0407
#define ECOFF_NMAGIC 0410
#define ECOFF_ZMAGIC 0413

#define ECOFF_ROUND(value, by) \
        (((value) + (by) - 1) & ~((by) - 1))

#define ECOFF_BLOCK_ALIGN(ep, value) \
        ((ep)->a.magic == ECOFF_ZMAGIC ? ECOFF_ROUND((value), ECOFF_LDPGSZ) : \
         (value))

#define ECOFF_TXTOFF(ep) \
        ((ep)->a.magic == ECOFF_ZMAGIC ? 0 : \
         ECOFF_ROUND(ECOFF_HDR_SIZE + (ep)->f.f_nscns * \
                     sizeof(struct ecoff_scnhdr), ECOFF_SEGMENT_ALIGNMENT(ep)))

#define ECOFF_DATOFF(ep) \
        (ECOFF_BLOCK_ALIGN((ep), ECOFF_TXTOFF(ep) + (ep)->a.tsize))

#define ECOFF_SEGMENT_ALIGN(ep, value) \
        (ECOFF_ROUND((value), ((ep)->a.magic == ECOFF_ZMAGIC ? ECOFF_LDPGSZ : \
         ECOFF_SEGMENT_ALIGNMENT(ep))))

#endif /* !_SYS_EXEC_ECOFF_H_ */
