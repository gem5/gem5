/*
 * Taken from NetBSD arch/alpha/ecoff_machdep.h
 */

/* $NetBSD: ecoff_machdep.h,v 1.5 1999/04/27 02:32:33 cgd Exp $ */

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
 *	This product includes software developed by Adam Glass.
 * 4. The name of the Author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Adam Glass ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL Adam Glass BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

//
// Define COFF/ECOFF integer type sizes
//
typedef  int16_t coff_short;
typedef uint16_t coff_ushort;
typedef  int32_t coff_int;
typedef uint32_t coff_uint;
typedef  int64_t coff_long;
typedef uint64_t coff_ulong;
typedef uint64_t coff_addr;

#define ECOFF_LDPGSZ 4096

#define ECOFF_PAD \
        coff_ushort	bldrev;					/* XXX */

#define ECOFF_MACHDEP \
        coff_uint	gprmask; \
        coff_uint	fprmask; \
        coff_ulong	gp_value

#define ECOFF_MAGIC_ALPHA		0603
#define ECOFF_MAGIC_NETBSD_ALPHA	0605
#define ECOFF_BADMAG(ep)						\
        ((ep)->f.f_magic != ECOFF_MAGIC_ALPHA &&			\
            (ep)->f.f_magic != ECOFF_MAGIC_NETBSD_ALPHA)

#define ECOFF_FLAG_EXEC			0002
#define ECOFF_SEGMENT_ALIGNMENT(ep) \
    (((ep)->f.f_flags & ECOFF_FLAG_EXEC) == 0 ? 8 : 16)

#define	ECOFF_FLAG_OBJECT_TYPE_MASK	0x3000
#define		ECOFF_OBJECT_TYPE_NO_SHARED	0x1000
#define		ECOFF_OBJECT_TYPE_SHARABLE	0x2000
#define		ECOFF_OBJECT_TYPE_CALL_SHARED	0x3000

