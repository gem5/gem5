/*
 * Taken from NetBSD sys/exec_aout.h
 */

/*	$NetBSD: exec_aout.h,v 1.29 2002/12/10 17:14:31 thorpej Exp $	*/

/*
 * Copyright (c) 1993, 1994 Christopher G. Demetriou
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
 *      This product includes software developed by Christopher G. Demetriou.
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

#ifndef _SYS_EXEC_AOUT_H_
#define _SYS_EXEC_AOUT_H_

#ifndef N_PAGSIZ
#define	N_PAGSIZ(ex)	(AOUT_LDPGSZ)
#endif

/* a_magic */
#define	OMAGIC		0407	/* old impure format */
#define	NMAGIC		0410	/* read-only text */
#define	ZMAGIC		0413	/* demand load format */

#define	N_ALIGN(ex,x) \
        (N_GETMAGIC(ex) == ZMAGIC ? \
        ((x) + AOUT_LDPGSZ - 1) & ~(AOUT_LDPGSZ - 1) : (x))

/* Valid magic number check. */
#define	N_BADMAG(ex) \
        (N_GETMAGIC(ex) != NMAGIC && N_GETMAGIC(ex) != OMAGIC && \
        N_GETMAGIC(ex) != ZMAGIC)

//Only alpha will be able to load aout for now
#include "arch/alpha/aout_machdep.h"

#endif /* !_SYS_EXEC_AOUT_H_ */
