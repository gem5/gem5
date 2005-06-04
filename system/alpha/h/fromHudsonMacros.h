/*
Copyright 1993, 1994 Hewlett-Packard Development Company, L.P.

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef HUDSON_MACROS_LOADED
#define	HUDSON_MACROS_LOADED	    1

/*
 *      VID: [T1.2] PT: [Fri Apr 21 16:47:16 1995] SF: [macros.h]
 *       TI: [/sae_users/cruz/bin/vice -iplatform.s -l// -p# -DEB164 -h -m -aeb164 ]
 */
/*
**
**  FACILITY:
**
**	DECchip 21164 PALcode
**
**  MODULE:
**
**	macros.h
**
**  MODULE DESCRIPTION:
**
**      DECchip 21164 PALcode macro definitions
**
**  AUTHOR: ER
**
**  CREATION DATE:  29-Nov-1993
**
**  $Id: fromHudsonMacros.h,v 1.1.1.1 1997/10/30 23:27:19 verghese Exp $
**
**  MODIFICATION HISTORY:
**
**  $Log: fromHudsonMacros.h,v $
**  Revision 1.1.1.1  1997/10/30 23:27:19  verghese
**  current 10/29/97
**
**  Revision 1.1  1995/11/18 01:46:23  boyle
**  Initial revision
**
**  Revision 1.5  1994/07/08  17:03:12  samberg
**  Changes to support platform specific additions
**
**  Revision 1.4  1994/05/20  19:24:19  ericr
**  Moved STALL macro from osfpal.s to here
**  Added LDLI macro
**
**  Revision 1.3  1994/05/20  18:08:14  ericr
**  Changed line comments to C++ style comment character
**
**  Revision 1.2  1994/02/28  18:45:51  ericr
**  Fixed EGORE related bugs
**
**  Revision 1.1  1993/12/16  21:55:05  eric
**  Initial revision
**
**
**--
*/

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
