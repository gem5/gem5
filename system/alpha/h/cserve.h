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


/*
 *      VID: [T1.2] PT: [Fri Apr 21 16:47:20 1995] SF: [cserve.h]
 *       TI: [/sae_users/cruz/bin/vice -iplatform.s -l// -p# -DEB164 -h -m -aeb164 ]
 */
#define	__CSERVE_LOADED	1
/*
*****************************************************************************
**
**  FACILITY:
**
**	DECchip 21164 OSF/1 PALcode
**
**  MODULE:
**
**	cserve.h
**
**  MODULE DESCRIPTION:
**
**      Platform specific cserve definitions.
**
**  AUTHOR: ES
**
**  CREATION DATE:  21-JUN-1994
**
**  $Id: cserve.h,v 1.1.1.1 1997/10/30 23:27:18 verghese Exp $
**
**  MODIFICATION HISTORY:
**
**  $Log: cserve.h,v $
**  Revision 1.1.1.1  1997/10/30 23:27:18  verghese
**  current 10/29/97
**
**  Revision 1.6  1995/04/03  17:29:52  samberg
**  Add rd_bccfg_off
**
**  Revision 1.5  1995/02/02  19:31:34  samberg
**  Added WR_BCACHE, deleted WR_BCCFG and WR_BCCTL
**
**  Revision 1.4  1994/12/08  17:13:34  samberg
**  Add CSERVE_K_WR_BCCTL and CSERVE_K_WR_BCCFG
**
**  Revision 1.3  1994/11/30  15:59:30  samberg
**  Use c-style comments for c compiler use
**
**  Revision 1.2  1994/11/22  19:02:46  samberg
**  Add constants for ev4 backward compatibility
**
**  Revision 1.2  1994/11/22  19:02:46  samberg
**  Add constants for ev4 backward compatibility
**
**  Revision 1.1  1994/07/08  17:01:40  samberg
**  Initial revision
**
**
*/

/*
** Console Service (cserve) sub-function codes:
*/
#define CSERVE_K_LDQP           0x01
#define CSERVE_K_STQP           0x02
#define CSERVE_K_JTOPAL         0x09
#define CSERVE_K_WR_INT         0x0A
#define CSERVE_K_RD_IMPURE      0x0B
#define CSERVE_K_PUTC           0x0F
#define CSERVE_K_WR_ICSR	0x10
#define CSERVE_K_WR_ICCSR	0x10    /* for ev4 backwards compatibility */
#define CSERVE_K_RD_ICSR	0x11
#define CSERVE_K_RD_ICCSR	0x11    /* for ev4 backwards compatibility */
#define CSERVE_K_RD_BCCTL	0x12
#define CSERVE_K_RD_BCCFG	0x13

#define CSERVE_K_WR_BCACHE      0x16

#define CSERVE_K_RD_BCCFG_OFF   0x17
#define CSERVE_K_JTOKERN	0x18


