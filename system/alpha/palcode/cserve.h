/*
 *      VID: [T1.2] PT: [Fri Apr 21 16:47:20 1995] SF: [cserve.h]
 *       TI: [/sae_users/cruz/bin/vice -iplatform.s -l// -p# -DEB164 -h -m -aeb164 ]
 */
#define	__CSERVE_LOADED	1
/*
*****************************************************************************
**                                                                          *
**  Copyright © 1993, 1994	       					    *
**  by Digital Equipment Corporation, Maynard, Massachusetts.		    *
**                                                                          *
**  All Rights Reserved							    *
**                                                                          *
**  Permission  is  hereby  granted  to  use, copy, modify and distribute   *
**  this  software  and  its  documentation,  in  both  source  code  and   *
**  object  code  form,  and without fee, for the purpose of distribution   *
**  of this software  or  modifications  of this software within products   *
**  incorporating  an  integrated   circuit  implementing  Digital's  AXP   *
**  architecture,  regardless  of the  source of such integrated circuit,   *
**  provided that the  above copyright  notice and this permission notice   *
**  appear  in  all copies,  and  that  the  name  of  Digital  Equipment   *
**  Corporation  not  be  used  in advertising or publicity pertaining to   *
**  distribution of the  document  or  software without specific, written   *
**  prior permission.							    *
**                                                                          *
**  Digital  Equipment  Corporation   disclaims  all   warranties  and/or   *
**  guarantees  with  regard  to  this  software,  including  all implied   *
**  warranties of fitness for  a  particular purpose and merchantability,   *
**  and makes  no  representations  regarding  the use of, or the results   *
**  of the use of, the software and documentation in terms of correctness,  *
**  accuracy,  reliability,  currentness  or  otherwise;  and you rely on   *
**  the software, documentation and results solely at your own risk.	    *
**                                                                          *
**  AXP is a trademark of Digital Equipment Corporation.		    *
**                                                                          *
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


