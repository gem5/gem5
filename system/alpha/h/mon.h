#ifndef __MON_H_LOADED
#define __MON_H_LOADED
/*****************************************************************************

       Copyright © 1993, 1994 Digital Equipment Corporation,
                       Maynard, Massachusetts.

                        All Rights Reserved

Permission to use, copy, modify, and distribute this software and its
documentation for any purpose and without fee is hereby granted, provided
that the copyright notice and this permission notice appear in all copies
of software and supporting documentation, and that the name of Digital not
be used in advertising or publicity pertaining to distribution of the software
without specific, written prior permission. Digital grants this permission
provided that you prominently mark, as not part of the original, any
modifications made to this software or documentation.

Digital Equipment Corporation disclaims all warranties and/or guarantees
with regard to this software, including all implied warranties of fitness for
a particular purpose and merchantability, and makes no representations
regarding the use of, or the results of the use of, the software and
documentation in terms of correctness, accuracy, reliability, currentness or
otherwise; and you rely on the software, documentation and results solely at
your own risk.

******************************************************************************/

/*
 *  $Id: mon.h,v 1.1.1.1 1997/10/30 23:27:16 verghese Exp $;
 */

/*
 * $Log: mon.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:16  verghese
 * current 10/29/97
 *
 * Revision 1.1  1995/11/30 18:01:05  berc
 * Initial revision
 *
 * Revision 1.15  1995/02/23  21:49:05  fdh
 * Added prototype for CompareMem().
 *
 * Revision 1.14  1994/11/08  21:43:55  fdh
 * Added include for lib.h
 *
 * Revision 1.13  1994/08/05  20:13:47  fdh
 * Updated Copyright header and RCS $Id: identifier.
 *
 * Revision 1.12  1994/06/23  13:43:18  rusling
 * Fixed up WNT compiler warnings.
 *
 * Revision 1.11  1994/06/20  14:18:59  fdh
 * Fixup header file preprocessor #include conditionals.
 *
 * Revision 1.10  1994/06/17  19:34:01  fdh
 * Clean-up...
 *
 * Revision 1.9  1994/06/13  15:15:03  rusling
 * Added more function prototypes.
 *
 * Revision 1.8  1994/06/03  20:15:43  fdh
 * Removed lib.h include and declaration for memtest.
 *
 * Revision 1.7  1994/04/03  00:35:49  fdh
 * Modified PrintMem() prototype.
 *
 * Revision 1.6  1994/04/01  22:41:46  fdh
 * Removed obsoleted global variable.
 *
 * Revision 1.5  1994/03/16  00:12:01  fdh
 * Added some prototypes...
 *
 * Revision 1.4  1994/03/13  14:37:45  fdh
 * Modified memtest prototype.
 *
 * Revision 1.3  1994/01/19  10:22:28  rusling
 * Ported to ALpha Windows NT.
 *
 * Revision 1.2  1993/10/02  23:37:53  berent
 * Merge in development by Anthony Berent
 *
 *>> Revision 1.2  1993/08/06  10:45:27  berent
 *>> Added MONITOR_ETHER_DEVICE constant. This defines which ethernet device the
 *>> monitor uses.
 *
 * Revision 1.1  1993/06/08  19:56:15  fdh
 * Initial revision
 *
 */

/* definitions for basic things */

#include "system.h"
#include "lib.h"

extern ul bootadr;

/* some constants */
#define RDFILE 0
#define VERIFY 1
#define MONITOR_ETHER_DEVICE 0  /* Ethernet port number to be used by monitor */

/*::::::::::::::
cmd.c
::::::::::::::*/
extern void user_main(void);
extern int main(void);

/*::::::::::::::
crt.c
::::::::::::::*/
extern void exit(void);

/*::::::::::::::
dis.c
::::::::::::::*/
extern void dis(int argc , ul first , ul last);

/*::::::::::::::
ffexec.c
::::::::::::::*/
extern void PutDotChar(register char c);
extern void CallIt(int (* adr)(int argc, ul val1, ul val2, ul val3, ul val4) ,
                   int argc , ul val1 , ul val2 , ul val3 , ul val4);
extern void ExecuteProgram(int (* place)(void));
extern ul CurrentDefault(void);
extern void PrintMem(int size , ul first , ul last , ul iterations , int silent, int virtual);
extern void CompareMem(ul first, ul last, ul copy);
extern void ChecksumMem(ul first , ul last);
extern void ChangeMem(int size , ul place, int virtual);
extern void FillMem(ul first , ul last , ui value);
extern void ctty(int x);
extern void tip(int port);
extern void PrintVersion(void);
extern void StackTrace(void);
extern void jToPal(ul bootadr);
extern ul csrv(ul data , ul address , int select);
extern int get_bootadr(void);

/*::::::::::::::
ffsrec.c
::::::::::::::*/
extern void GetRecord(int port);
extern int Download(void);
extern ui GetHex1(void);
extern ui GetHex2(ui in);
extern ui GetHex4(ui in);
extern void GetData(void);
extern void CheckChecksum(void);

/*::::::::::::::
pReg.c
::::::::::::::*/
extern void printReg(ul * register_array , int group);
extern void changeReg(ul * register_array , int group , int index , ul data);

#endif /* __MON_H_LOADED */
