/*
Copyright 1993Hewlett-Packard Development Company, L.P.

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


#ifndef __REGDEFS_H_LOADED
#define __REGDEFS_H_LOADED

/*
 *  $Id: regdefs.h,v 1.1.1.1 1997/10/30 23:27:17 verghese Exp $;
 */

/*
 * $Log: regdefs.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:17  verghese
 * current 10/29/97
 *
 * Revision 1.2  1995/02/24  16:00:18  fdh
 * Conditional around #define AT.
 *
 * Revision 1.1  1995/02/24  15:54:26  fdh
 * Initial revision
 *
 */

#define v0	$0
#define t0	$1
#define t1	$2
#define t2	$3
#define t3	$4
#define t4	$5
#define t5	$6
#define t6	$7
#define t7	$8
#define s0	$9
#define s1	$10
#define s2	$11
#define s3	$12
#define s4	$13
#define s5	$14
#define s6	$15
#define fp	$15	/* fp & s6 are the same	*/
#define a0	$16
#define a1	$17
#define a2	$18
#define a3	$19
#define a4	$20
#define a5	$21
#define t8	$22
#define t9	$23
#define t10	$24
#define t11	$25
#define ra	$26
#define pv	$27	/* pv and t5 are the same */
#define t12	$27
#ifndef AT
#define AT	$at
#endif
#define gp	$29
#define	sp	$30
#define zero	$31

#endif /* __REGDEFS_H_LOADED */
