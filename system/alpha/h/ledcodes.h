#ifndef __LEDCODES_H_LOADED
#define __LEDCODES_H_LOADED
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
 *  $Id: ledcodes.h,v 1.1.1.1 1997/10/30 23:27:16 verghese Exp $;
 */

/*
 * $Log: ledcodes.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:16  verghese
 * current 10/29/97
 *
 * Revision 1.2  1994/11/28  19:39:44  fdh
 * Added Startup LED codes.
 *
 * Revision 1.1  1994/11/08  21:42:29  fdh
 * Initial revision
 *
 */

#define led_k_ksp_initialized   0xFF    /* Kernel Stack Pointer Initialized */
#define led_k_sysdata_inited    0xFE    /* System Data Structure Initialized */
#define led_k_init_IO           0xFD    /* About to complete IO bus initialization */
#define led_k_IO_inited         0xFC    /* IO bus initialization complete */
#define led_k_uart_inited       0xFB    /* UARTs initialized */

/*
 * Flash ROM Utility LED codes.
 */
#define led_k_flash_entered	0xB0	/* Flash Utility Entered */
#define led_k_flash_found	0xB2	/* A FLASH ROM was found. */
#define led_k_erase_flash	0xB3	/* About to erase flash. */
#define led_k_write_flash	0xB4	/* About to write flash. */
#define led_k_verify_flash	0xB5	/* About to verify flash. */
#define led_k_flash_exit	0xBF	/* Program finished. */

#endif /* __LEDCODES_H_LOADED */
