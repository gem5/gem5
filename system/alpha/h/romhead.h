#ifndef __ROMHEAD_H_LOADED
#define __ROMHEAD_H_LOADED
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
 *  $Id: romhead.h,v 1.1.1.1 1997/10/30 23:27:17 verghese Exp $;
 */

/*
 * $Log: romhead.h,v $
 * Revision 1.1.1.1  1997/10/30 23:27:17  verghese
 * current 10/29/97
 *
 * Revision 1.16  1995/03/05  00:51:01  fdh
 * Fixed up a couple of MACROS.
 *
 * Revision 1.15  1995/03/04  05:55:57  fdh
 * Bracket COMPUTE_CHECKSUM Macro.
 *
 * Revision 1.14  1995/02/27  15:36:21  fdh
 * Removed ul definitions. Replaced with structs of ui's.
 *
 * Revision 1.13  1995/02/16  22:06:06  fdh
 * Changed Open VMS to OpenVMS because of some trademark junk.
 *
 * Revision 1.12  1995/02/14  21:19:33  cruz
 * Fixed up header picture to match implementation.
 *
 * Revision 1.11  1995/02/10  02:19:03  fdh
 * Created COMPUTE_CHECKSUM Macro.
 * Corrected prototypes.
 *
 * Revision 1.10  1995/02/08  00:34:44  fdh
 * Added ROMH_VERSION and ROTATE_RIGHT macros.
 *
 * Revision 1.9  1995/02/07  22:28:09  fdh
 * Changed Windows NT alias from "WNT" to "NT".
 *
 * Revision 1.8  1995/02/07  04:54:33  fdh
 * Added ostype definitions.
 * Modified ROM_HEADER_CHECKSUM Macro.
 *
 * Revision 1.7  1995/02/06  02:47:40  fdh
 * Added prototypes for romhead.c routines.
 *
 * Revision 1.6  1995/02/05  01:53:05  fdh
 * Modified the definition for the ROM header.
 * Added a MACRO that can be used to access the
 * ROM header checksum which can be in different location
 * for different versions of the header.
 *
 * Revision 1.5  1995/02/02  20:05:31  fdh
 * Moved fwid_array[] initialization to header file.
 *
 * Revision 1.4  1995/01/31  23:44:00  fdh
 * Added field for "OPTIONAL FW ID, continued" to make
 * the optional field really 7 bytes.
 *
 * Revision 1.3  1995/01/31  23:21:25  fdh
 * Updated...
 *
 * Revision 1.2  1995/01/23  22:34:32  fdh
 * Extended the ROM header spec.  This is now version one
 * which is a super-set of the version 0 header.
 *
 * Revision 1.1  1994/11/19  03:47:29  fdh
 * Initial revision
 *
 *
 *
 *	Special ROM header
 *	==================
 *	The System ROM can contain multiple ROM images, each with
 *	its own header. That header tells the SROM where to load
 *	the image and also if it has been compressed with the
 *	"makerom" tool.  For System ROMs which contain a single
 *	image, the header is optional.  If the header does not
 *	exist the complete System ROM is loaded and executed at
 *	physical address zero.
 *
 *	31                                           0
 *	+---------------------------------------------+
 *	|   VALIDATION PATTERN 0x5A5AC3C3             |	0x00
 *	+---------------------------------------------+
 *	|   INVERSE VALIDATION PATTERN 0xA5A53C3C     |	0x04
 *	+---------------------------------------------+
 *	|   HEADER SIZE (Bytes)                       |	0x08
 *	+---------------------------------------------+
 *	|   IMAGE CHECKSUM                            |	0x0C
 *	+---------------------------------------------+
 *	|   IMAGE SIZE (Memory Footprint)             |	0x10
 *	+---------------------------------------------+
 *	|   DECOMPRESSION FLAG                        |	0x14
 *	+---------------------------------------------+
 *	|   DESTINATION ADDRESS LOWER LONGWORD        |	0x18
 *	+---------------------------------------------+
 *	|   DESTINATION ADDRESS UPPER LONGWORD        |	0x1C
 *	+---------------------------------------------+
 *	|   RSVD<31:16>| ID <15:8>| HEADER REV <7:0>  |	0x20  \
 *	+---------------------------------------------+        \
 *	|   ROM IMAGE SIZE                            |	0x24    |
 *	+---------------------------------------------+         |  New for
 *	|   OPTIONAL FIRMWARE ID <31:0>               |	0x28    +- Rev 1
 *	+---------------------------------------------+         |  headers.
 *	|   OPTIONAL FIRMWARE ID <63:32>              |	0x2C    |
 *	+---------------------------------------------+        /
 *	|   HEADER CHECKSUM (excluding this field)    |	0x30  /
 *	+---------------------------------------------+
 *
 *	VALIDATION PATTERN
 *	------------------
 *	The first quadword contains a special signature pattern
 *	that is used to verify that this "special" ROM header
 *	has been located.  The pattern is 0x5A5AC3C3A5A53C3C.
 *
 *	HEADER SIZE (Bytes)
 *	-------------------
 *	The header size is the next longword.  This is provided
 *	to allow for some backward compatibility in the event that
 *	the header is extended in the future.  When the header
 *	is located, current versions of SROM code determine where
 *	the image begins based on the header size.  Additional data
 *	added to the header in the future will simply be ignored
 *	by current SROM code. Additionally, the header size = 0x20
 *	implies version 0 of this header spec.  For any other size
 *      see HEADER REVISION to determine header version.
 *
 *
 *	IMAGE CHECKSUM
 *	--------------
 *	The next longword contains the image checksum.  This is
 *	used to verify the integrity of the ROM.  Checksum is computed
 *      in the same fashion as the header checksum.
 *
 *	IMAGE SIZE (Memory Footprint)
 *	-----------------------------
 *	The image size reflects the size of the image after it has
 *	been loaded into memory from the ROM. See ROM IMAGE SIZE.
 *
 *	DECOMPRESSION FLAG
 *	------------------
 *	The decompression flag tells the SROM code if the makerom
 *	tool was used to compress the ROM image with a "trivial
 *	repeating byte algorithm".  The SROM code contains routines
 *	which perform this decompression algorithm.  Other
 *	compression/decompression schemes may be employed which work
 *	independently from this one.
 *
 *	DESTINATION ADDRESS
 *	-------------------
 *	This quadword contains the destination address for the
 *	image.  The SROM code  will begin loading the image at this
 *	address and subsequently begin its execution there.
 *
 *	HEADER REV
 *	----------
 *	The revision of the header specifications used in this
 *	header.  This is necessary to provide compatibility to
 *	future changes to this header spec.  Version 0 headers
 *	are identified by the size of the header.  See HEADER
 *	SIZE.
 *
 *	FIRMWARE ID
 *	-----------
 *	The firmware ID is a byte that specifies the firmware type.
 *	This facilitates image boot options necessary to boot
 *	different operating systems.
 *
 *		  firmware
 *	firmware    type
 *	--------  --------
 *	  DBM	     0	     Alpha Evaluation Boards Debug Monitor
 *	  WNT        1       Windows NT Firmware
 *	  SRM        2       Alpha System Reference Manual Console
 *
 *	ROM IMAGE SIZE
 *	--------------
 *	The ROM image size reflects the size of the image as it is
 *	contained in the ROM. See IMAGE SIZE.
 *
 *	OPTIONAL FW ID
 *	--------------
 *	This is an optional field that can be used to provide
 *	additional firmware information such as firmware revision
 *	or a character descriptive string up to 8 characters.
 *
 *	HEADER CHECKSUM
 *	---------------
 *	The checksum of the header.  This is used to validate
 *	the presence of a header beyond the validation provided
 *	by the validation pattern.  See VALIDATION PATTERN.
 *	The header checksum is computed from the beginning of
 *	the header up to but excluding the header checksum
 *	field itself.  If there are future versions of this
 *	header the header checksum should always be the last
 *	field defined in the header.  The checksum algorithm used
 *	is compatible with the standard BSD4.3 algorithm provided
 *	on most implementations of Unix.  Algorithm: The checksum
 *	is rotated right by one bit around a 16 bit field before
 *	adding in the value of each byte.
 *
 */

#include "local.h"       /* Include environment specific definitions */

#define ROM_H_SIGNATURE 0x5A5AC3C3
#define ROM_H_REVISION  1


typedef union {
  struct {
    /*
     * Version 0 definition of the ROM header.
     */
    struct {
      ui signature;		/* validation signature                    */
      ui csignature;		/* inverse validation signature            */
      ui hsize;			/* header size                             */
      ui checksum;		/* checksum                                */
      ui size;			/* image size (Memory Footprint)           */
      ui decomp;		/* decompression algorithm                 */
      struct {
        ui low;
        ui high;
      } destination; 	/* destination address                     */
    } V0;

    /*
     * Version 1 extensions to the ROM header.
     */
    struct {
      char hversion;		/* ROM header version   (Byte 0)            */
      char fw_id;		/* Firmware ID          (Byte 1)            */
      char reserved[2];		/* Reserved             (Bytes 3:2)         */
      ui rimage_size;		/* ROM image size                           */
      union {
        char id[8];		/* Optional Firmware ID (character array)   */
        struct {
          ui low;
          ui high;
        } id_S;
      } fwoptid;
    } V1;

    /*
     * Future extensions to the header should be included before
     * this header checksum.  (See HEADER CHECKSUM description)
     */
    ui hchecksum;		/* Header checksum, (Always last entry)     */
  } romh;
  ui romh_array[1];		/* To allow longword access to the data     */
} romheader_t;

/*
 * Registered Firmware types.
 */
#define FW_DBM 0
#define FW_WNT 1
#define FW_SRM 2

#define FW_DBM_STRINGS "Alpha Evaluation Board Debug Monitor", "DBM", "Debug Monitor", "Monitor", NULL
#define FW_WNT_STRINGS "Windows NT Firmware",                  "WNT", "NTFW", "ARC", "NT", NULL
#define FW_SRM_STRINGS "Alpha SRM Console",                    "SRM", "VMS", "OSF", NULL

typedef struct fw_id {
  int firmware_id;
  char **id_string;
} fw_id_t;

extern fw_id_t fwid_array[];

#define FW_OSTYPE_DBM 0
#define FW_OSTYPE_WNT 1
#define FW_OSTYPE_VMS 2
#define FW_OSTYPE_OSF 3

#define OS_DBM_STRINGS "Alpha Evaluation Board Debug Monitor", "DBM"
#define OS_WNT_STRINGS "The Windows NT Operating System",      "NT"
#define OS_VMS_STRINGS "OpenVMS",                              "VMS"
#define OS_OSF_STRINGS "DEC OSF/1",                            "OSF"

typedef struct os_types {
  int ostype;
  int firmware_id;
  char **id_string;
} ostype_t;

extern ostype_t ostype_array[];

/*
 * The ROM header checksum should always be assigned to the last
 * field in the header.  Therefore, when reading headers created
 * by various versions of makerom the ROM header checksum can be
 * in different locations.  This macro can be used to access the
 * ROM header checksum in the proper location.
 */
#define ROM_HEADER_CHECKSUM(x) \
  ((x)->romh_array[((x)->romh.V0.hsize - sizeof((x)->romh.hchecksum))/sizeof(ui)])

/*
 * Macro to provide the header version number
 */
#define ROMH_VERSION(x) ((x)->romh.V0.hsize == 0x20 ? 0 : (x)->romh.V1.hversion)

/*
 * Macro to assist in computing the BSD4.3 style checksum.
 */
#define ROTATE_RIGHT(x) if ((x) & 1) (x) = ((x) >>1) + 0x8000; else (x) = (x) >>1;

/*
 * Macro used to increment the checksum
 * by a new byte while keeping the total
 * checksum within the 16 bit range.
 */
#define COMPUTE_CHECKSUM(c,k) \
   {ROTATE_RIGHT(k); k += (ub) c; k &= 0xffff;}

/*
 * romheader.c prototypes
 */
extern fw_id_t * fwid_match(char * arg);
extern fw_id_t * fwid_match_i(int fwid);
extern void fwid_dump(char * pre);
extern ostype_t * ostype_match(char * arg);
extern ostype_t * ostype_match_i(int ostype);
extern void ostype_dump(char * pre);
extern ui compute_romh_chksum(romheader_t * header);
extern int dumpHeader(romheader_t * header);

#endif /* __ROMHEAD_H_LOADED */
