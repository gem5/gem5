/*
 * Copyright © 2007 MIPS Technologies, Inc.  All Rights Reserved
 *
 * This software is part of the M5 simulator.
 *
 * THIS IS A LEGAL AGREEMENT.  BY DOWNLOADING, USING, COPYING, CREATING
 * DERIVATIVE WORKS, AND/OR DISTRIBUTING THIS SOFTWARE YOU ARE AGREEING
 * TO THESE TERMS AND CONDITIONS.
 *
 * Permission is granted to use, copy, create derivative works and
 * distribute this software and such derivative works for any purpose,
 * so long as (1) the copyright notice above, this grant of permission,
 * and the disclaimer below appear in all copies and derivative works
 * made, (2) the copyright notice above is augmented as appropriate to
 * reflect the addition of any new copyrightable work in a derivative
 * work (e.g., Copyright © <Publication Year> Copyright Owner), and (3)
 * the name of MIPS Technologies, Inc. (“MIPS”) is not used in any
 * advertising or publicity pertaining to the use or distribution of
 * this software without specific, written prior authorization.
 *
 * THIS SOFTWARE IS PROVIDED “AS IS.”  MIPS MAKES NO WARRANTIES AND
 * DISCLAIMS ALL WARRANTIES, WHETHER EXPRESS, STATUTORY, IMPLIED OR
 * OTHERWISE, INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
 * NON-INFRINGEMENT OF THIRD PARTY RIGHTS, REGARDING THIS SOFTWARE.
 * IN NO EVENT SHALL MIPS BE LIABLE FOR ANY DAMAGES, INCLUDING DIRECT,
 * INDIRECT, INCIDENTAL, CONSEQUENTIAL, SPECIAL, OR PUNITIVE DAMAGES OF
 * ANY KIND OR NATURE, ARISING OUT OF OR IN CONNECTION WITH THIS AGREEMENT,
 * THIS SOFTWARE AND/OR THE USE OF THIS SOFTWARE, WHETHER SUCH LIABILITY
 * IS ASSERTED ON THE BASIS OF CONTRACT, TORT (INCLUDING NEGLIGENCE OR
 * STRICT LIABILITY), OR OTHERWISE, EVEN IF MIPS HAS BEEN WARNED OF THE
 * POSSIBILITY OF ANY SUCH LOSS OR DAMAGE IN ADVANCE.
 *
 * Authors: Korey L. Sewell
 *
 */

#include "arch/mips/linux/linux.hh"

#include <fcntl.h>

// open(2) flags translation table
OpenFlagTransTable MipsLinux::openFlagTable[] = {
#ifdef _MSC_VER
  { MipsLinux::TGT_O_RDONLY,	_O_RDONLY },
  { MipsLinux::TGT_O_WRONLY,	_O_WRONLY },
  { MipsLinux::TGT_O_RDWR,	_O_RDWR },
  { MipsLinux::TGT_O_APPEND,	_O_APPEND },
  { MipsLinux::TGT_O_CREAT,	_O_CREAT },
  { MipsLinux::TGT_O_TRUNC,	_O_TRUNC },
  { MipsLinux::TGT_O_EXCL,	_O_EXCL },
#ifdef _O_NONBLOCK
  { MipsLinux::TGT_O_NONBLOCK,	_O_NONBLOCK },
#endif
#ifdef _O_NOCTTY
  { MipsLinux::TGT_O_NOCTTY,	_O_NOCTTY },
#endif
#ifdef _O_SYNC
  { MipsLinux::TGT_O_SYNC,	_O_SYNC },
#endif
#else /* !_MSC_VER */
  { MipsLinux::TGT_O_RDONLY,	O_RDONLY },
  { MipsLinux::TGT_O_WRONLY,	O_WRONLY },
  { MipsLinux::TGT_O_RDWR,	O_RDWR },
  { MipsLinux::TGT_O_APPEND,	O_APPEND },
  { MipsLinux::TGT_O_CREAT,	O_CREAT },
  { MipsLinux::TGT_O_TRUNC,	O_TRUNC },
  { MipsLinux::TGT_O_EXCL,	O_EXCL },
  { MipsLinux::TGT_O_NONBLOCK,	O_NONBLOCK },
  { MipsLinux::TGT_O_NOCTTY,	O_NOCTTY },
#ifdef O_SYNC
  { MipsLinux::TGT_O_SYNC,	O_SYNC },
#endif
#endif /* _MSC_VER */
};

const int MipsLinux::NUM_OPEN_FLAGS =
        (sizeof(MipsLinux::openFlagTable)/sizeof(MipsLinux::openFlagTable[0]));




