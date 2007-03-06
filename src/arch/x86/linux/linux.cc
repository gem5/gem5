/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
 *
 * Redistribution and use of this software in source and binary forms,
 * with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * The software must be used only for Non-Commercial Use which means any
 * use which is NOT directed to receiving any direct monetary
 * compensation for, or commercial advantage from such use.  Illustrative
 * examples of non-commercial use are academic research, personal study,
 * teaching, education and corporate research & development.
 * Illustrative examples of commercial use are distributing products for
 * commercial advantage and providing services using the software for
 * commercial advantage.
 *
 * If you wish to use this software or functionality therein that may be
 * covered by patents for commercial use, please contact:
 *     Director of Intellectual Property Licensing
 *     Office of Strategy and Technology
 *     Hewlett-Packard Company
 *     1501 Page Mill Road
 *     Palo Alto, California  94304
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.  Redistributions
 * in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.  Neither the name of
 * the COPYRIGHT HOLDER(s), HEWLETT-PACKARD COMPANY, nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.  No right of
 * sublicense is granted herewith.  Derivatives of the software and
 * output created using the software may be prepared, but only for
 * Non-Commercial Uses.  Derivatives of the software may be shared with
 * others provided: (i) the others agree to abide by the list of
 * conditions herein which includes the Non-Commercial Use restrictions;
 * and (ii) such Derivatives of the software include the above copyright
 * notice to acknowledge the contribution from this software where
 * applicable, this list of conditions and the disclaimer below.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Gabe Black
 */

#include "arch/x86/linux/linux.hh"
#include <fcntl.h>

// open(2) flags translation table
OpenFlagTransTable X86Linux::openFlagTable[] = {
#ifdef _MSC_VER
  { TGT_O_RDONLY, _O_RDONLY },
  { TGT_O_WRONLY, _O_WRONLY },
  { TGT_O_RDWR, _O_RDWR },
  { TGT_O_APPEND, _O_APPEND },
  { TGT_O_CREAT, _O_CREAT },
  { TGT_O_TRUNC, _O_TRUNC },
  { TGT_O_EXCL, _O_EXCL },
#ifdef _O_NONBLOCK
  { TGT_O_NONBLOCK, _O_NONBLOCK },
#endif
#ifdef _O_NOCTTY
  { TGT_O_NOCTTY, _O_NOCTTY },
#endif
#ifdef _O_SYNC
  { TGT_O_SYNC, _O_SYNC },
#endif
#else /* !_MSC_VER */
  { TGT_O_RDONLY, O_RDONLY },
  { TGT_O_WRONLY, O_WRONLY },
  { TGT_O_RDWR, O_RDWR },
  { TGT_O_APPEND, O_APPEND },
  { TGT_O_CREAT, O_CREAT },
  { TGT_O_TRUNC, O_TRUNC },
  { TGT_O_EXCL, O_EXCL },
  { TGT_O_NONBLOCK, O_NONBLOCK },
  { TGT_O_NOCTTY, O_NOCTTY },
#ifdef O_SYNC
  { TGT_O_SYNC, O_SYNC },
#endif
#endif /* _MSC_VER */
};

const int X86Linux::NUM_OPEN_FLAGS =
        (sizeof(X86Linux::openFlagTable)/sizeof(X86Linux::openFlagTable[0]));

