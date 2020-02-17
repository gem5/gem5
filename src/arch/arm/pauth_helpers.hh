// -*- mode:c++ -*-

// Copyright (c) 2020 Metempsy Technology Consulting
// All rights reserved
//
// The license below extends only to copyright in the software and shall
// not be construed as granting a license to any other intellectual
// property including but not limited to intellectual property relating
// to a hardware implementation of the functionality of the software
// licensed hereunder.  You may use the software subject to the license
// terms below provided that you ensure that this notice is replicated
// unmodified and in its entirety in all distributions of the software,
// modified or unmodified, in source code or in binary form.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met: redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer;
// redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution;
// neither the name of the copyright holders nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef __ARCH_ARM_PAUTH_HELPERS_HH__
#define __ARCH_ARM_PAUTH_HELPERS_HH__

#include "arch/arm/qarma.hh"
#include "arch/arm/system.hh"
#include "arch/arm/types.hh"
#include "arch/arm/utility.hh"
#include "base/bitfield.hh"
#include "base/bitunion.hh"
#include "cpu/thread_context.hh"

namespace ArmISA
{

  inline bool
  upperAndLowerRange(ThreadContext* tc, ExceptionLevel el)
  {
      HCR hcr = tc->readMiscReg(MISCREG_HCR_EL2);
      return (el == EL1 || el == EL0 || (el == EL2 && hcr.e2h == 1));
  }

  bool
  calculateTBI(ThreadContext* tc, ExceptionLevel el, uint64_t ptr, bool data);

  int
  calculateBottomPACBit(ThreadContext* tc, ExceptionLevel el, bool top_bit);

  Fault
  trapPACUse(ThreadContext *tc, ExceptionLevel el);



  // AddPAC()
  // ========
  // Calculates the pointer authentication code for a 64-bit quantity
  // and then inserts that into pointer authentication code field of that
  // 64-bit quantity.

  uint64_t
  addPAC (ThreadContext* tc, ExceptionLevel el, uint64_t  ptr,
          uint64_t modifier, uint64_t k1, uint64_t k0, bool data);


  uint64_t
  auth(ThreadContext *tc, ExceptionLevel el, uint64_t ptr, uint64_t modifier,
       uint64_t k1, uint64_t K0, bool data, uint8_t errorcode);

  Fault
  authDA(ThreadContext* tc, uint64_t X, uint64_t Y, uint64_t* out);

  Fault
  authDB(ThreadContext* tc, uint64_t X, uint64_t Y, uint64_t* out);


  Fault
  authIA(ThreadContext* tc, uint64_t X, uint64_t Y, uint64_t* out);

  Fault
  authIB(ThreadContext* tc, uint64_t X, uint64_t Y, uint64_t* out);

  Fault
  addPACDA(ThreadContext* tc, uint64_t X, uint64_t Y, uint64_t* out);

  Fault
  addPACDB(ThreadContext* tc, uint64_t X, uint64_t Y, uint64_t* out);

  Fault
  addPACGA(ThreadContext* tc, uint64_t X, uint64_t Y, uint64_t* out);

  Fault
  addPACIA(ThreadContext* tc, uint64_t X, uint64_t Y, uint64_t* out);

  Fault
  addPACIB(ThreadContext* tc, uint64_t X, uint64_t Y, uint64_t* out);

  //  Strip()
  //    =======
  //      Strip() returns a 64-bit value containing A, but replacing the
  // pointer authentication code field bits with the extension of the
  // address bits. This can apply to either instructions or data, where,
  // as the use of tagged pointers is distinct, it might be
  // handled differently.

  Fault
  stripPAC(ThreadContext* tc, uint64_t A, bool data, uint64_t* out);

};
#endif //__ARCH_ARM_PAUTH_HELPERS_HH__
