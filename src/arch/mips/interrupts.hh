/*
 * Copyright N) 2007 MIPS Technologies, Inc.  All Rights Reserved
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
 * work (e.g., Copyright N) <Publication Year> Copyright Owner), and (3)
 * the name of MIPS Technologies, Inc. ($(B!H(BMIPS$(B!I(B) is not used in any
 * advertising or publicity pertaining to the use or distribution of
 * this software without specific, written prior authorization.
 *
 * THIS SOFTWARE IS PROVIDED $(B!H(BAS IS.$(B!I(B  MIPS MAKES NO WARRANTIES AND
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
 * Authors: Richard Strong
 */

#ifndef __ARCH_MIPS_INTERRUPT_HH__
#define __ARCH_MIPS_INTERRUPT_HH__


#include "arch/mips/faults.hh"
#include "base/compiler.hh"



namespace MipsISA
{
class Interrupts
{
    /*
      protected:
      uint8_t intstatus;
      bool oncputimerintr;
      public:
      Interrupts()
      {
      intstatus = 0;
      newInfoSet = false;
      oncputimerintr = false;

      }
      //  post(int int_num, int index) is responsible
      //  for posting an interrupt. It sets a bit
      //  in intstatus corresponding to Cause IP*. The
      //  MIPS register Cause is updated by updateIntrInfo
      //  which is called by check_interrupts
      //
      void post(int int_num, int index);
      // clear(int int_num, int index) is responsible
      //  for clearing an interrupt. It clear a bit
      //  in intstatus corresponding to Cause IP*. The
      //  MIPS register Cause is updated by updateIntrInfo
      //  which is called by check_interrupts
      //
      void clear(int int_num, int index);
      //  clear_all() is responsible
      //  for clearing all interrupts. It clears all bits
      //  in intstatus corresponding to Cause IP*. The
      //  MIPS register Cause is updated by updateIntrInfo
      //  which is called by check_interrupts
      //
      void clear_all();

      // getInterrupt(ThreadContext * tc) checks if an interrupt
      //  should be returned. It ands the interrupt mask and
      //  and interrupt pending bits to see if one exists. It
      //  also makes sure interrupts are enabled (IE) and
      //  that ERL and ERX are not set
      //
      Fault getInterrupt(ThreadContext * tc);

      // updateIntrInfo(ThreadContext *tc) const syncs the
      //  MIPS cause register with the instatus variable. instatus
      //  is essentially a copy of the MIPS cause[IP7:IP0]
      //
      void updateIntrInfo(ThreadContext *tc) const;
      void updateIntrInfoCpuTimerIntr(ThreadContext *tc) const;
      bool onCpuTimerInterrupt(ThreadContext *tc) const;

      uint64_t get_vec(int int_num);

      bool check_interrupts(ThreadContext * tc) const{
      //return (intstatus != 0) && !(tc->readPC() & 0x3);
      if (oncputimerintr == false){
      updateIntrInfo(tc);
      return ((intstatus != 0) || onCpuTimerInterrupt(tc));
      }
      else
      return true;

      }
    */


  protected:
    //uint8_t intstatus;
    //bool oncputimerintr;
  public:
    Interrupts()
    {
        //intstatus = 0;
        newInfoSet = false;
        //oncputimerintr = false;

    }
    //  post(int int_num, int index) is responsible
    //  for posting an interrupt. It sets a bit
    //  in intstatus corresponding to Cause IP*. The
    //  MIPS register Cause is updated by updateIntrInfo
    //  which is called by check_interrupts
    //
    void post(int int_num, ThreadContext* tc);
    void post(int int_num, int index);

    // clear(int int_num, int index) is responsible
    //  for clearing an interrupt. It clear a bit
    //  in intstatus corresponding to Cause IP*. The
    //  MIPS register Cause is updated by updateIntrInfo
    //  which is called by check_interrupts
    //
    void clear(int int_num, ThreadContext* tc);
    void clear(int int_num, int index);

    //  clear_all() is responsible
    //  for clearing all interrupts. It clears all bits
    //  in intstatus corresponding to Cause IP*. The
    //  MIPS register Cause is updated by updateIntrInfo
    //  which is called by check_interrupts
    //
    void clear_all(ThreadContext* tc);
    void clear_all();

    // getInterrupt(ThreadContext * tc) checks if an interrupt
    //  should be returned. It ands the interrupt mask and
    //  and interrupt pending bits to see if one exists. It
    //  also makes sure interrupts are enabled (IE) and
    //  that ERL and ERX are not set
    //
    Fault getInterrupt(ThreadContext * tc);

    // updateIntrInfo(ThreadContext *tc) const syncs the
    //  MIPS cause register with the instatus variable. instatus
    //  is essentially a copy of the MIPS cause[IP7:IP0]
    //
    void updateIntrInfo(ThreadContext *tc) const;
    bool interruptsPending(ThreadContext *tc) const;
    bool onCpuTimerInterrupt(ThreadContext *tc) const;

    uint64_t get_vec(int int_num);

    bool check_interrupts(ThreadContext * tc) const{
        return interruptsPending(tc);
    }


    void serialize(std::ostream &os)
    {
        fatal("Serialization of Interrupts Unimplemented for MIPS");
        //SERIALIZE_ARRAY(interrupts, NumInterruptLevels);
        //SERIALIZE_SCALAR(intstatus);
    }

    void unserialize(Checkpoint *cp, const std::string &section)
    {
        fatal("Unserialization of Interrupts Unimplemented for MIPS");
        //UNSERIALIZE_ARRAY(interrupts, NumInterruptLevels);
        //UNSERIALIZE_SCALAR(intstatus);
    }



  private:
    bool newInfoSet;
    int newIpl;
    int newSummary;

};

}

#endif

