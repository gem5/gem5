/*
 * Copyright (c) 2007 MIPS Technologies, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
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
 * Authors: Rick Strong
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

