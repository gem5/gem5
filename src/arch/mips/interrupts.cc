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
 * Authors: Richard Strong
 */


#include "arch/mips/pra_constants.hh"
#include "arch/mips/isa_traits.hh"
#include "cpu/thread_context.hh"
#include "arch/mips/interrupts.hh"

namespace MipsISA
{
static inline uint8_t getCauseIP_(ThreadContext *tc) {
    MiscReg cause = tc->readMiscRegNoEffect(MipsISA::Cause);
    uint8_t IP_ = bits(cause,Cause_IP7, Cause_IP0);
    return IP_;
}

static inline void setCauseIP_(ThreadContext *tc, uint8_t val) {
    MiscReg cause = tc->readMiscRegNoEffect(MipsISA::Cause);
    replaceBits(cause,Cause_IP7,Cause_IP0,val);
    tc->setMiscRegNoEffect(MipsISA::Cause,cause);
}

/*
  void Interrupts::post(int int_num, int index)
  {
  DPRINTF(Interrupt, "Interrupt %d posted\n", int_num);

  //index should not be used
  assert(index == 0);

  if (int_num < 0 || int_num >= NumInterruptLevels)
  panic("int_num out of bounds\n");
  intstatus |= 1 << int_num;
  }

  void Interrupts::clear(int int_num, int index)
  {
  DPRINTF(Interrupt, "Interrupt %d cleared\n", int_num);

  //index should not be used
  assert(index == 0);

  if (int_num < 0 || int_num >= NumInterruptLevels)
  panic("int_num out of bounds\n");

  intstatus &= ~(1 << int_num);
  }

  void Interrupts::clear_all()
  {
  DPRINTF(Interrupt, "Interrupts all cleared\n");
  intstatus = 0;
  }



  Fault Interrupts::getInterrupt(ThreadContext * tc)
  {
  DPRINTF(Interrupt, "Interrupts getInterrupt\n");
  // If a timer interrupt has occured, check to see if a
  // mtc0 to Compare register caused this interrupt to
  // be cleared. If this is the case, clear intstatus
  // bit for timer interrupt
  if (oncputimerintr){
  DPRINTF(Interrupt, "Interrupts oncputimerintr==true\n");
  MiscReg cause = tc->readMiscRegNoEffect(MipsISA::Cause);
  uint8_t IP_ = bits(cause,Cause_IP7, Cause_IP0);
  MiscReg intctl = tc->readMiscRegNoEffect(MipsISA::IntCtl);
  uint8_t IPTI = bits(intctl, IntCtl_IPTI_HI, IntCtl_IPTI_LO);
  //mtc0 to compare must have cleared bit in IP
  if ( ((1 << IPTI) & IP_) == 0){
  clear(IPTI, 0);
  oncputimerintr=false;
  }
  }
  //if there is a on cpu timer interrupt (i.e. Compare == Count)
  //update intstatus before proceeding to interrupt
  if (onCpuTimerInterrupt(tc)){
  DPRINTF(Interrupt, "Interrupts OnCpuTimerINterrupt(tc)==true\n");
  //determine timer interrupt IP #
  MiscReg intctl = tc->readMiscRegNoEffect(MipsISA::IntCtl);
  uint8_t IPTI = bits(intctl, IntCtl_IPTI_HI, IntCtl_IPTI_LO);
  //set intstatus to correspond
  post(IPTI, 0);
  oncputimerintr=true;
  }

  //Check if there are any outstanding interrupts
  MiscReg status = tc->readMiscRegNoEffect(MipsISA::Status);
  if (bits(status, Status_IE_LO) == 1 && //interrupts must be enabled
  bits(status, Status_ERL) == 0 &&  //error level must be 0 or interrupts inhibited
  bits(status, Status_EXL) == 0 ) //exception level must be 0 or interrupts inhibited
  {
  // Software interrupts & hardware interrupts are handled in software.
  // So if any interrupt that isn't masked is detected, jump to interrupt
  // handler
  uint8_t IM, IP; //IM=interrupt mask, IP=interrupt pending
  IM = bits(status,Status_IM7,Status_IM0);
  IP = intstatus;
  //IM and IP are already correctly aligned
  if (IM & IP){
  DPRINTF(Flow, "Interrupt! IM[7:0]=%d IP[7:0]=%d \n",
  IM, IP);
  return new InterruptFault;
  }
  }

  return NoFault;

  }

  void Interrupts::updateIntrInfo(ThreadContext *tc) const
  {
  //Merge Interrupts.intstatus with mips MipISA::Status
  MiscReg cause = tc->readMiscRegNoEffect(MipsISA::Cause);
  replaceBits(cause,Cause_IP7,Cause_IP0,intstatus);
  tc->setMiscRegNoEffect(MipsISA::Cause,cause);
  }

  bool Interrupts::onCpuTimerInterrupt(ThreadContext * tc) const
  {
  MiscReg compare = tc->readMiscRegNoEffect(MipsISA::Compare);
  MiscReg count = tc->readMiscRegNoEffect(MipsISA::Count);
  if (compare == count)
  return true;
  return false;
  }


  uint64_t Interrupts::get_vec(int int_num)
  {
  panic("MipsISA::Interrupts::get_vec() is not implemented. \n");
  M5_DUMMY_RETURN
  }
*/
void Interrupts::post(int int_num, ThreadContext* tc)
{
    DPRINTF(Interrupt, "Interrupt %d posted\n", int_num);
    if (int_num < 0 || int_num >= NumInterruptLevels)
        panic("int_num out of bounds\n");

    uint8_t intstatus= getCauseIP_(tc);
    intstatus |= 1 << int_num;
    setCauseIP_(tc, intstatus);
}

void Interrupts::post(int int_num, int index)
{
    fatal("Must use Thread COntext when posting MIPS Interrupts in M5");
}

void Interrupts::clear(int int_num, ThreadContext* tc)
{
    DPRINTF(Interrupt, "Interrupt %d cleared\n", int_num);
    if (int_num < 0 || int_num >= NumInterruptLevels)
        panic("int_num out of bounds\n");

    uint8_t intstatus = getCauseIP_(tc);
    intstatus &= ~(1 << int_num);
    setCauseIP_(tc, intstatus);
}

void Interrupts::clear(int int_num, int index)
{
    fatal("Must use Thread COntext when clearing MIPS Interrupts in M5");
}

void Interrupts::clear_all(ThreadContext *tc)
{
    DPRINTF(Interrupt, "Interrupts all cleared\n");
    uint8_t intstatus = 0;
    setCauseIP_(tc, intstatus);
}

void Interrupts::clear_all()
{
    fatal("Must use Thread COntext when clearing MIPS Interrupts in M5");
}



Fault Interrupts::getInterrupt(ThreadContext * tc)
{
    DPRINTF(Interrupt, "Interrupts getInterrupt\n");



    //Check if there are any outstanding interrupts
    MiscReg status = tc->readMiscRegNoEffect(MipsISA::Status);
    if (bits(status, Status_IE_LO) == 1 && //interrupts must be enabled
        bits(status, Status_ERL_HI,Status_ERL_LO) == 0 &&  //error level must be 0 or interrupts inhibited
        bits(status, Status_EXL_HI,Status_EXL_LO) == 0 ) //exception level must be 0 or interrupts inhibited
    {
        // Software interrupts & hardware interrupts are handled in software.
        // So if any interrupt that isn't masked is detected, jump to interrupt
        // handler
        uint8_t IM, IP; //IM=interrupt mask, IP=interrupt pending
        IM = bits(status,Status_IM7,Status_IM0);
        IP = getCauseIP_(tc);
        //IM and IP are already correctly aligned
        if (IM & IP){
            DPRINTF(Interrupt, "Interrupt! IM[7:0]=%d IP[7:0]=%d \n",
                    IM, IP);
            return new InterruptFault;
        }
    }

    return NoFault;

}
bool Interrupts::onCpuTimerInterrupt(ThreadContext * tc) const
{
    MiscReg compare = tc->readMiscRegNoEffect(MipsISA::Compare);
    MiscReg count = tc->readMiscRegNoEffect(MipsISA::Count);
    if (compare == count && count != 0)
        return true;
    return false;
}
void Interrupts::updateIntrInfo(ThreadContext *tc) const
{
    //Nothing needs to be done.
    ;
}

uint64_t Interrupts::get_vec(int int_num)
{
    panic("MipsISA::Interrupts::get_vec() is not implemented. \n");
    M5_DUMMY_RETURN
        }

bool Interrupts::interruptsPending(ThreadContext *tc) const
{
    //if there is a on cpu timer interrupt (i.e. Compare == Count)
    //update CauseIP before proceeding to interrupt
    if (onCpuTimerInterrupt(tc)){
        DPRINTF(Interrupt, "Interrupts OnCpuTimerINterrupt(tc)==true\n");
        //determine timer interrupt IP #
        MiscReg intctl = tc->readMiscRegNoEffect(MipsISA::IntCtl);
        uint8_t IPTI = bits(intctl, IntCtl_IPTI_HI, IntCtl_IPTI_LO);
        //set intstatus to correspond
        //post(IPTI, tc);
        uint8_t intstatus= getCauseIP_(tc);
        intstatus |= 1 << IPTI;
        setCauseIP_(tc, intstatus);
    }

    return (getCauseIP_(tc) != 0);

}





}
