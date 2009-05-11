
/*
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
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
 */

/*
 * $Id: BarrierGenerator.C 1.3 2005/01/19 13:12:35-06:00 mikem@maya.cs.wisc.edu $
 *
 */

#include "BarrierGenerator.hh"
#include "Sequencer.hh"
#include "System.hh"
#include "RubyConfig.hh"
#include "SubBlock.hh"
#include "SyntheticDriver.hh"
#include "Chip.hh"

BarrierGenerator::BarrierGenerator(NodeID node, SyntheticDriver& driver) :
  m_driver(driver)
{
  m_status = BarrierGeneratorStatus_Thinking;
  m_last_transition = 0;
  m_node = node;
  m_counter = 0;
  proc_counter = 0;
  m_local_sense = false;

  m_total_think = 0;
  m_think_periods = 0;

  g_eventQueue_ptr->scheduleEvent(this, 1+(random() % 200));
}

BarrierGenerator::~BarrierGenerator()
{
}

void BarrierGenerator::wakeup()
{
  DEBUG_EXPR(TESTER_COMP, MedPrio, m_node);
  DEBUG_EXPR(TESTER_COMP, MedPrio, m_status);

  if (m_status == BarrierGeneratorStatus_Thinking) {
    m_barrier_done = false;
    m_local_sense = !m_local_sense;
    m_status = BarrierGeneratorStatus_Test_Pending;
    m_last_transition = g_eventQueue_ptr->getTime();
    initiateTest();  // Test
  } else if (m_status == BarrierGeneratorStatus_Test_Waiting) {
    m_status = BarrierGeneratorStatus_Test_Pending;
    m_last_transition = g_eventQueue_ptr->getTime();
    initiateTest();  // Test
  } else if (m_status == BarrierGeneratorStatus_Release_Waiting) {
    m_status = BarrierGeneratorStatus_Release_Pending;
    m_last_transition = g_eventQueue_ptr->getTime();
    initiateRelease();  // Test
  } else if (m_status == BarrierGeneratorStatus_StoreBarrierCounter_Waiting) {
    m_status = BarrierGeneratorStatus_StoreBarrierCounter_Pending;
    m_last_transition = g_eventQueue_ptr->getTime();
    initiateStoreCtr();
  } else if (m_status == BarrierGeneratorStatus_StoreFlag_Waiting) {
    m_status = BarrierGeneratorStatus_StoreFlag_Pending;
    m_last_transition = g_eventQueue_ptr->getTime();
    initiateStoreFlag();
  } else if (m_status == BarrierGeneratorStatus_Holding) {
    m_status = BarrierGeneratorStatus_Release_Pending;
    m_last_transition = g_eventQueue_ptr->getTime();
    initiateRelease();  // Release
  } else if (m_status == BarrierGeneratorStatus_Before_Swap) {
    m_status = BarrierGeneratorStatus_Swap_Pending;
    m_last_transition = g_eventQueue_ptr->getTime();
    initiateSwap();
  } else if (m_status == BarrierGeneratorStatus_SpinFlag_Ready) {
    m_status = BarrierGeneratorStatus_SpinFlag_Pending;
    m_last_transition = g_eventQueue_ptr->getTime();
    initiateLoadFlag();
  } else {
    WARN_EXPR(m_status);
    ERROR_MSG("Invalid status");
  }
}

void BarrierGenerator::performCallback(NodeID proc, SubBlock& data)
{
  Address address = data.getAddress();
  assert(proc == m_node);

  DEBUG_EXPR(TESTER_COMP, LowPrio, proc);
  DEBUG_EXPR(TESTER_COMP, LowPrio, m_status);
  DEBUG_EXPR(TESTER_COMP, LowPrio, address);
  DEBUG_EXPR(TESTER_COMP, LowPrio, data);

  if (m_status == BarrierGeneratorStatus_Test_Pending) {
    uint8 dat = data.readByte();
    uint8 lock = dat >> 7;
    if (lock == 1) {
      // Locked - keep spinning
      m_status = BarrierGeneratorStatus_Test_Waiting;
      m_last_transition = g_eventQueue_ptr->getTime();
      g_eventQueue_ptr->scheduleEvent(this, waitTime());
    } else {
      // Unlocked - try the swap
      m_driver.recordTestLatency(g_eventQueue_ptr->getTime() - m_last_transition);
      m_status = BarrierGeneratorStatus_Before_Swap;
      m_last_transition = g_eventQueue_ptr->getTime();
      g_eventQueue_ptr->scheduleEvent(this, waitTime());
    }
  } else if (m_status == BarrierGeneratorStatus_Swap_Pending) {
    m_driver.recordSwapLatency(g_eventQueue_ptr->getTime() - m_last_transition);
    uint8 dat = data.readByte();
    uint8 lock = dat >> 7;
    if (lock == 1) {
      // We failed to aquire the lock
      m_status = BarrierGeneratorStatus_Test_Waiting;
      m_last_transition = g_eventQueue_ptr->getTime();
      g_eventQueue_ptr->scheduleEvent(this, waitTime());
    } else {
      // We acquired the lock
      dat = dat | 0x80;
      data.writeByte(dat);
      m_status = BarrierGeneratorStatus_StoreBarrierCounter_Waiting;
      m_last_transition = g_eventQueue_ptr->getTime();
      DEBUG_MSG(TESTER_COMP, HighPrio, "Acquired");
      DEBUG_EXPR(TESTER_COMP, HighPrio, proc);
      DEBUG_EXPR(TESTER_COMP, HighPrio, g_eventQueue_ptr->getTime());
      // g_eventQueue_ptr->scheduleEvent(this, holdTime());

      g_eventQueue_ptr->scheduleEvent(this, 1);

      // initiateLoadCtr();
    }
  } else if (m_status == BarrierGeneratorStatus_StoreBarrierCounter_Pending) {

    // if value == p, reset counter and set local sense flag
    uint8 ctr = data.readByte();
    //uint8 sense = ctr >> 4;
    ctr = ctr & 0x0F;

    ctr++;
    data.writeByte( ctr | 0x80);  // store counter and lock

    //cout << m_node << " incremented Barrier_ctr to " << (int)ctr << ", " << data << "\n";

    if (ctr == (uint8) 16) {

       data.writeByte( 0x0 );
       m_status = BarrierGeneratorStatus_StoreFlag_Waiting;
       m_barrier_done = true;

       g_eventQueue_ptr->scheduleEvent(this, 1);
    }
    else {

       m_status = BarrierGeneratorStatus_Release_Waiting;
       g_eventQueue_ptr->scheduleEvent(this, 1);
    }
  } else if (m_status == BarrierGeneratorStatus_StoreFlag_Pending) {

     // write flag
     if (m_local_sense) {
         data.writeByte( 0x01 );
     }
     else {
         data.writeByte( 0x00 );
     }

     m_status = BarrierGeneratorStatus_Release_Waiting;
     g_eventQueue_ptr->scheduleEvent(this, 1);

  } else if (m_status == BarrierGeneratorStatus_Release_Pending) {
    m_driver.recordReleaseLatency(g_eventQueue_ptr->getTime() - m_last_transition);
    // We're releasing the lock
    uint8 dat = data.readByte();
    dat = dat & 0x7F;
    data.writeByte(dat);

    if (m_barrier_done) {
      m_counter++;
      proc_counter++;
      if (m_counter < g_tester_length) {
        m_status = BarrierGeneratorStatus_Thinking;
        m_last_transition = g_eventQueue_ptr->getTime();
        g_eventQueue_ptr->scheduleEvent(this, thinkTime());
      } else {

        m_driver.reportDone(proc_counter, m_node);
        m_last_transition = g_eventQueue_ptr->getTime();
      }
    }
    else {
      m_status = BarrierGeneratorStatus_SpinFlag_Ready;
      m_last_transition = g_eventQueue_ptr->getTime();
      g_eventQueue_ptr->scheduleEvent(this, waitTime());
    }
  } else if (m_status == BarrierGeneratorStatus_SpinFlag_Pending) {

    uint8 sense = data.readByte();


    if (sense != m_local_sense) {
      m_status = BarrierGeneratorStatus_SpinFlag_Ready;
      m_last_transition = g_eventQueue_ptr->getTime();
      g_eventQueue_ptr->scheduleEvent(this, waitTime());
    }
    else {
      m_counter++;
      proc_counter++;
      if (m_counter < g_tester_length) {
        m_status = BarrierGeneratorStatus_Thinking;
        m_last_transition = g_eventQueue_ptr->getTime();
        g_eventQueue_ptr->scheduleEvent(this, thinkTime());
      } else {
        m_driver.reportDone(proc_counter, m_node);
        m_status = BarrierGeneratorStatus_Done;
        m_last_transition = g_eventQueue_ptr->getTime();
      }
    }

  } else {
    WARN_EXPR(m_status);
    ERROR_MSG("Invalid status");
  }
}

int BarrierGenerator::thinkTime()
{
  int ret;
  float ratio = g_think_fudge_factor;

  // return 400;

  if (ratio == 0) {
    return g_think_time;
  }

  int r = random();
  int x = (int) ( (float)g_think_time*ratio*2.0);
  int mod = r % x;


  int rand = ( mod+1 -  ((float)g_think_time*ratio) );

  ret = (g_think_time + rand);

  m_total_think += ret;
  m_think_periods++;

  return ret;
}

int BarrierGenerator::waitTime() const
{
  return g_wait_time;
}


void BarrierGenerator::initiateTest()
{
  DEBUG_MSG(TESTER_COMP, MedPrio, "initiating Test");
  sequencer()->makeRequest(CacheMsg(Address(0x40), CacheRequestType_LD, Address(1), AccessModeType_UserMode, 1, PrefetchBit_No, 0, false));
}

void BarrierGenerator::initiateSwap()
{
  DEBUG_MSG(TESTER_COMP, MedPrio, "initiating Swap");
  sequencer()->makeRequest(CacheMsg(Address(0x40), CacheRequestType_ATOMIC, Address(2), AccessModeType_UserMode, 1, PrefetchBit_No, 0, false));
}

void BarrierGenerator::initiateRelease()
{
  DEBUG_MSG(TESTER_COMP, MedPrio, "initiating Release");
  sequencer()->makeRequest(CacheMsg(Address(0x40), CacheRequestType_ST, Address(3), AccessModeType_UserMode, 1, PrefetchBit_No, 0, false));
}

void BarrierGenerator::initiateLoadCtr()
{
  DEBUG_MSG(TESTER_COMP, MedPrio, "initiating load of barrier counter");
  sequencer()->makeRequest(CacheMsg(Address(0x40), CacheRequestType_LD, Address(3), AccessModeType_UserMode, 1, PrefetchBit_No, 0, false));
}

void BarrierGenerator::initiateStoreCtr()
{
  DEBUG_MSG(TESTER_COMP, MedPrio, "initiating load of barrier counter");
  sequencer()->makeRequest(CacheMsg(Address(0x40), CacheRequestType_ST, Address(3), AccessModeType_UserMode, 1, PrefetchBit_No, 0, false));
}

void BarrierGenerator::initiateStoreFlag()
{
  DEBUG_MSG(TESTER_COMP, MedPrio, "initiating load of barrier counter");
  sequencer()->makeRequest(CacheMsg(Address(0x00), CacheRequestType_ST, Address(3), AccessModeType_UserMode, 1, PrefetchBit_No, 0, false));
}

void BarrierGenerator::initiateLoadFlag()
{
  DEBUG_MSG(TESTER_COMP, MedPrio, "initiating load of barrier counter");
  sequencer()->makeRequest(CacheMsg(Address(0x00), CacheRequestType_LD, Address(3), AccessModeType_UserMode, 1, PrefetchBit_No, 0, false));
}


Sequencer* BarrierGenerator::sequencer() const
{
  return g_system_ptr->getChip(m_node/RubyConfig::numberOfProcsPerChip())->getSequencer(m_node%RubyConfig::numberOfProcsPerChip());
}

void BarrierGenerator::print(ostream& out) const
{
}

