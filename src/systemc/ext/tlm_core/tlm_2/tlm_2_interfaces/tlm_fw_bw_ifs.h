/*****************************************************************************

  Licensed to Accellera Systems Initiative Inc. (Accellera) under one or
  more contributor license agreements.  See the NOTICE file distributed
  with this work for additional information regarding copyright ownership.
  Accellera licenses this file to you under the Apache License, Version 2.0
  (the "License"); you may not use this file except in compliance with the
  License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
  implied.  See the License for the specific language governing
  permissions and limitations under the License.

 *****************************************************************************/

#ifndef __TLM_FW_BW_IFS_H__
#define __TLM_FW_BW_IFS_H__

#include <systemc>
#include "tlm_core/tlm_2/tlm_generic_payload/tlm_generic_payload.h"
#include "tlm_core/tlm_2/tlm_2_interfaces/tlm_dmi.h"

namespace tlm {

enum tlm_sync_enum { TLM_ACCEPTED, TLM_UPDATED, TLM_COMPLETED };

////////////////////////////////////////////////////////////////////////////
// Basic interfaces
////////////////////////////////////////////////////////////////////////////
template <typename TRANS = tlm_generic_payload,
          typename PHASE = tlm_phase>
class tlm_fw_nonblocking_transport_if : public virtual sc_core::sc_interface {
public:
  virtual tlm_sync_enum nb_transport_fw(TRANS& trans,
                                        PHASE& phase,
                                        sc_core::sc_time& t) = 0;
};

template <typename TRANS = tlm_generic_payload,
          typename PHASE = tlm_phase>
class tlm_bw_nonblocking_transport_if : public virtual sc_core::sc_interface {
public:
  virtual tlm_sync_enum nb_transport_bw(TRANS& trans,
                                        PHASE& phase,
                                        sc_core::sc_time& t) = 0;
};

template <typename TRANS = tlm_generic_payload>
class tlm_blocking_transport_if : public virtual sc_core::sc_interface {
public:
  virtual void b_transport(TRANS& trans,
                           sc_core::sc_time& t) = 0;
};

//////////////////////////////////////////////////////////////////////////
// DMI interfaces for getting and invalidating DMI pointers:
//////////////////////////////////////////////////////////////////////////

// The semantics of the forward interface are as follows:
//
// - An initiator that wants to get direct access to a target's memory region
//   can call the get_direct_mem_ptr method with the 'trans' parameter set to
//   the address that it wants to gain access to. It sets the trans.m_command
//   to specify if the initiator intended use (read or write)
//   to the target's DMI region. The initiator is responsible for calling the
//   method with a freshly initialized tlm_dmi object either by using a newly
//   constructed object, or by calling an existing object's init() method.
// - Although a reference to a complete 'TRANS' type is passed to the get_
//   direct_mem_ptr call, only the address command, and extension fields are of
//   interest in most cases.
// - Read and write ranges are not necessarily identical. If they are, a target
//   can specify that the range is valid for all accesses with the tlm_data
//   m_type attribute in the.
// - The interconnect, if any, needs to decode the address and forward the
//   call to the corresponding target. It needs to handle the address exactly
//   as the target would expect on a transaction call, e.g. mask the address
//   according to the target's address width.
// - If the target supports DMI access for the given address, it sets the
//   data fields in the DMI struct and returns true.
// - If a target does not support DMI access it needs to return false.
//   The target can either set the correct address range in the DMI struct
//   to indicate the memory region where DMI is disallowed, or it can specify
//   the complete address range if it doesn't know it's memory range. In this
//   case the interconnect is responsible for clipping the address range to
//   the correct range that the target serves.
// - The interconnect must always translate the addresses to the initiator's
//   address space. This must be the inverse operation of what the
//   interconnect needed to do when forwarding the call. In case the
//   component wants to change any member of the tlm_dmi object, e.g. for
//   its own latency to the target's latency, it must only do so *after* the
//   target has been called. The target is always allowed to overwrite all
//   values in the tlm_dmi object.
// - In case the slave returned with an invalid region the bus/interconnect
//   must fill in the complete address region for the particular slave in the
//   DMI data structure.
//
// DMI hint optimization:
//
// Initiators may use the DMI hint in the tlm_generic_payload to avoid
// unnecessary DMI attempts. The recommended sequence of interface
// method calls would be:
//
// - The initiator first tries to check if it has a valid DMI region for the
//   address that it wants to access next.
// - If not, it performs a normal transaction.
// - If the DMI hint in this transaction is true, the initiator can try and
//   get the DMI region.
//
// Note that the DMI hint optimization is completely optional and every
// initiator model is free to ignore the DMI hint. However, a target is
// required to set the DMI hint to true if a DMI request on the given address
// with the given transaction type (read or write) would have succeeded.

template <typename TRANS = tlm_generic_payload>
class tlm_fw_direct_mem_if : public virtual sc_core::sc_interface
{
public:
  virtual bool get_direct_mem_ptr(TRANS& trans,
                                  tlm_dmi&  dmi_data) = 0;
};

// The semantics of the backwards call is as follows:
//
// - An interconnect component or a target is required to invalidate all
//   affected DMI regions whenever any change in the regions take place.
//   The exact rule is that a component must invalidate all those DMI regions
//   that it already reported, if it would answer the same DMI request
//   with any member of the tlm_dmi data structure set differently.
// - An interconnect component must forward the invalidate_direct_mem_ptr call
//   to all initiators that could potentially have a DMI pointer to the region
//   specified in the method arguments. A safe implementation is to call
//   every attached initiator.
// - An interconnect component must transform the address region of an
//   incoming invalidate_direct_mem_ptr to the corresponding address space
//   for the initiators. Basically, this is the same address transformation
//   that the interconnect does on the DMI ranges on the forward direction.
// - Each initiator must check if it has a pointer to the given region and
//   throw this away. It is recommended that the initiator throws away all DMI
//   regions that have any overlap with the given regions, but this is not a
//   hard requirement.
//
// - A full DMI pointer invalidation, e.g. for a bus remap can be signaled
//   by setting the range: 0x0 - 0xffffffffffffffffull = (sc_dt::uint64)-1
// - An initiator must throw away all DMI pointers in this case.
//
// - Under no circumstances a model is allowed to call the get_direct_mem_ptr
//   from within the invalidate_direct_mem_ptr method, directly or indirectly.
//
class tlm_bw_direct_mem_if : public virtual sc_core::sc_interface
{
public:
  virtual void invalidate_direct_mem_ptr(sc_dt::uint64 start_range,
                                         sc_dt::uint64 end_range) = 0;
};

/////////////////////////////////////////////////////////////////////
// debug interface for memory access
/////////////////////////////////////////////////////////////////////
//
// This interface can be used to gain access to a targets memory or registers
// in a non-intrusive manner. No side effects, waits or event notifications
// must happen in the course of the method.
//
// Semantics:
// - The initiator calls the transport_dbg method with transaction 'trans' as
//   argument. The commonly used parts of trans for debug are:
//   . address: The start address that it wants to peek or poke.
//   . length:  The number of bytes that it requests to read or write.
//   . command: Indicates a read or write access.
//   . data:    A pointer to the initiator-allocated data buffer, which must
//              be at least num_bytes large. The data is always organized in
//              the endianness of the machine.
//   . extensions: Any extension that could affect the transaction.
// - The interconnect, if any, will decode the address and forward the call to
//   the appropriate target.
// - The target must return the number of successfully transmitted bytes, where
//   this number must be <= num_bytes. Thus, a target can safely return 0 if it
//   does not support debug transactions.
//
template <typename TRANS = tlm_generic_payload>
class tlm_transport_dbg_if : public virtual sc_core::sc_interface
{
public:
  // The return value of defines the number of bytes successfully
  // transferred.
  virtual unsigned int transport_dbg(TRANS& trans) = 0;
};

////////////////////////////////////////////////////////////////////////////
// Combined interfaces
////////////////////////////////////////////////////////////////////////////

struct tlm_base_protocol_types
{
  typedef tlm_generic_payload tlm_payload_type;
  typedef tlm_phase tlm_phase_type;
};

// The forward interface:
template <typename TYPES = tlm_base_protocol_types>
class tlm_fw_transport_if
  : public virtual tlm_fw_nonblocking_transport_if<typename TYPES::tlm_payload_type,
                                                   typename TYPES::tlm_phase_type>
  , public virtual tlm_blocking_transport_if<typename TYPES::tlm_payload_type>
  , public virtual tlm_fw_direct_mem_if<typename TYPES::tlm_payload_type>
  , public virtual tlm_transport_dbg_if<typename TYPES::tlm_payload_type>
{};

// The backward interface:
template <typename TYPES = tlm_base_protocol_types>
class tlm_bw_transport_if
  : public virtual tlm_bw_nonblocking_transport_if<typename TYPES::tlm_payload_type,
                                                   typename TYPES::tlm_phase_type>
  , public virtual tlm_bw_direct_mem_if
{};

} // namespace tlm

#endif /* __TLM_FW_BW_IFS_H__ */
