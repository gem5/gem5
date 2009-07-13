
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

#include "mem/ruby/common/Global.hh"
#include "mem/ruby/system/Sequencer.hh"
#include "mem/ruby/system/System.hh"
#include "mem/protocol/Protocol.hh"
#include "mem/ruby/profiler/Profiler.hh"
#include "mem/ruby/system/CacheMemory.hh"
#include "mem/protocol/CacheMsg.hh"
#include "mem/ruby/recorder/Tracer.hh"
#include "mem/ruby/common/SubBlock.hh"
#include "mem/protocol/Protocol.hh"
#include "mem/gems_common/Map.hh"
#include "mem/ruby/buffers/MessageBuffer.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"

//Sequencer::Sequencer(int core_id, MessageBuffer* mandatory_q)

Sequencer::Sequencer(const string & name)
  :RubyPort(name)
{
}

void Sequencer::init(const vector<string> & argv)
{
  m_deadlock_check_scheduled = false;
  m_outstanding_count = 0;

  m_max_outstanding_requests = 0;
  m_deadlock_threshold = 0;
  m_version = -1;
  m_instCache_ptr = NULL;
  m_dataCache_ptr = NULL;
  m_controller = NULL;
  for (size_t i=0; i<argv.size(); i+=2) {
    if ( argv[i] == "controller") {
      m_controller = RubySystem::getController(argv[i+1]); // args[i] = "L1Cache"
      m_mandatory_q_ptr = m_controller->getMandatoryQueue();
    } else if ( argv[i] == "icache")
      m_instCache_ptr = RubySystem::getCache(argv[i+1]);
    else if ( argv[i] == "dcache")
      m_dataCache_ptr = RubySystem::getCache(argv[i+1]);
    else if ( argv[i] == "version")
      m_version = atoi(argv[i+1].c_str());
    else if ( argv[i] == "max_outstanding_requests")
      m_max_outstanding_requests = atoi(argv[i+1].c_str());
    else if ( argv[i] == "deadlock_threshold")
      m_deadlock_threshold = atoi(argv[i+1].c_str());
    else {
      cerr << "WARNING: Sequencer: Unkown configuration parameter: " << argv[i] << endl;
      assert(false);
    }
  }
  assert(m_max_outstanding_requests > 0);
  assert(m_deadlock_threshold > 0);
  assert(m_version > -1);
  assert(m_instCache_ptr != NULL);
  assert(m_dataCache_ptr != NULL);
  assert(m_controller != NULL);
}

Sequencer::~Sequencer() {

}

void Sequencer::wakeup() {
  // Check for deadlock of any of the requests
  Time current_time = g_eventQueue_ptr->getTime();

  // Check across all outstanding requests
  int total_outstanding = 0;

  Vector<Address> keys = m_readRequestTable.keys();
  for (int i=0; i<keys.size(); i++) {
    SequencerRequest* request = m_readRequestTable.lookup(keys[i]);
    if (current_time - request->issue_time >= m_deadlock_threshold) {
      WARN_MSG("Possible Deadlock detected");
      WARN_EXPR(request);
      WARN_EXPR(m_version);
      WARN_EXPR(keys.size());
      WARN_EXPR(current_time);
      WARN_EXPR(request->issue_time);
      WARN_EXPR(current_time - request->issue_time);
      ERROR_MSG("Aborting");
    }
  }

  keys = m_writeRequestTable.keys();
  for (int i=0; i<keys.size(); i++) {
    SequencerRequest* request = m_writeRequestTable.lookup(keys[i]);
    if (current_time - request->issue_time >= m_deadlock_threshold) {
      WARN_MSG("Possible Deadlock detected");
      WARN_EXPR(request);
      WARN_EXPR(m_version);
      WARN_EXPR(current_time);
      WARN_EXPR(request->issue_time);
      WARN_EXPR(current_time - request->issue_time);
      WARN_EXPR(keys.size());
      ERROR_MSG("Aborting");
    }
  }
  total_outstanding += m_writeRequestTable.size() + m_readRequestTable.size();

  assert(m_outstanding_count == total_outstanding);

  if (m_outstanding_count > 0) { // If there are still outstanding requests, keep checking
    g_eventQueue_ptr->scheduleEvent(this, m_deadlock_threshold);
  } else {
    m_deadlock_check_scheduled = false;
  }
}

void Sequencer::printProgress(ostream& out) const{
  /*
  int total_demand = 0;
  out << "Sequencer Stats Version " << m_version << endl;
  out << "Current time = " << g_eventQueue_ptr->getTime() << endl;
  out << "---------------" << endl;
  out << "outstanding requests" << endl;

  Vector<Address> rkeys = m_readRequestTable.keys();
  int read_size = rkeys.size();
  out << "proc " << m_version << " Read Requests = " << read_size << endl;
  // print the request table
  for(int i=0; i < read_size; ++i){
    SequencerRequest * request = m_readRequestTable.lookup(rkeys[i]);
    out << "\tRequest[ " << i << " ] = " << request->type << " Address " << rkeys[i]  << " Posted " << request->issue_time << " PF " << PrefetchBit_No << endl;
    total_demand++;
  }

  Vector<Address> wkeys = m_writeRequestTable.keys();
  int write_size = wkeys.size();
  out << "proc " << m_version << " Write Requests = " << write_size << endl;
  // print the request table
  for(int i=0; i < write_size; ++i){
      CacheMsg & request = m_writeRequestTable.lookup(wkeys[i]);
      out << "\tRequest[ " << i << " ] = " << request.getType() << " Address " << wkeys[i]  << " Posted " << request.getTime() << " PF " << request.getPrefetch() << endl;
      if( request.getPrefetch() == PrefetchBit_No ){
        total_demand++;
      }
  }

  out << endl;

  out << "Total Number Outstanding: " << m_outstanding_count << endl;
  out << "Total Number Demand     : " << total_demand << endl;
  out << "Total Number Prefetches : " << m_outstanding_count - total_demand << endl;
  out << endl;
  out << endl;
  */
}

void Sequencer::printConfig(ostream& out) const {
  out << "Seqeuncer config: " << m_name << endl;
  out << "  controller: " << m_controller->getName() << endl;
  out << "  version: " << m_version << endl;
  out << "  max_outstanding_requests: " << m_max_outstanding_requests << endl;
  out << "  deadlock_threshold: " << m_deadlock_threshold << endl;
}

// Insert the request on the correct request table.  Return true if
// the entry was already present.
bool Sequencer::insertRequest(SequencerRequest* request) {
  int total_outstanding = m_writeRequestTable.size() + m_readRequestTable.size();

  assert(m_outstanding_count == total_outstanding);

  // See if we should schedule a deadlock check
  if (m_deadlock_check_scheduled == false) {
    g_eventQueue_ptr->scheduleEvent(this, m_deadlock_threshold);
    m_deadlock_check_scheduled = true;
  }

  Address line_addr(request->ruby_request.paddr);
  line_addr.makeLineAddress();
  if ((request->ruby_request.type == RubyRequestType_ST) ||
      (request->ruby_request.type == RubyRequestType_RMW_Read) ||  
      (request->ruby_request.type == RubyRequestType_RMW_Write)) {
    if (m_writeRequestTable.exist(line_addr)) {
      m_writeRequestTable.lookup(line_addr) = request;
      //      return true;
      assert(0); // drh5: isn't this an error?  do you lose the initial request?
    }
    m_writeRequestTable.allocate(line_addr);
    m_writeRequestTable.lookup(line_addr) = request;
    m_outstanding_count++;
  } else {
    if (m_readRequestTable.exist(line_addr)) {
      m_readRequestTable.lookup(line_addr) = request;
      //      return true;
      assert(0); // drh5: isn't this an error?  do you lose the initial request?
    }
    m_readRequestTable.allocate(line_addr);
    m_readRequestTable.lookup(line_addr) = request;
    m_outstanding_count++;
  }

  g_system_ptr->getProfiler()->sequencerRequests(m_outstanding_count);

  total_outstanding = m_writeRequestTable.size() + m_readRequestTable.size();
  assert(m_outstanding_count == total_outstanding);

  return false;
}

void Sequencer::removeRequest(SequencerRequest* srequest) {

  assert(m_outstanding_count == m_writeRequestTable.size() + m_readRequestTable.size());

  const RubyRequest & ruby_request = srequest->ruby_request;
  Address line_addr(ruby_request.paddr);
  line_addr.makeLineAddress();
  if ((ruby_request.type == RubyRequestType_ST) ||
      (ruby_request.type == RubyRequestType_RMW_Read) ||
      (ruby_request.type == RubyRequestType_RMW_Write)) {
    m_writeRequestTable.deallocate(line_addr);
  } else {
    m_readRequestTable.deallocate(line_addr);
  }
  m_outstanding_count--;

  assert(m_outstanding_count == m_writeRequestTable.size() + m_readRequestTable.size());
}

void Sequencer::writeCallback(const Address& address, DataBlock& data) {

  assert(address == line_address(address));
  assert(m_writeRequestTable.exist(line_address(address)));

  SequencerRequest* request = m_writeRequestTable.lookup(address);
  removeRequest(request);

  assert((request->ruby_request.type == RubyRequestType_ST) ||
         (request->ruby_request.type == RubyRequestType_RMW_Read) ||
         (request->ruby_request.type == RubyRequestType_RMW_Write));
  // POLINA: the assumption is that atomics are only on data cache and not instruction cache
  if (request->ruby_request.type == RubyRequestType_RMW_Read) {
    m_dataCache_ptr->setLocked(address, m_version);
  }

  hitCallback(request, data);
}

void Sequencer::readCallback(const Address& address, DataBlock& data) {

  assert(address == line_address(address));
  assert(m_readRequestTable.exist(line_address(address)));

  SequencerRequest* request = m_readRequestTable.lookup(address);
  removeRequest(request);

  assert((request->ruby_request.type == RubyRequestType_LD) ||
         (request->ruby_request.type == RubyRequestType_IFETCH));

  hitCallback(request, data);
}

void Sequencer::hitCallback(SequencerRequest* srequest, DataBlock& data) {
  const RubyRequest & ruby_request = srequest->ruby_request;
  Address request_address(ruby_request.paddr);
  Address request_line_address(ruby_request.paddr);
  request_line_address.makeLineAddress();
  RubyRequestType type = ruby_request.type;
  Time issued_time = srequest->issue_time;

  // Set this cache entry to the most recently used
  if (type == RubyRequestType_IFETCH) {
    if (m_instCache_ptr->isTagPresent(request_line_address) )
      m_instCache_ptr->setMRU(request_line_address);
  } else {
    if (m_dataCache_ptr->isTagPresent(request_line_address) )
      m_dataCache_ptr->setMRU(request_line_address);
  }

  assert(g_eventQueue_ptr->getTime() >= issued_time);
  Time miss_latency = g_eventQueue_ptr->getTime() - issued_time;

  // Profile the miss latency for all non-zero demand misses
  if (miss_latency != 0) {
    g_system_ptr->getProfiler()->missLatency(miss_latency, type);

    if (Debug::getProtocolTrace()) {
      g_system_ptr->getProfiler()->profileTransition("Seq", m_version, Address(ruby_request.paddr),
                                                     "", "Done", "", int_to_string(miss_latency)+" cycles");
    }
  }
  /*
  if (request.getPrefetch() == PrefetchBit_Yes) {
    return; // Ignore the prefetch
  }
  */

  // update the data
  if (ruby_request.data != NULL) {
    if ((type == RubyRequestType_LD) ||
        (type == RubyRequestType_IFETCH)) {
      memcpy(ruby_request.data, data.getData(request_address.getOffset(), ruby_request.len), ruby_request.len);
    } else {
      data.setData(ruby_request.data, request_address.getOffset(), ruby_request.len);
    }
  }

  m_hit_callback(srequest->id);
  delete srequest;
}

// Returns true if the sequencer already has a load or store outstanding
bool Sequencer::isReady(const RubyRequest& request) const {
  // POLINA: check if we are currently flushing the write buffer, if so Ruby is returned as not ready
  // to simulate stalling of the front-end
  // Do we stall all the sequencers? If it is atomic instruction - yes!
  if (m_outstanding_count >= m_max_outstanding_requests) {
    return false;
  }

  if( m_writeRequestTable.exist(line_address(Address(request.paddr))) ||
      m_readRequestTable.exist(line_address(Address(request.paddr))) ){
    //cout << "OUTSTANDING REQUEST EXISTS " << p << " VER " << m_version << endl;
    //printProgress(cout);
    return false;
  }

  return true;
}

bool Sequencer::empty() const {
  return (m_writeRequestTable.size() == 0) && (m_readRequestTable.size() == 0);
}


// -2 means that the LLSC failed
int64_t Sequencer::makeRequest(const RubyRequest & request)
{
  assert(Address(request.paddr).getOffset() + request.len <= RubySystem::getBlockSizeBytes());
  if (isReady(request)) {
    int64_t id = makeUniqueRequestID();
    SequencerRequest *srequest = new SequencerRequest(request, id, g_eventQueue_ptr->getTime());
    bool found = insertRequest(srequest);
    if (!found)
      if (request.type == RubyRequestType_RMW_Write) {
        if (!m_dataCache_ptr->isLocked(line_address(Address(request.paddr)), m_version)) {
          return -2;
        }
        else {
          m_dataCache_ptr->clearLocked(line_address(Address(request.paddr)));
        }
      }
      issueRequest(request);

    // TODO: issue hardware prefetches here
    return id;
  }
  else {
    return -1;
  }
}

void Sequencer::issueRequest(const RubyRequest& request) {

  // TODO: get rid of CacheMsg, CacheRequestType, and AccessModeTYpe, & have SLICC use RubyRequest and subtypes natively
  CacheRequestType ctype;
  switch(request.type) {
  case RubyRequestType_IFETCH:
    ctype = CacheRequestType_IFETCH;
    break;
  case RubyRequestType_LD:
    ctype = CacheRequestType_LD;
    break;
  case RubyRequestType_ST:
    ctype = CacheRequestType_ST;
    break;
  case RubyRequestType_RMW_Read:
    ctype = CacheRequestType_ATOMIC;
    break;
  case RubyRequestType_RMW_Write:
    ctype = CacheRequestType_ATOMIC;
    break;
  default:
    assert(0);
  }
  AccessModeType amtype;
  switch(request.access_mode){
  case RubyAccessMode_User:
    amtype = AccessModeType_UserMode;
    break;
  case RubyAccessMode_Supervisor:
    amtype = AccessModeType_SupervisorMode;
    break;
  case RubyAccessMode_Device:
    amtype = AccessModeType_UserMode;
    break;
  default:
    assert(0);
  }
  Address line_addr(request.paddr);
  line_addr.makeLineAddress();
  CacheMsg msg(line_addr, Address(request.paddr), ctype, Address(request.pc), amtype, request.len, PrefetchBit_No);

  if (Debug::getProtocolTrace()) {
    g_system_ptr->getProfiler()->profileTransition("Seq", m_version, Address(request.paddr),
                                                   "", "Begin", "", RubyRequestType_to_string(request.type));
  }

  if (g_system_ptr->getTracer()->traceEnabled()) {
    g_system_ptr->getTracer()->traceRequest(m_name, line_addr, Address(request.pc),
                                            request.type, g_eventQueue_ptr->getTime());
  }

  Time latency = 0;  // initialzed to an null value

  if (request.type == RubyRequestType_IFETCH)
    latency = m_instCache_ptr->getLatency();
  else
    latency = m_dataCache_ptr->getLatency();

  // Send the message to the cache controller
  assert(latency > 0);


  m_mandatory_q_ptr->enqueue(msg, latency);
}
/*
bool Sequencer::tryCacheAccess(const Address& addr, CacheRequestType type,
                               AccessModeType access_mode,
                               int size, DataBlock*& data_ptr) {
  if (type == CacheRequestType_IFETCH) {
    return m_instCache_ptr->tryCacheAccess(line_address(addr), type, data_ptr);
  } else {
    return m_dataCache_ptr->tryCacheAccess(line_address(addr), type, data_ptr);
  }
}
*/

void Sequencer::print(ostream& out) const {
  out << "[Sequencer: " << m_version
      << ", outstanding requests: " << m_outstanding_count;

  out << ", read request table: " << m_readRequestTable
      << ", write request table: " << m_writeRequestTable;
  out << "]";
}

// this can be called from setState whenever coherence permissions are upgraded
// when invoked, coherence violations will be checked for the given block
void Sequencer::checkCoherence(const Address& addr) {
#ifdef CHECK_COHERENCE
  g_system_ptr->checkGlobalCoherenceInvariant(addr);
#endif
}

/*
bool Sequencer::getRubyMemoryValue(const Address& addr, char* value,
                                   unsigned int size_in_bytes )
{
    bool found = false;
    const Address lineAddr = line_address(addr);
    DataBlock data;
    PhysAddress paddr(addr);
    DataBlock* dataPtr = &data;

    MachineID l2_mach = map_L2ChipId_to_L2Cache(addr, m_chip_ptr->getID() );
    int l2_ver = l2_mach.num%RubyConfig::numberOfL2CachePerChip();

    if (Protocol::m_TwoLevelCache) {
      if(Protocol::m_CMP){
        assert(n->m_L2Cache_L2cacheMemory_vec[l2_ver] != NULL);
      }
      else{
        assert(n->m_L1Cache_cacheMemory_vec[m_version] != NULL);
      }
    }

    if (n->m_L1Cache_L1IcacheMemory_vec[m_version]->tryCacheAccess(lineAddr, CacheRequestType_IFETCH, dataPtr)){
      n->m_L1Cache_L1IcacheMemory_vec[m_version]->getMemoryValue(addr, value, size_in_bytes);
      found = true;
    } else if (n->m_L1Cache_L1DcacheMemory_vec[m_version]->tryCacheAccess(lineAddr, CacheRequestType_LD, dataPtr)){
      n->m_L1Cache_L1DcacheMemory_vec[m_version]->getMemoryValue(addr, value, size_in_bytes);
      found = true;
    } else if (Protocol::m_CMP && n->m_L2Cache_L2cacheMemory_vec[l2_ver]->tryCacheAccess(lineAddr, CacheRequestType_LD, dataPtr)){
      n->m_L2Cache_L2cacheMemory_vec[l2_ver]->getMemoryValue(addr, value, size_in_bytes);
      found = true;
    // } else if (n->TBE_TABLE_MEMBER_VARIABLE->isPresent(lineAddr)){
//       ASSERT(n->TBE_TABLE_MEMBER_VARIABLE->isPresent(lineAddr));
//       L1Cache_TBE tbeEntry = n->TBE_TABLE_MEMBER_VARIABLE->lookup(lineAddr);

//       int offset = addr.getOffset();
//       for(int i=0; i<size_in_bytes; ++i){
//         value[i] = tbeEntry.getDataBlk().getByte(offset + i);
//       }

//       found = true;
    } else {
      // Address not found
      //cout << "  " << m_chip_ptr->getID() << " NOT IN CACHE, Value at Directory is: " << (int) value[0] << endl;
      n = dynamic_cast<Chip*>(g_system_ptr->getChip(map_Address_to_DirectoryNode(addr)/RubyConfig::numberOfDirectoryPerChip()));
      int dir_version = map_Address_to_DirectoryNode(addr)%RubyConfig::numberOfDirectoryPerChip();
      for(unsigned int i=0; i<size_in_bytes; ++i){
        int offset = addr.getOffset();
        value[i] = n->m_Directory_directory_vec[dir_version]->lookup(lineAddr).m_DataBlk.getByte(offset + i);
      }
      // Address not found
      //WARN_MSG("Couldn't find address");
      //WARN_EXPR(addr);
      found = false;
    }
    return true;
}

bool Sequencer::setRubyMemoryValue(const Address& addr, char *value,
                                   unsigned int size_in_bytes) {
  char test_buffer[64];

  // idea here is that coherent cache should find the
  // latest data, the update it
  bool found = false;
  const Address lineAddr = line_address(addr);
  PhysAddress paddr(addr);
  DataBlock data;
  DataBlock* dataPtr = &data;
  Chip* n = dynamic_cast<Chip*>(m_chip_ptr);

  MachineID l2_mach = map_L2ChipId_to_L2Cache(addr, m_chip_ptr->getID() );
  int l2_ver = l2_mach.num%RubyConfig::numberOfL2CachePerChip();

  assert(n->m_L1Cache_L1IcacheMemory_vec[m_version] != NULL);
  assert(n->m_L1Cache_L1DcacheMemory_vec[m_version] != NULL);
  if (Protocol::m_TwoLevelCache) {
    if(Protocol::m_CMP){
      assert(n->m_L2Cache_L2cacheMemory_vec[l2_ver] != NULL);
    }
    else{
      assert(n->m_L1Cache_cacheMemory_vec[m_version] != NULL);
    }
  }

  if (n->m_L1Cache_L1IcacheMemory_vec[m_version]->tryCacheAccess(lineAddr, CacheRequestType_IFETCH, dataPtr)){
    n->m_L1Cache_L1IcacheMemory_vec[m_version]->setMemoryValue(addr, value, size_in_bytes);
    found = true;
  } else if (n->m_L1Cache_L1DcacheMemory_vec[m_version]->tryCacheAccess(lineAddr, CacheRequestType_LD, dataPtr)){
    n->m_L1Cache_L1DcacheMemory_vec[m_version]->setMemoryValue(addr, value, size_in_bytes);
    found = true;
  } else if (Protocol::m_CMP && n->m_L2Cache_L2cacheMemory_vec[l2_ver]->tryCacheAccess(lineAddr, CacheRequestType_LD, dataPtr)){
    n->m_L2Cache_L2cacheMemory_vec[l2_ver]->setMemoryValue(addr, value, size_in_bytes);
    found = true;
  } else {
    // Address not found
    n = dynamic_cast<Chip*>(g_system_ptr->getChip(map_Address_to_DirectoryNode(addr)/RubyConfig::numberOfDirectoryPerChip()));
    int dir_version = map_Address_to_DirectoryNode(addr)%RubyConfig::numberOfDirectoryPerChip();
    for(unsigned int i=0; i<size_in_bytes; ++i){
      int offset = addr.getOffset();
      n->m_Directory_directory_vec[dir_version]->lookup(lineAddr).m_DataBlk.setByte(offset + i, value[i]);
    }
    found = false;
  }

  if (found){
    found = getRubyMemoryValue(addr, test_buffer, size_in_bytes);
    assert(found);
    if(value[0] != test_buffer[0]){
      WARN_EXPR((int) value[0]);
      WARN_EXPR((int) test_buffer[0]);
      ERROR_MSG("setRubyMemoryValue failed to set value.");
    }
  }

  return true;
}
*/
/*

void
Sequencer::rubyMemAccess(const uint64 paddr, char* data, const int len, const AccessType type)
{
  if ( type == AccessType_Read || type == AccessType_Write ) {
    // need to break up the packet data
    uint64 guest_ptr = paddr;
    Vector<DataBlock*> datablocks;
    while (paddr + len != guest_ptr) {
      Address addr(guest_ptr);
      Address line_addr = line_address(addr);

      int bytes_copied;
      if (addr.getOffset() == 0) {
        bytes_copied = (guest_ptr + RubyConfig::dataBlockBytes() > paddr + len)?
          (paddr + len - guest_ptr):
          RubyConfig::dataBlockBytes();
      } else {
        bytes_copied = RubyConfig::dataBlockBytes() - addr.getOffset();
        if (guest_ptr + bytes_copied > paddr + len)
          bytes_copied = paddr + len - guest_ptr;
      }

      // first we need to find all data blocks that have to be updated for a write
      // and the highest block for a read
     for(int i=0;i<RubyConfig::numberOfProcessors();i++) {
        if (Protocol::m_TwoLevelCache){
          if(m_chip_ptr->m_L1Cache_L1IcacheMemory_vec[i]->isTagPresent(line_address(addr)))
            datablocks.insertAtBottom(&m_chip_ptr->m_L1Cache_L1IcacheMemory_vec[i]->lookup(line_addr).getDataBlk());
          if(m_chip_ptr->m_L1Cache_L1DcacheMemory_vec[i]->isTagPresent(line_address(addr)))
            datablocks.insertAtBottom(&m_chip_ptr->m_L1Cache_L1DcacheMemory_vec[i]->lookup(line_addr).getDataBlk());
        } else {
          if(m_chip_ptr->m_L1Cache_cacheMemory_vec[i]->isTagPresent(line_address(addr)))
            datablocks.insertAtBottom(&m_chip_ptr->m_L1Cache_cacheMemory_vec[i]->lookup(line_addr).getDataBlk());
        }
      }
      if (Protocol::m_TwoLevelCache){
        int l2_bank = map_L2ChipId_to_L2Cache(addr, 0).num; // TODO: ONLY WORKS WITH CMP!!!
        if (m_chip_ptr->m_L2Cache_L2cacheMemory_vec[l2_bank]->isTagPresent(line_address(Address(paddr)))) {
          datablocks.insertAtBottom(&m_chip_ptr->m_L2Cache_L2cacheMemory_vec[l2_bank]->lookup(addr).getDataBlk());
        }
      }
      assert(dynamic_cast<Chip*>(m_chip_ptr)->m_Directory_directory_vec.size() > map_Address_to_DirectoryNode(addr));
      DirectoryMemory* dir = dynamic_cast<Chip*>(m_chip_ptr)->m_Directory_directory_vec[map_Address_to_DirectoryNode(addr)];
      Directory_Entry& entry = dir->lookup(line_addr);
      datablocks.insertAtBottom(&entry.getDataBlk());

      if (pkt->isRead()){
        datablocks[0]->copyData(pkt_data, addr.getOffset(), bytes_copied);
      } else {// pkt->isWrite() {
        for (int i=0;i<datablocks.size();i++)
          datablocks[i]->setData(pkt_data, addr.getOffset(), bytes_copied);
      }

      guest_ptr += bytes_copied;
      pkt_data += bytes_copied;
      datablocks.clear();
    }
}

*/
