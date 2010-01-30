
#include "mem/ruby/system/DMASequencer.hh"
#include "mem/ruby/buffers/MessageBuffer.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"

/* SLICC generated types */
#include "mem/protocol/SequencerMsg.hh"
#include "mem/protocol/SequencerRequestType.hh"
#include "mem/ruby/system/System.hh"

DMASequencer::DMASequencer(const Params *p)
  : RubyPort(p)
{
}

void DMASequencer::init()
{
  m_is_busy = false;
  m_data_block_mask = ~ (~0 << RubySystem::getBlockSizeBits());
}

int64_t DMASequencer::makeRequest(const RubyRequest & request)
{
  uint64_t paddr = request.paddr;
  uint8_t* data = request.data;
  int len = request.len;
  bool write = false;
  switch(request.type) {
  case RubyRequestType_LD:
    write = false;
    break;
  case RubyRequestType_ST:
    write = true;
    break;
  case RubyRequestType_NULL:
  case RubyRequestType_IFETCH:
  case RubyRequestType_Locked_Read:
  case RubyRequestType_Locked_Write:
  case RubyRequestType_RMW_Read:
  case RubyRequestType_RMW_Write:
  case RubyRequestType_NUM:
    assert(0);
  }

  assert(!m_is_busy);  // only support one outstanding DMA request
  m_is_busy = true;

  active_request.start_paddr = paddr;
  active_request.write = write;
  active_request.data = data;
  active_request.len = len;
  active_request.bytes_completed = 0;
  active_request.bytes_issued = 0;
  active_request.id = makeUniqueRequestID();

  SequencerMsg msg;
  msg.getPhysicalAddress() = Address(paddr);
  msg.getLineAddress() = line_address(msg.getPhysicalAddress());
  msg.getType() = write ? SequencerRequestType_ST : SequencerRequestType_LD;
  int offset = paddr & m_data_block_mask;
  msg.getLen() = (offset + len) <= RubySystem::getBlockSizeBytes() ?
    len : 
    RubySystem::getBlockSizeBytes() - offset;
  if (write)
    msg.getDataBlk().setData(data, offset, msg.getLen());
  m_mandatory_q_ptr->enqueue(msg);
  active_request.bytes_issued += msg.getLen();

  return active_request.id;
}

void DMASequencer::issueNext()
{
  assert(m_is_busy == true);
  active_request.bytes_completed = active_request.bytes_issued;
  if (active_request.len == active_request.bytes_completed) {
    m_hit_callback(active_request.id);
    m_is_busy = false;
    return;
  }

  SequencerMsg msg;
  msg.getPhysicalAddress() = Address(active_request.start_paddr + 
				     active_request.bytes_completed);
  assert((msg.getPhysicalAddress().getAddress() & m_data_block_mask) == 0);
  msg.getLineAddress() = line_address(msg.getPhysicalAddress());
  msg.getType() = (active_request.write ? SequencerRequestType_ST : 
		   SequencerRequestType_LD);
  msg.getLen() = (active_request.len - 
		  active_request.bytes_completed < RubySystem::getBlockSizeBytes() ?
		  active_request.len - active_request.bytes_completed :
		  RubySystem::getBlockSizeBytes());
  if (active_request.write) {
    msg.getDataBlk().setData(&active_request.data[active_request.bytes_completed], 
			     0, msg.getLen());
    msg.getType() = SequencerRequestType_ST;
  } else {
    msg.getType() = SequencerRequestType_LD;
  }
  m_mandatory_q_ptr->enqueue(msg);
  active_request.bytes_issued += msg.getLen();
}

void DMASequencer::dataCallback(const DataBlock & dblk)
{
  assert(m_is_busy == true);
  int len = active_request.bytes_issued - active_request.bytes_completed;
  int offset = 0;
  if (active_request.bytes_completed == 0)
    offset = active_request.start_paddr & m_data_block_mask;
  assert( active_request.write == false );
  memcpy(&active_request.data[active_request.bytes_completed], 
	 dblk.getData(offset, len), len);
  issueNext();
}

void DMASequencer::ackCallback()
{
  issueNext();
}

void DMASequencer::printConfig(ostream & out)
{

}


DMASequencer *
DMASequencerParams::create()
{
    return new DMASequencer(this);
}
