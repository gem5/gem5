
#include "mem/ruby/system/DMASequencer.hh"
#include "mem/ruby/buffers/MessageBuffer.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"

/* SLICC generated types */
#include "mem/protocol/DMARequestMsg.hh"
#include "mem/protocol/DMARequestType.hh"
#include "mem/protocol/DMAResponseMsg.hh"
#include "mem/ruby/system/System.hh"

DMASequencer::DMASequencer(const string & name)
  : RubyPort(name)
{
}

void DMASequencer::init(const vector<string> & argv)
{
  m_version = -1;
  m_controller = NULL;
  for (size_t i=0;i<argv.size();i+=2) {
    if (argv[i] == "controller")
      m_controller = RubySystem::getController(argv[i+1]);
    else if (argv[i] == "version")
      m_version = atoi(argv[i+1].c_str());
  }
  assert(m_controller != NULL);
  assert(m_version != -1);

  m_mandatory_q_ptr = m_controller->getMandatoryQueue();
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

  DMARequestMsg msg;
  msg.getPhysicalAddress() = Address(paddr);
  msg.getLineAddress() = line_address(msg.getPhysicalAddress());
  msg.getType() = write ? DMARequestType_WRITE : DMARequestType_READ;
  msg.getOffset() = paddr & m_data_block_mask;
  msg.getLen() = (msg.getOffset() + len) <= RubySystem::getBlockSizeBytes() ?
    len : 
    RubySystem::getBlockSizeBytes() - msg.getOffset();
  if (write) {
    msg.getType() = DMARequestType_WRITE;
    msg.getDataBlk().setData(data, msg.getOffset(), msg.getLen());
  } else {
    msg.getType() = DMARequestType_READ;
  }
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

  DMARequestMsg msg;
  msg.getPhysicalAddress() = Address(active_request.start_paddr + 
				     active_request.bytes_completed);
  assert((msg.getPhysicalAddress().getAddress() & m_data_block_mask) == 0);
  msg.getLineAddress() = line_address(msg.getPhysicalAddress());
  msg.getOffset() = 0;
  msg.getType() = (active_request.write ? DMARequestType_WRITE : 
		   DMARequestType_READ);
  msg.getLen() = (active_request.len - 
		  active_request.bytes_completed < RubySystem::getBlockSizeBytes() ?
		  active_request.len - active_request.bytes_completed :
		  RubySystem::getBlockSizeBytes());
  if (active_request.write) {
    msg.getDataBlk().setData(&active_request.data[active_request.bytes_completed], 
			     0, msg.getLen());
    msg.getType() = DMARequestType_WRITE;
  } else {
    msg.getType() = DMARequestType_READ;
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
