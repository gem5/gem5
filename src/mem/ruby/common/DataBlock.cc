
#include "mem/ruby/common/DataBlock.hh"

DataBlock &
DataBlock::operator=(const DataBlock & obj)
{
  if (this == &obj) {
    //    assert(false);
  } else {
    if (!m_alloc)
      m_data = new uint8[RubySystem::getBlockSizeBytes()];
    memcpy(m_data, obj.m_data, RubySystem::getBlockSizeBytes());
    m_alloc = true;
  }
  return *this;
}
