
#ifndef MEMORYVECTOR_H
#define MEMORYVECTOR_H

#include "mem/ruby/common/Address.hh"

class DirectoryMemory;

/**
 *  MemoryVector holds memory data (DRAM only)
 */
class MemoryVector {
 public:
  MemoryVector();
  MemoryVector(uint32 size);
  ~MemoryVector();
  friend class DirectoryMemory;

  void setSize(uint32 size);  // destructive

  void write(const Address & paddr, uint8* data, int len);
  uint8* read(const Address & paddr, uint8* data, int len);

 private:
  uint8* getBlockPtr(const Address & paddr);

  uint32 m_size;
  uint8* m_vec;
};

inline
MemoryVector::MemoryVector()
{
  m_size = 0;
  m_vec = NULL;
}

inline
MemoryVector::MemoryVector(uint32 size)
{
  m_size = size;
  m_vec = new uint8[size];
}

inline
MemoryVector::~MemoryVector()
{
  delete [] m_vec;
}

inline
void MemoryVector::setSize(uint32 size)
{
  m_size = size;
  if (m_vec != NULL)
    delete [] m_vec;
  m_vec = new uint8[size];
}

inline
void MemoryVector::write(const Address & paddr, uint8* data, int len)
{
  assert(paddr.getAddress() + len <= m_size);
  memcpy(m_vec + paddr.getAddress(), data, len);
}

inline
uint8* MemoryVector::read(const Address & paddr, uint8* data, int len)
{
  assert(paddr.getAddress() + len <= m_size);
  memcpy(data, m_vec + paddr.getAddress(), len);
  return data;
}

inline
uint8* MemoryVector::getBlockPtr(const Address & paddr)
{
  return m_vec + paddr.getAddress();
}

#endif // MEMORYVECTOR_H
