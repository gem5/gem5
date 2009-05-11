
/*
 * Copyright (c) 1999 by Mark Hill and David Wood for the Wisconsin
 * Multifacet Project.  ALL RIGHTS RESERVED.
 *
 * ##HEADER##
 *
 * This software is furnished under a license and may be used and
 * copied only in accordance with the terms of such license and the
 * inclusion of the above copyright notice.  This software or any
 * other copies thereof or any derivative works may not be provided or
 * otherwise made available to any other persons.  Title to and
 * ownership of the software is retained by Mark Hill and David Wood.
 * Any use of this software must include the above copyright notice.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS".  THE LICENSOR MAKES NO
 * WARRANTIES ABOUT ITS CORRECTNESS OR PERFORMANCE.
 * */

/*
 * $Id$
 */

#ifndef ADDRESS_H
#define ADDRESS_H

#include <iomanip>
#include "mem/ruby/common/Global.hh"
#include "mem/ruby/config/RubyConfig.hh"
#include "mem/ruby/system/NodeID.hh"
#include "mem/ruby/system/MachineID.hh"

const int ADDRESS_WIDTH = 64; // address width in bytes

class Address;
typedef Address PhysAddress;
typedef Address VirtAddress;

class Address {
public:
  // Constructors
  Address() { m_address = 0; }
  explicit Address(physical_address_t address) { m_address = address; }

  Address(const Address& obj);
  Address& operator=(const Address& obj);

  // Destructor
  //  ~Address();

  // Public Methods

  void setAddress(physical_address_t address) { m_address = address; }
  physical_address_t getAddress() const {return m_address;}
  // selects bits inclusive
  physical_address_t bitSelect(int small, int big) const;
  physical_address_t maskLowOrderBits(int number) const;
  physical_address_t maskHighOrderBits(int number) const;
  physical_address_t shiftLowOrderBits(int number) const;
  physical_address_t getLineAddress() const
    { return bitSelect(RubyConfig::dataBlockBits(), ADDRESS_WIDTH); }
  physical_address_t getOffset() const
    { return bitSelect(0, RubyConfig::dataBlockBits()-1); }

  void makeLineAddress() { m_address = maskLowOrderBits(RubyConfig::dataBlockBits()); }
  // returns the next stride address based on line address
  void makeNextStrideAddress( int stride) {
    m_address = maskLowOrderBits(RubyConfig::dataBlockBits())
      + RubyConfig::dataBlockBytes()*stride;
  }
  void makePageAddress() { m_address = maskLowOrderBits(RubyConfig::pageSizeBits()); }
  int getBankSetNum() const;
  int getBankSetDist() const;

  Index memoryModuleIndex() const;

  void print(ostream& out) const;
  void output(ostream& out) const;
  void input(istream& in);

  void setOffset( int offset ){
    // first, zero out the offset bits
    makeLineAddress();
    m_address |= (physical_address_t) offset;
  }

private:
  // Private Methods

  // Private copy constructor and assignment operator
  //  Address(const Address& obj);
  //  Address& operator=(const Address& obj);

  // Data Members (m_ prefix)
  physical_address_t m_address;
};

inline
Address line_address(const Address& addr) { Address temp(addr); temp.makeLineAddress(); return temp; }

inline
Address next_stride_address(const Address& addr, int stride) {
  Address temp = addr;
  temp.makeNextStrideAddress(stride);
  temp.setAddress(temp.maskHighOrderBits(ADDRESS_WIDTH-RubyConfig::memorySizeBits()));  // surpress wrap-around problem
  return temp;
}

inline
Address page_address(const Address& addr) { Address temp(addr); temp.makePageAddress(); return temp; }

// Output operator declaration
ostream& operator<<(ostream& out, const Address& obj);
// comparison operator declaration
bool operator==(const Address& obj1, const Address& obj2);
bool operator!=(const Address& obj1, const Address& obj2);
bool operator<(const Address& obj1, const Address& obj2);
/* Address& operator=(const physical_address_t address); */

inline
bool operator<(const Address& obj1, const Address& obj2)
{
  return obj1.getAddress() < obj2.getAddress();
}

// ******************* Definitions *******************

// Output operator definition
inline
ostream& operator<<(ostream& out, const Address& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

inline
bool operator==(const Address& obj1, const Address& obj2)
{
  return (obj1.getAddress() == obj2.getAddress());
}

inline
bool operator!=(const Address& obj1, const Address& obj2)
{
  return (obj1.getAddress() != obj2.getAddress());
}

inline
physical_address_t Address::bitSelect(int small, int big) const // rips bits inclusive
{
  physical_address_t mask;
  assert(big >= small);

  if (big >= ADDRESS_WIDTH - 1) {
    return (m_address >> small);
  } else {
    mask = ~((physical_address_t)~0 << (big + 1));
    // FIXME - this is slow to manipulate a 64-bit number using 32-bits
    physical_address_t partial = (m_address & mask);
    return (partial >> small);
  }
}

inline
physical_address_t Address::maskLowOrderBits(int number) const
{
  physical_address_t mask;

  if (number >= ADDRESS_WIDTH - 1) {
    mask = ~0;
  } else {
    mask = (physical_address_t)~0 << number;
  }
  return (m_address & mask);
}

inline
physical_address_t Address::maskHighOrderBits(int number) const
{
  physical_address_t mask;

  if (number >= ADDRESS_WIDTH - 1) {
    mask = ~0;
  } else {
    mask = (physical_address_t)~0 >> number;
  }
  return (m_address & mask);
}

inline
physical_address_t Address::shiftLowOrderBits(int number) const
{
  return (m_address >> number);
}

inline
integer_t Address::memoryModuleIndex() const
{
  integer_t index = bitSelect(RubyConfig::dataBlockBits()+RubyConfig::memoryBits(), ADDRESS_WIDTH);
  assert (index >= 0);
  if (index >= RubyConfig::memoryModuleBlocks()) {
    cerr << " memoryBits: " << RubyConfig::memoryBits() << " memorySizeBits: " << RubyConfig::memorySizeBits()
         << " Address: " << "[" << hex << "0x" << m_address << "," << " line 0x" << maskLowOrderBits(RubyConfig::dataBlockBits()) << dec << "]" << flush
         << "error: limit exceeded. " <<
      " dataBlockBits: " << RubyConfig::dataBlockBits() <<
      " memoryModuleBlocks: " << RubyConfig::memoryModuleBlocks() <<
      " index: " << index << endl;
  }
  assert (index < RubyConfig::memoryModuleBlocks());
  return index;

  //  Index indexHighPortion = address.bitSelect(MEMORY_SIZE_BITS-1, PAGE_SIZE_BITS+NUMBER_OF_MEMORY_MODULE_BITS);
  //  Index indexLowPortion  = address.bitSelect(DATA_BLOCK_BITS, PAGE_SIZE_BITS-1);

  //Index index = indexLowPortion | (indexHighPortion << (PAGE_SIZE_BITS - DATA_BLOCK_BITS));

  /*
  Round-robin mapping of addresses, at page size granularity

ADDRESS_WIDTH    MEMORY_SIZE_BITS        PAGE_SIZE_BITS  DATA_BLOCK_BITS
  |                    |                       |               |
 \ /                  \ /                     \ /             \ /       0
  -----------------------------------------------------------------------
  |       unused        |xxxxxxxxxxxxxxx|       |xxxxxxxxxxxxxxx|       |
  |                     |xxxxxxxxxxxxxxx|       |xxxxxxxxxxxxxxx|       |
  -----------------------------------------------------------------------
                        indexHighPortion         indexLowPortion
                                        <------->
                               NUMBER_OF_MEMORY_MODULE_BITS
  */
}

inline
void Address::print(ostream& out) const
{
  out << "[" << hex << "0x" << m_address << "," << " line 0x" << maskLowOrderBits(RubyConfig::dataBlockBits()) << dec << "]" << flush;
}

class Address;
namespace __gnu_cxx {
  template <> struct hash<Address>
  {
    size_t operator()(const Address &s) const { return (size_t) s.getAddress(); }
  };
}
namespace std {
  template <> struct equal_to<Address>
  {
    bool operator()(const Address& s1, const Address& s2) const { return s1 == s2; }
  };
}

#endif //ADDRESS_H

