#include <iostream>

#include "mem/ruby/slicc_interface/RubyRequest.hh"

using namespace std;

void
RubyRequest::print(ostream& out) const
{
  out << "[RubyRequest: ";
  out << "LineAddress = " << m_LineAddress << " ";
  out << "PhysicalAddress = " << m_PhysicalAddress << " ";
  out << "Type = " << m_Type << " ";
  out << "ProgramCounter = " << m_ProgramCounter << " ";
  out << "AccessMode = " << m_AccessMode << " ";
  out << "Size = " << m_Size << " ";
  out << "Prefetch = " << m_Prefetch << " ";
//  out << "Time = " << getTime() << " ";
  out << "]";
}

bool
RubyRequest::functionalRead(Packet *pkt)
{
    // This needs a little explanation. Initially I thought that this
    // message should be read. But the way the memtester works for now,
    // we should not be reading this message as memtester updates the
    // functional memory only after a write has actually taken place.
    return false;
}

bool
RubyRequest::functionalWrite(Packet *pkt)
{
    // This needs a little explanation. I am not sure if this message
    // should be written. Essentially the question is how are writes
    // ordered. I am assuming that if a functional write is issued after
    // a timing write to the same address, then the functional write
    // has to overwrite the data for the timing request, even if the
    // timing request has still not been ordered globally.

    Address pktLineAddr(pkt->getAddr());
    pktLineAddr.makeLineAddress();

    if (pktLineAddr == m_LineAddress) {
        uint8_t *pktData = pkt->getPtr<uint8_t>(true);
        unsigned int size_in_bytes = pkt->getSize();
        unsigned startByte = pkt->getAddr() - m_LineAddress.getAddress();

        for (unsigned i = 0; i < size_in_bytes; ++i) {
            data[i + startByte] = pktData[i];
        }

        return true;
    }
    return false;
}
