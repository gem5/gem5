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

    Addr wBase = pkt->getAddr();
    Addr wTail = wBase + pkt->getSize();
    Addr mBase = m_PhysicalAddress.getAddress();
    Addr mTail = mBase + m_Size;

    uint8_t * pktData = pkt->getPtr<uint8_t>(true);

    Addr cBase = std::max(wBase, mBase);
    Addr cTail = std::min(wTail, mTail);

    for (Addr i = cBase; i < cTail; ++i) {
        data[i - mBase] = pktData[i - wBase];
    }

    return cBase < cTail;
}
