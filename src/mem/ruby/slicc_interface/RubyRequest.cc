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
