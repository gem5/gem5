#include "mem/ruby/network/garnet/fixed-pipeline/routing/aco/AntMsg.hh"

using namespace std;

AntMsg::AntMsg(const Tick curTime, bool forward, int sourceRouter, int destinationRouter)
: Message(curTime)
{
    m_forward = forward;
    m_sourceRouter = sourceRouter;
    m_destinationRouter = destinationRouter;

    MachineID mach = {MachineType_Directory, (NodeID) m_destinationRouter};
    m_Destination.add(mach);
}

void
AntMsg::setForward(bool forward)
{
    m_forward = forward;
}

bool
AntMsg::isForward()
{
    return m_forward;
}

void
AntMsg::setSourceRouter(int sourceRouter)
{
    m_sourceRouter = sourceRouter;
}

int
AntMsg::getSourceRouter()
{
    return m_sourceRouter;
}

void
AntMsg::setDestinationRouter(int destinationRouter)
{
    m_destinationRouter = destinationRouter;
}

int
AntMsg::getDestinationRouter()
{
    return m_destinationRouter;
}

vector<int>&
AntMsg::getMemory()
{
    return m_memory;
}

/** Print the state of this object */
void
AntMsg::print(ostream& out) const
{
    out << "[AntMsg: ";
    out << "forward = " << (m_forward ? "TRUE" : "FALSE") << " ";
    out << "sourceRouter = " << m_sourceRouter << " ";
    out << "destinationRouter = " << m_destinationRouter << " ";
    out << "destination = " << m_Destination << " ";
    out << "MessageSize = " << m_MessageSize << " ";
    out << "]";
}

bool
AntMsg::functionalWrite(Packet* param_pkt)
{
    //return (testAndWrite(m_addr, m_DataBlk, param_pkt));
    panic("AntMsg does not support functional accesses!");
    ;
}

bool
AntMsg::functionalRead(Packet* param_pkt)
{
    //return (testAndRead(m_addr, m_DataBlk, param_pkt));
    panic("AntMsg does not support functional accesses!");
    ;
}