#include "model/PortInfo.h"

namespace DSENT
{
    PortInfo::PortInfo(const String& port_name_, const NetIndex& net_index_)
        : m_port_name_(port_name_), m_net_index_(net_index_), m_tran_info_(TransitionInfo())
    {
    }

    PortInfo::~PortInfo()
    {
    }

    const String& PortInfo::getPortName() const
    {
        return m_port_name_;
    }

    const NetIndex& PortInfo::getNetIndex() const
    {
        return m_net_index_;
    }

    void PortInfo::setTransitionInfo(const TransitionInfo& trans_info_)
    {
        m_tran_info_ = trans_info_;
        return;
    }

    const TransitionInfo& PortInfo::getTransitionInfo() const
    {
        return m_tran_info_;
    }
} // namespace DSENT

