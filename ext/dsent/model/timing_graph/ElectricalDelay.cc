
#include "model/timing_graph/ElectricalDelay.h"
#include "model/timing_graph/ElectricalLoad.h"

namespace DSENT
{
    //-------------------------------------------------------------------------
    // Electrical Delay
    //-------------------------------------------------------------------------

    ElectricalDelay::ElectricalDelay(const String& instance_name_, ElectricalModel* model_)
        : ElectricalTimingNode(instance_name_, model_), m_delay_(0.0)
    {

    }

    ElectricalDelay::~ElectricalDelay()
    {

    }

    void ElectricalDelay::setDelay(double delay_)
    {
        m_delay_ = delay_;
        return;
    }

    double ElectricalDelay::getDelay() const
    {
        return m_delay_;
    }
    
    double ElectricalDelay::calculateDelay() const
    {
        return m_delay_;
    }

    double ElectricalDelay::calculateTransition() const
    {
        return 1.386 * getMaxUpstreamRes() * getTotalDownstreamCap();
    }

    double ElectricalDelay::getMaxUpstreamRes() const
    {
        return ElectricalTimingNode::getMaxUpstreamRes();
    }
    
    double ElectricalDelay::getTotalDownstreamCap() const
    {
        return ElectricalTimingNode::getTotalDownstreamCap();
    }
    
} // namespace DSENT


