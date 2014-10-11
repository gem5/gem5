
#include "model/timing_graph/ElectricalLoad.h"
#include "model/ElectricalModel.h"
#include "model/timing_graph/ElectricalDriver.h"

namespace DSENT
{
    ElectricalLoad::ElectricalLoad(const String& instance_name_, ElectricalModel* model_)
        : ElectricalTimingNode(instance_name_, model_), m_load_cap_(0.0)
    {
    }

    ElectricalLoad::~ElectricalLoad()
    {
    }

    void ElectricalLoad::setLoadCap(double load_cap_)
    {
        m_load_cap_ = load_cap_;
        return;
    }
    
    double ElectricalLoad::getLoadCap() const
    {
        return m_load_cap_;
    }
    
    bool ElectricalLoad::isLoad() const
    {
        return true;
    }    
    
    double ElectricalLoad::calculateDelay() const
    {
        return 0;
    }

    double ElectricalLoad::calculateTransition() const
    {
        return 1.386 * getMaxUpstreamRes() * getTotalDownstreamCap();
    }

    double ElectricalLoad::getTotalDownstreamCap() const
    {
        return m_load_cap_;
    }

} // namespace DSENT

