#include "model/std_cells/StdCell.h"

#include "model/timing_graph/ElectricalNet.h"
#include "model/timing_graph/ElectricalDriver.h"
#include "model/timing_graph/ElectricalLoad.h"

#include <cmath>
#include <algorithm>

namespace DSENT
{
    StdCell::StdCell(const String& instance_name_, const TechModel* tech_model_)
        : ElectricalModel(instance_name_, tech_model_)
    {
        initParameters();
        initProperties();
    }

    StdCell::~StdCell()
    {

    }
    
    
    void StdCell::initParameters()
    {
        addParameterName("AvailableDrivingStrengths");
        return;
    }
    
    void StdCell::initProperties()
    {
        addPropertyName("DrivingStrength");
        return;
    }

    // Get PMOS to NMOS ratio
    double StdCell::getPToNRatio() const
    {
        return m_p_to_n_ratio_;
    }
    
    void StdCell::setPToNRatio(double p_to_n_ratio_)
    {
        m_p_to_n_ratio_ = p_to_n_ratio_;
    }
    
    // Get height of the standard cell taken by active transistors
    double StdCell::getActiveHeight() const
    {
        return m_active_height_;
    }
    
    void StdCell::setActiveHeight(double active_height_)
    {
        m_active_height_ = active_height_;
    }
    
    // Get total height of the standard cell including overheads
    double StdCell::getTotalHeight() const
    {
        return m_total_height_;
    }

    void StdCell::setTotalHeight(double total_height_)
    {
        m_total_height_ = total_height_;
    }

} // namespace DSENT

