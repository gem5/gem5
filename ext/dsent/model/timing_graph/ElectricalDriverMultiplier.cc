
#include "model/timing_graph/ElectricalDriverMultiplier.h"
#include "model/timing_graph/ElectricalLoad.h"

namespace DSENT
{
    //-------------------------------------------------------------------------
    // Electrical Net
    //-------------------------------------------------------------------------

    ElectricalDriverMultiplier::ElectricalDriverMultiplier(const String& instance_name_, ElectricalModel* model_)
        : ElectricalTimingNode(instance_name_, model_)
    {

    }

    ElectricalDriverMultiplier::~ElectricalDriverMultiplier()
    {

    }

    double ElectricalDriverMultiplier::calculateDriveRes( double input_drive_res_) const
    {
        return input_drive_res_;
    }

    double ElectricalDriverMultiplier::calculateDelay() const
    {
        // This is just a model helper element, it does not contribute delay
        return 0;
    }

    double ElectricalDriverMultiplier::calculateTransition() const
    {
        return getMaxUpstreamRes() * getTotalDownstreamCap();
    }
    
    double ElectricalDriverMultiplier::getTotalDownstreamCap() const
    {
        // Finds the max of the load caps (as opposed to summing)
        double max_cap = 0;
        vector<ElectricalTimingNode*>* downstream_nodes = ElectricalTimingNode::getDownstreamNodes();
        for (unsigned int i = 0; i < downstream_nodes->size(); ++i)
        {
            max_cap = std::max(max_cap, downstream_nodes->at(i)->getTotalDownstreamCap());
        }
        
        return max_cap;
    }    
    
} // namespace DSENT


