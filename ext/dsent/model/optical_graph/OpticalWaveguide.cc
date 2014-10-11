
#include "model/optical_graph/OpticalWaveguide.h"

namespace DSENT
{
    OpticalWaveguide::OpticalWaveguide(const String& instance_name_, OpticalModel* model_, const WavelengthGroup& wavelengths_)
        : OpticalNode(OpticalNode::WAVEGUIDE, instance_name_, model_, wavelengths_)
    {
        
    }

    OpticalWaveguide::~OpticalWaveguide()
    {

    }

            
} // namespace DSENT


