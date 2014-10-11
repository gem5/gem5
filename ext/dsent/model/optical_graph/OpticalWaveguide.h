#ifndef __DSENT_MODEL_OPTICALGRAPH_OPTICALWAVEGUIDE_H__
#define __DSENT_MODEL_OPTICALGRAPH_OPTICALWAVEGUIDE_H__

#include "model/optical_graph/OpticalNode.h"
#include "util/CommonType.h"

namespace DSENT
{
    class OpticalWaveguide : public OpticalNode
    {
        public:
            OpticalWaveguide(const String& instance_name_, OpticalModel* model_, const WavelengthGroup& wavelengths_);
            ~OpticalWaveguide();

        public:
            // Nothing here...
        
        private:
            // Disable copy constructor
            OpticalWaveguide(const OpticalWaveguide& node_);            
        
    };
    
} // namespace DSENT

#endif // __DSENT_MODEL_OPTICALGRAPH_OPTICALWAVEGUIDE_H__

