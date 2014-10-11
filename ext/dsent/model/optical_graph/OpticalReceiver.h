#ifndef __DSENT_MODEL_OPTICALGRAPH_OPTICALRECEIVER_H__
#define __DSENT_MODEL_OPTICALGRAPH_OPTICALRECEIVER_H__

#include "model/OpticalModel.h"
#include "util/CommonType.h"

namespace DSENT
{
    // The job of an optical receiver interface is to provide a function to
    // return a sensitivity (in Amps)
    class OpticalReceiver
    {
        public:
            OpticalReceiver(){};
            virtual ~OpticalReceiver(){};

        public:
            // Returns the sensitivity of the receiver given an extinction ratio
            virtual double getSensitivity(double ER_dB_) const = 0;
    };
    
} // namespace DSENT

#endif // __DSENT_MODEL_OPTICALGRAPH_OPTICALRECEIVER_H__

