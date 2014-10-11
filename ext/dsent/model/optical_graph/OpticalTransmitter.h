#ifndef __DSENT_MODEL_OPTICALGRAPH_OPTICALTRANSMITTER_H__
#define __DSENT_MODEL_OPTICALGRAPH_OPTICALTRANSMITTER_H__

#include "model/OpticalModel.h"
#include "util/CommonType.h"

namespace DSENT
{
    // The job of a optical sender interface is to provide a function to
    // allow the insertion loss and extinction ratio to be changed
    class OpticalTransmitter
    {
        public:
            OpticalTransmitter(){};
            virtual ~OpticalTransmitter(){};

        public:
            // Set the transmitter specifications, returns whether it is possible
            // to build a modulator that met those specs
            virtual bool setTransmitterSpec(double IL_dB_, double ER_dB_) = 0;
            // Returns power of the transmitter at a given utilization
            virtual double getPower(double util_) const = 0;
    }; // class OpticalTransmitter
    
} // namespace DSENT

#endif // __DSENT_MODEL_OPTICALGRAPH_OPTICALTRANSMITTER_H__

