#ifndef __DSENT_MODEL_OPTICAL_RINGMODULATOR_H__
#define __DSENT_MODEL_OPTICAL_RINGMODULATOR_H__

#include "util/CommonType.h"
#include "model/OpticalModel.h"
#include "model/optical_graph/OpticalTransmitter.h"

namespace DSENT
{
    class RingModulator : public OpticalModel, public OpticalTransmitter
    {
        public:
            RingModulator(const String& instance_name_, const TechModel* tech_model_);
            virtual ~RingModulator();

        public:
            // Set a list of properties' name needed to construct model
            void initParameters();
            // Set a list of properties' name needed to construct model
            void initProperties();
            // Set the transmitter specifications, returns whether it is possible
            // to build a modulator that met those specs
            bool setTransmitterSpec(double IL_dB_, double ER_dB_);
            // Returns power of the transmitter at a given utilization
            double getPower(double util_) const;
        
        private:
            // Precompute values based on tech parameters
            void precomputeTech();
            // Design ring modulator driver
            bool designModulator(double IL_dB_, double ER_dB_);
            // Calculate modulator energy
            double calcModulatorEnergy() const;
            
        protected:
            // Build the model
            virtual void constructModel();
            virtual void updateModel();
            virtual void useModel();
            virtual void propagateTransitionInfo();

        private:
            // Some precomputed tech values
            double m_precompute_V_bi_;
            double m_precompute_x_d0_;
            double m_precompute_C_j0_;
            double m_precompute_Q_0_;
            
            
    }; // class RingModulator
} // namespace DSENT

#endif // __DSENT_MODEL_OPTICAL_RINGMODULATOR_H__

