#ifndef __DSENT_MODEL_OPTICAL_RINGDETECTOR_H__
#define __DSENT_MODEL_OPTICAL_RINGDETECTOR_H__

#include "util/CommonType.h"
#include "model/OpticalModel.h"
#include "model/optical_graph/OpticalReceiver.h"

namespace DSENT
{
    class RingDetector : public OpticalModel, public OpticalReceiver
    {
        public:
            // Receiver topolgy strings
            static const String INTEGRATINGSENSEAMP;
    
        public:
            RingDetector(const String& instance_name_, const TechModel* tech_model_);
            virtual ~RingDetector();

        public:
            // Set a list of properties' name needed to construct model
            void initParameters();
            // Set a list of properties' name needed to construct model
            void initProperties();
            // Returns the sensitivity of the receiver given an extinction ratio
            double getSensitivity(double ER_dB_) const;
            
        private:
            // Precompute values based on tech parameters
            void precomputeTech();
            // Design the receiver helper function
            void designReceiver();
            // Calculates inverse normal cdf
            double calcInvNormCdf(double num_);

        protected:
            // Build the model
            virtual void constructModel();
            virtual void updateModel();
            virtual void useModel();
            virtual void propagateTransitionInfo();
            
        private:
            // Precomputed numbers
            double m_quad_a_;
            double m_quad_b1_;
            double m_quad_b2_;
            double m_quad_c_;

    }; // class RingDetector
} // namespace DSENT

#endif // __DSENT_MODEL_OPTICAL_RINGDETECTOR_H__

