#ifndef __DSENT_MODEL_TRANSITION_INFO_H__
#define __DSENT_MODEL_TRANSITION_INFO_H__

#include "util/CommonType.h"

namespace DSENT
{
    /**
     * \brief This class contains the number of transitions and the frequency multiplier information
     */
    class TransitionInfo
    {
        public:
            TransitionInfo();
            TransitionInfo(double number_transitions_00_, double number_transitions_01_, double number_transitions_11_);
            ~TransitionInfo();

        public:
            inline double getNumberTransitions00() const { return m_number_transitions_00_; }
            inline double getNumberTransitions01() const { return m_number_transitions_01_; }
            inline double getNumberTransitions11() const { return m_number_transitions_11_; }
            inline double getFrequencyMultiplier() const { return m_frequency_multiplier_; }
            inline double getProbability0() const { return m_probability_1_; }
            inline double getProbability1() const { return m_probability_1_; }

            inline void setNumberTransitions00(double value_) { m_number_transitions_00_ = value_; }
            inline void setNumberTransitions01(double value_) { m_number_transitions_01_ = value_; }
            inline void setNumberTransitions11(double value_) { m_number_transitions_11_ = value_; }
            inline void setFrequencyMultiplier(double value_) { m_frequency_multiplier_ = value_; }

            void update();
            TransitionInfo scaleFrequencyMultiplier(double frequency_multiplier_) const;

            void print(std::ostream& ost_) const;

        private:
            // m_number_transitions_xy_ defines number of transitions from x to y
            double m_number_transitions_00_;
            double m_number_transitions_01_;
            double m_number_transitions_11_;
            // The multiplier that defines the ratio of the transition frequency and the main core clock frequency
            double m_frequency_multiplier_;
            // Probability of being at state 0/1
            double m_probability_0_;
            double m_probability_1_;
    }; // class TransitionInfo
} // namespace DSENT

#endif // __DSENT_MODEL_TRANSITION_INFO_H__

