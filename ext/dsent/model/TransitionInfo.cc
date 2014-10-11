/* Copyright (c) 2012 Massachusetts Institute of Technology
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "model/TransitionInfo.h"

namespace DSENT
{
    TransitionInfo::TransitionInfo()
        : m_number_transitions_00_(0.25), m_number_transitions_01_(0.25), 
        m_number_transitions_11_(0.25), m_frequency_multiplier_(1.0)
    {
        update();
    }

    TransitionInfo::TransitionInfo(double number_transitions_00_, double number_transitions_01_,
            double number_transitions_11_)
        : m_number_transitions_00_(number_transitions_00_), m_number_transitions_01_(number_transitions_01_),
        m_number_transitions_11_(number_transitions_11_)
    {
        m_frequency_multiplier_ = m_number_transitions_00_ + 2.0 * m_number_transitions_01_ + m_number_transitions_11_;
        update();
    }

    TransitionInfo::~TransitionInfo()
    {}

    void TransitionInfo::update()
    {
        ASSERT(m_number_transitions_00_ >= 0.0, "[Error] Number of 0->0 transitions (" + 
                (String)m_number_transitions_00_ + ") must be >= 0.0!");
        ASSERT(m_number_transitions_01_ >= 0.0, "[Error] Number of 0->1 transitions (" + 
                (String)m_number_transitions_01_ + ") must be >= 0.0!");
        ASSERT(m_number_transitions_11_ >= 0.0, "[Error] Number of 1->1 transitions (" + (String)m_number_transitions_11_ + ") must be >= 0.0!");

        m_probability_1_ = (m_number_transitions_01_ + m_number_transitions_11_) / m_frequency_multiplier_;
        m_probability_0_ = 1.0 - m_probability_1_;
        return;
    }

    TransitionInfo TransitionInfo::scaleFrequencyMultiplier(double frequency_multiplier_) const
    {
        // A short cut if frequency_multiplier_ == m_frequency_multiplier_ to avoid excess calculations
        if(frequency_multiplier_ == m_frequency_multiplier_)
        {
            return TransitionInfo(m_number_transitions_00_, m_number_transitions_01_, m_number_transitions_11_);
        }
        else if (frequency_multiplier_ > m_frequency_multiplier_)
        {
            double freq_ratio = frequency_multiplier_ / m_frequency_multiplier_;
            double diff_num_trans_01 = m_number_transitions_01_ * (freq_ratio - 1.0);
            double norm_num_trans_00 = m_number_transitions_00_ * freq_ratio + diff_num_trans_01;
            double norm_num_trans_01 = m_number_transitions_01_;
            double norm_num_trans_11 = m_number_transitions_11_ * freq_ratio + diff_num_trans_01;

            return TransitionInfo(norm_num_trans_00, norm_num_trans_01, norm_num_trans_11);
        }
        else
        {
            ASSERT(false, "[Error] Cannot scale to frequency multiplier (" + (String) frequency_multiplier_ +
                ") since current frequency multiplier (" + (String) m_frequency_multiplier_ + ") is bigger!");
        }
    }

    void TransitionInfo::print(std::ostream& ost_) const
    {
        ost_ << "[" << m_number_transitions_00_ << ", " << m_number_transitions_01_ << ", " << m_number_transitions_11_ << "]";
        ost_ << " [" << m_number_transitions_00_ / m_frequency_multiplier_;
        ost_ << ", " << m_number_transitions_01_*2.0 / m_frequency_multiplier_;
        ost_ << ", " << m_number_transitions_11_ / m_frequency_multiplier_ << "]" << endl;
        return;
    }
} // namespace DSENT

