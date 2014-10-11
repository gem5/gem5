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

#include "util/Result.h"

#include <iostream>

#include "libutil/Log.h"
#include "libutil/Assert.h"

namespace DSENT
{
    using std::ostream;
    using std::endl;

    Result::SubResult::SubResult(const Result* sub_result_, const String& producer_, double num_results_)
        : m_result_(sub_result_), m_producer_(producer_), m_num_results_(num_results_)
    {
        // Check if the result is not a null pointer
        ASSERT((sub_result_ != NULL), "Internal error: sub_result_ is null");

        // Check if the number of results greater than 0
        ASSERT((num_results_ >= 0), "Internal error: num_results_ (" + String(num_results_) + ") is less than 0");
    }

    Result::SubResult::~SubResult()
    {}

    const Result* Result::SubResult::getResult() const
    {
        return m_result_;
    }

    const String& Result::SubResult::getProducer() const
    {
        return m_producer_;
    }

    double Result::SubResult::getNumResults() const
    {
        return m_num_results_;
    }

    Result::SubResult* Result::SubResult::clone() const
    {
        return new SubResult(*this);
    }

    Result::SubResult::SubResult(const SubResult& sub_result_)
        : m_result_(sub_result_.m_result_), m_producer_(sub_result_.m_producer_), m_num_results_(sub_result_.m_num_results_)
    {}

    Result::Result()
    {}

    Result::Result(const String& result_name_)
        : m_result_name_(result_name_)
    {}

    Result::~Result()
    {
        // Clear all sub results
        for(vector<SubResult*>::iterator it = m_sub_results_.begin(); 
            it != m_sub_results_.end(); ++it)
        {
            SubResult* sub_result = (*it);
            delete sub_result;
        }
    }

    const String& Result::getName() const
    {
        return m_result_name_;
    }

    void Result::setValue(double /* value_ */)
    {
        throw LibUtil::Exception("[Error] " + getName() + " -> Cannot set the value of a non-atomic result!");
        return;
    }

    void Result::addValue(double /* value_ */)
    {
        throw LibUtil::Exception("[Error] " + getName() + 
                " -> Cannot add the value of a non-atomic result");
        return;
    }

    double Result::getValue() const
    {
        throw LibUtil::Exception("[Error] " + getName() + " -> Cannot get the value of a non-atomic result!");
        return 0.0;
    }    
    
    void Result::addSubResult(const Result* sub_result_, const String& result_producer_, double num_results_)
    {
        SubResult* new_sub_result = new SubResult(sub_result_, result_producer_, num_results_);
        m_sub_results_.push_back(new_sub_result);
        return;
    }

    void Result::removeAllSubResults()
    {
        // Clear all sub results
        for(vector<SubResult*>::iterator it = m_sub_results_.begin(); 
            it != m_sub_results_.end(); ++it)
        {
            SubResult* sub_result = (*it);
            delete sub_result;
        }
        m_sub_results_.clear();
        return;
    }

    double Result::calculateSum() const
    {
        double sum = 0.0;

        // Loop through all sub results and calculate the sum
        for(vector<SubResult*>::const_iterator it = m_sub_results_.begin();
            it != m_sub_results_.end(); ++it)
        {
            const SubResult* temp_sub_result = (*it);
            const Result* temp_result = temp_sub_result->getResult();
            double num_results = temp_sub_result->getNumResults();
            sum += temp_result->calculateSum()*num_results;
        }
        return sum;
    }

    void Result::print(const String& prepend_str_, int detail_level_, ostream& ost_) const
    {
        print(prepend_str_, 1.0, detail_level_, ost_);
        return;
    }

    Result* Result::clone() const
    {
        return new Result(*this);
    }

    Result::Result(const Result& result_)
    {
        // Copy the result name
        m_result_name_ = result_.m_result_name_;

        // Clone all sub results
        for(vector<SubResult*>::const_iterator it = m_sub_results_.begin(); 
            it != m_sub_results_.end(); ++it)
        {
            const SubResult* temp_sub_result = (*it);
            SubResult* new_sub_result = temp_sub_result->clone();
            m_sub_results_.push_back(new_sub_result);
        }
    }

    void Result::print(const String& prepend_str_, double num_results_, int detail_level_, ostream& ost_) const
    {
        // Go down to lower level if detail_level_ > 0, else print the sthe sthe sthe sum
        if(detail_level_ > 0)
        {
            for(vector<SubResult*>::const_iterator it = m_sub_results_.begin();
                    it != m_sub_results_.end(); ++it)
            {
                const SubResult* temp_sub_result = (*it);
                const Result* temp_result = temp_sub_result->getResult();
                const String& temp_producer = temp_sub_result->getProducer();
                const String& temp_result_name = temp_result->getName();
                double temp_num_results = temp_sub_result->getNumResults();
                String temp_prepend_str = prepend_str_ + "->" + temp_producer;

                if(!temp_result_name.empty())
                {
                    temp_prepend_str += ":" + temp_result_name;
                }
                temp_result->print(temp_prepend_str, num_results_*temp_num_results, detail_level_ - 1, ost_);
            }
        }
        else
        {
            ost_ << prepend_str_ << " = " << calculateSum()*num_results_;
            ost_ << " (" << calculateSum() << " * " << num_results_ << ")" << endl;
        }
        return;
    }

    void Result::printHierarchy(const String& prepend_str_, int detail_level_, ostream& ost_) const
    {
        if(detail_level_ > 0)
        {
            for(vector<SubResult*>::const_iterator it = m_sub_results_.begin(); it != m_sub_results_.end(); ++it)
            {
                const SubResult* temp_sub_result = (*it);
                const Result* temp_result = temp_sub_result->getResult();
                const String& temp_producer = temp_sub_result->getProducer();
                const String& temp_result_name = temp_result->getName();
                String temp_prepend_str = prepend_str_ + "    ";

                ost_ << prepend_str_ << " |--" << temp_producer << "->" << temp_result_name << endl;

                temp_result->printHierarchy(temp_prepend_str, detail_level_ - 1, ost_);
            }
        }
        return;
    }

    AtomicResult::AtomicResult(const String& result_name_, double value_)
        : Result(result_name_), m_value_(value_)
    {}

    AtomicResult::~AtomicResult()
    {}

    void AtomicResult::setValue(double value_)
    {
        m_value_ = value_;
        return;
    }
    
    void AtomicResult::addValue(double value_)
    {
        m_value_ += value_;
        return;
    }

    double AtomicResult::getValue() const
    {
        return m_value_;
    }

    double AtomicResult::calculateSum() const
    {
        return m_value_;
    }

    AtomicResult* AtomicResult::clone() const
    {
        return new AtomicResult(*this);
    }

    AtomicResult::AtomicResult(const AtomicResult& atomic_result_)
        : Result(atomic_result_), m_value_(atomic_result_.m_value_)
    {}

    void AtomicResult::print(const String& prepend_str_, double num_results_, int /* detail_level_ */, ostream& ost_) const
    {
        ost_ << prepend_str_ << " = " << m_value_*num_results_;
        ost_ << " (" << m_value_ << " * " << num_results_ << ")" << endl;
        return;
    }
} // namespace DSENT

