#ifndef __DSENT_UTIL_RESULT_H__
#define __DSENT_UTIL_RESULT_H__

#include <iostream>
#include <vector>

#include "libutil/String.h"
#include "libutil/Map.h"

namespace DSENT
{
    using std::ostream;
    using std::vector;
    using LibUtil::Map;
    using LibUtil::String;

    class Result
    {
        public:
            class SubResult
            {
                public:
                    SubResult(const Result* result_, const String& producer_, double num_results_);
                    ~SubResult();

                public:
                    const Result* getResult() const;
                    const String& getProducer() const;
                    double getNumResults() const;

                    SubResult* clone() const;

                protected:
                    SubResult(const SubResult& sub_result_);

                private:
                    // Pointer to the actual result
                    const Result* m_result_;
                    // Name of the instance that produces this result
                    String m_producer_;
                    // Number of the times this result should be produce
                    double m_num_results_;
            }; // class SubResult

        public:
            Result();
            Result(const String& result_name_);
            virtual ~Result();

        public:
            // Get the name of result
            const String& getName() const;
            // Add a sub result
            void addSubResult(const Result* sub_result_, const String& result_producer_, double num_results_);
            // Remove all sub results
            void removeAllSubResults();
            // Set the value of a result, not available except for AtomicResult
            virtual void setValue(double value_);
            // Set the value of a result, not available except for AtomicResult
            virtual void addValue(double value_);
            // Get the value of a result, not available except for AtomicResult
            virtual double getValue() const;
            // Loop through all sub results and calculate the sum
            virtual double calculateSum() const;
            // Print the result with hierarchy if detail_level_ > 0. Print the sum when detail_level_ <= 0
            void print(const String& prepend_str_, int detail_level_, ostream& ost_) const;
            // Print the tree of the results
            void printHierarchy(const String& prepend_str_, int detail_level_, ostream& ost_) const;

            Result* clone() const;

        protected:
            Result(const Result& result_);
            virtual void print(const String& prepend_str_, double num_results_, int detail_level_, ostream& ost_) const;

        private:
            String m_result_name_;
            vector<SubResult*> m_sub_results_;
    }; // class Result

    class AtomicResult : public Result
    {
        public:
            AtomicResult(const String& result_name_, double value_ = 0.0);
            ~AtomicResult();

        public:
            void setValue(double value_);
            void addValue(double value_);
            double getValue() const;
            virtual double calculateSum() const;
            AtomicResult* clone() const;

        protected:
            AtomicResult(const AtomicResult& atomic_result_);
            virtual void print(const String& prepend_str_, double num_results_, int detail_level_, ostream& ost_) const;

        private:
            // Actual value of the result
            double m_value_;
    }; // class AtomicResult
} // namespace DSENT

#endif // __DSENT_UTIL_RESULT_H__

