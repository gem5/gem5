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

#ifndef __STRING_H__
#define __STRING_H__

#include <string>
#include <cstdarg>
#include <vector>
#include <sstream>
#include <bitset>

namespace LibUtil
{
    using std::string;
    using std::vector;

    class String : public string
    {
        public:
            static String format(const String& format_, ...);
            static String format(const String& format_, va_list args_);
            template<class T> static String toString(const T& value_);
            static String toBitString(unsigned int value_, unsigned int num_bits_);
            template<class T> static T fromString(const String& str_);

        private:
            static const unsigned int msBufferSize;

        public:
            String();
            String(const string& str_);
            String(const char* str_, size_t n);
            String(const char* str_);
            String(size_t n, char c);
            String(int value_);
            String(unsigned int value_);
            String(long value_);
            String(unsigned long value_);
            String(float value_);
            String(double value_);
            String(bool value_);
            ~String();

        public:
            // Remove leading and trailing whitespace
            String& trim();
            // Substitute str1 with str2
            String& substitute(const String& str1_, const String& str2_);
            // Split the String into vector of Strings separated by delimiters_
            vector<String> split(const char* delimiters_) const;
            vector<String> split(const String* delimiters_, unsigned int num_delimiters_ = 1) const;
            vector<String> splitByString(const String& delimiters_) const;

            // Check if contains str
            bool contain(const String& str_) const;

        public:
            // Convertions
            const char* toCString() const;
            int toInt() const;
            unsigned int toUInt() const;
            long toLong() const;
            unsigned long toULong() const;
            float toFloat() const;
            double toDouble() const;
            bool toBool() const;
            operator const char*() const;
            operator int() const;
            operator unsigned int() const;
            operator long() const;
            operator unsigned long() const;
            operator float() const;
            operator double() const;
            operator bool() const;
            String& operator=(char c_);
    };

    template<class T> String String::toString(const T& value_)
    {
        std::ostringstream ost;
        ost << value_;
        return ost.str();
    }

    template<> inline String String::toString<bool>(const bool& value_)
    {
        if(value_ == true)
        {
            return "TRUE";
        }
        else
        {
            return "FALSE";
        }
    }

    inline String String::toBitString(unsigned int value_, unsigned int num_bits_)
    {
        std::bitset<sizeof(unsigned int)*8> bitSet(value_);
        String ret = String(bitSet.to_string());
        ret = ret.substr(ret.length()-num_bits_);
        return ret;
    }

    template<class T> T String::fromString(const String& str_)
    {
        T ret;
        std::istringstream ist(str_);
        ist >> ret;
        return ret;
    }

    template<> inline String String::fromString<String>(const String& str_)
    {
        return str_;
    }

    template<> inline bool String::fromString<bool>(const String& str_)
    {
        bool ret;
        if((str_ == String("TRUE")) || (str_ == String("true")))
        {
            ret = true;
        }
        else if((str_ == string("FALSE")) || (str_ == String("false")))
        {
            ret = false;
        }
        else
        {
            //std::cerr << "Invalid bool value: " << str_ << std::endl;
            throw ("Invalid bool value: " + str_);
        }
        return ret;
    }

    template<class T> String arrayToString(
            const T* array_, unsigned int start_index_, unsigned int end_index_, 
            const String& delimiters_
            )
    {
        // Ensure end_index_ >= start_index_ + 1
        if(end_index_ <= start_index_)
        {
            throw("Invalid index range: start_index = " + (String)start_index_ + ", end_index = " + (String)end_index_);
        }

        String ret = "[";
        for(unsigned int i = start_index_; i < (end_index_-1); ++i)
        {
            ret += (String)array_[i] + delimiters_;
        }
        ret += (String)array_[end_index_-1] + "]";
        return ret;
    }

    template<class T> String arrayToString(const T* array_, unsigned int num_elements_)
    {
        return arrayToString(array_, 0, num_elements_, ", ");
    }

    template<class T> String arrayToString(const T* array_, unsigned int start_index_, unsigned int end_index_)
    {
        return arrayToString(array_, start_index_, end_index_);
    }

    template<class T> String vectorToString(
        const vector<T>& vector_, unsigned int start_index_, unsigned int end_index_,
        const String& delimiters_
        )
    {
        // Ensure end_index_ >= start_index_ + 1, or if the vector is empty
        if((end_index_ <= start_index_) || (end_index_ > vector_.size()))
        {
            // If the vector is empty, return empty array
            if (vector_.size() == 0)
                return "[]";
                
            throw("Invalid index range: start_index = " + (String)start_index_ + ", end_index = " + (String)end_index_);
        }

        String ret = "[";
        for(unsigned int i = start_index_; i < (end_index_-1); ++i)
        {
            ret += (String)vector_[i] + delimiters_;
        }
        ret += (String)vector_[end_index_-1] + "]";
        return ret;
    }

    template<class T> String vectorToString(const vector<T>& vector_)
    {
        return vectorToString(vector_, 0, vector_.size(), ", ");
    }

    template<class T> String vectorToString(const vector<T>& vector_, unsigned int num_elements_)
    {
        return vectorToString(vector_, 0, num_elements_, ", ");
    }

    template<class T> String vectorToString(const vector<T>& vector_, unsigned int start_index_, unsigned int end_index_)
    {
        return vectorToString(vector_, start_index_, end_index_);
    }

    template<class T> vector<T> castStringVector(const vector<String>& vector_)
    {
        vector<T> ret_vector;
        for(unsigned int i = 0; i < vector_.size(); ++i)
        {
            ret_vector.push_back((T)vector_[i]);
        }
        return ret_vector;
    }

    std::istream& safeGetline(std::istream& is_, String& str_);
} // namespace LibUtil

#endif // __STRING_H__

