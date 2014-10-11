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

#include "String.h"

#include <cstdarg>
#include <cstdio>
#include <iostream>
#include <ios>

namespace LibUtil
{
    const unsigned int String::msBufferSize = 4096;

    String String::format(const String& format_, ...)
    {
        char buffer[msBufferSize];

        va_list args;
        va_start(args, format_);
        vsnprintf(buffer, msBufferSize, format_.c_str(), args);
        va_end(args);

        return (String)(buffer);
    }

    String String::format(const String& format_, va_list args_)
    {
        char buffer[msBufferSize];

        vsnprintf(buffer, msBufferSize, format_.c_str(), args_);

        return (String)(buffer);
    }

    String::String()
    {}

    String::String(const string& str_)
        : string(str_)
    {}

    String::String(const char* str_, size_t n_)
        : string(str_, n_)
    {}

    String::String(const char* str_)
        : string(str_)
    {}

    String::String(size_t n_, char c_)
        : string(n_, c_)
    {}

    String::String(int value_)
        : string(toString<int>(value_))
    {}

    String::String(unsigned int value_)
        : string(toString<unsigned int>(value_))
    {}

    String::String(long value_)
        : string(toString<long>(value_))
    {}

    String::String(unsigned long value_)
        : string(toString<unsigned long>(value_))
    {}

    String::String(float value_)
        : string(toString<float>(value_))
    {}

    String::String(double value_)
        : string(toString<double>(value_))
    {}

    String::String(bool value_)
        : string(toString<bool>(value_))
    {}

    String::~String()
    {}

    String& String::trim()
    {
        // Remove leading and trailing whitespace
        static const char whitespace[] = " \n\t\v\r\f";
        erase(0, find_first_not_of(whitespace));
        erase(find_last_not_of(whitespace) + 1U);
        return (*this);
    }

    String& String::substitute(const String& str1_, const String& str2_)
    {
        size_t str1Size = str1_.size();
        size_t str2Size = str2_.size();

        size_t pos;
        pos = find(str1_);
        while(pos != string::npos)
        {
            replace(pos, str1Size, str2_);
            pos += str2Size;
            pos = find(str1_, pos);
        }
        return (*this);
    }

    vector<String> String::split(const char* delimiters_) const
    {
        vector<String> result;

        if(size() == 0)
        {
            return result;
        }

        size_t currPos, nextPos;
        currPos = 0;
        nextPos = find_first_of(delimiters_);
        while(1)
        {
            if(nextPos == string::npos)
            {
                if(currPos != size())
                {
                    result.push_back(substr(currPos));
                }
                break;
            }

            if(nextPos != currPos)
            {
                result.push_back(substr(currPos, nextPos - currPos));
            }
            currPos = nextPos + 1;
            nextPos = find_first_of(delimiters_, currPos);
        }

        return result;
    }

    vector<String> String::split(const String* delimiters_, unsigned int num_delimiters_) const
    {
        vector<String> result;

        if(size() == 0)
        {
            return result;
        }

        if(num_delimiters_ == 1)
        {
            size_t currPos, nextPos;
            currPos = 0;
            nextPos = find(delimiters_[0]);
            while(1)
            {
                if(nextPos == String::npos)
                {
                    result.push_back(substr(currPos));
                    break;
                }

                if(nextPos != currPos)
                {
                    result.push_back(substr(currPos, nextPos - currPos));
                }
                currPos = nextPos + delimiters_[0].size();
                nextPos = find(delimiters_[0], currPos);
            }
        }
        else
        {
            // Currently the length of the delimiters are not checked
            unsigned int delimiterLength = 0;
            size_t currPos, nextPos;
            currPos = 0;
            nextPos = size();
            for(unsigned int i = 0; i < num_delimiters_; ++i)
            {
                size_t tempPos = find(delimiters_[i], currPos);
                if((tempPos != String::npos) && (tempPos < nextPos))
                {
                    nextPos = tempPos;
                    delimiterLength = delimiters_[i].size();
                }
            }
            while(1)
            {
                if((nextPos == String::npos) || (nextPos == size()))
                {
                    result.push_back(substr(currPos));
                    break;
                }

                if(nextPos != currPos)
                {
                    result.push_back(substr(currPos, nextPos - currPos));
                }
                currPos = nextPos + delimiterLength;
                nextPos = size();
                delimiterLength = 0;
                for(unsigned int i = 0; i < num_delimiters_; ++i)
                {
                    size_t tempPos = find(delimiters_[i], currPos);
                    if((tempPos != String::npos) && (tempPos < nextPos))
                    {
                        nextPos = tempPos;
                        delimiterLength = delimiters_[i].size();
                    }
                }
            }
        }
        return result;
    }

    vector<String> String::splitByString(const String& delimiter_) const
    {
        return split(&delimiter_, 1);
    }

    bool String::contain(const String& str_) const
    {
        return (find(str_) != String::npos);
    }

    const char* String::toCString() const
    {
        return this->c_str();
    }

    int String::toInt() const
    {
        return fromString<int>(*this);
    }

    unsigned int String::toUInt() const
    {
        return fromString<unsigned int>(*this);
    }

    long String::toLong() const
    {
        return fromString<long>(*this);
    }

    unsigned long String::toULong() const
    {
        return fromString<unsigned long>(*this);
    }

    float String::toFloat() const
    {
        return fromString<float>(*this);
    }

    double String::toDouble() const
    {
        return fromString<double>(*this);
    }

    bool String::toBool() const
    {
        return fromString<bool>(*this);
    }

    String::operator const char*() const
    {
        return this->c_str();
    }

    String::operator int() const
    {
        return fromString<int>(*this);
    }

    String::operator unsigned int() const
    {
        return fromString<unsigned int>(*this);
    }

    String::operator long() const
    {
        return fromString<long>(*this);
    }

    String::operator unsigned long() const
    {
        return fromString<unsigned long>(*this);
    }

    String::operator float() const
    {
        return fromString<float>(*this);
    }

    String::operator double() const
    {
        return fromString<double>(*this);
    }

    String::operator bool() const
    {
        return fromString<bool>(*this);
    }

    String& String::operator=(char c_)
    {
        this->assign(1, c_);
        return *this;
    }

    std::istream& safeGetline(std::istream& is_, String& str_)
    {
        str_.clear();

        // The characters in the stream are read one-by-one using a std::streambuf.
        // That is faster than reading them one-by-one using the std::istream.
        // Code that uses streambuf this way must be guarded by a sentry object.
        // The sentry object performs various tasks,
        // such as thread synchronization and updating the stream state.

        std::istream::sentry se(is_, true);
        std::streambuf* sb = is_.rdbuf();

        while(1)
        {
            int c = sb->sbumpc();
            switch(c)
            {
                case '\r':
                    c = sb->sgetc();
                    if(c == '\n')
                        sb->sbumpc();
                    return is_;
                case '\n':
                    return is_;
                case EOF:
                    is_.setstate(std::ios_base::failbit|std::ios_base::eofbit);
                    return is_;
                default:
                    str_ += String(1, (char)c);
            }
        }
    }
} // namespace LibUtil

