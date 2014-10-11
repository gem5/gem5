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

#include "Log.h"

#include "Assert.h"

namespace LibUtil
{
    using std::ostream;
    using std::endl;

    Log* Log::msSingleton = NULL;
    const bool Log::msIsLog = LIBUTIL_IS_LOG;

    void Log::allocate(const String& log_file_name_)
    {
        if(msIsLog)
        {
            // Allocate static Config instance
            ASSERT(!msSingleton, "Log singleton is allocated");
            msSingleton = new Log(log_file_name_);
        }
    }

    void Log::release()
    {
        if(msIsLog)
        {
            ASSERT(msSingleton, "Log singleton is not allocated");
            delete msSingleton;
            msSingleton = NULL;
        }
        return;
    }

    void Log::print(const String& str_)
    {
        if(msIsLog)
        {
            ASSERT(msSingleton, "Log singleton is not allocated");
            msSingleton->ofs << str_;
        }
        return;
    }

    void Log::print(ostream& stream_, const String& str_)
    {
        if(msIsLog)
        {
            ASSERT(msSingleton, "Log singleton is not allocated");
            msSingleton->ofs << str_;
        }
        stream_ << str_;
        return;
    }

    void Log::printLine(const String& str_)
    {
        if(msIsLog)
        {
            ASSERT(msSingleton, "Log singleton is not allocated");
            msSingleton->ofs << str_ << endl;
        }
        return;
    }

    void Log::printLine(ostream& stream_, const String& str_)
    {
        if(msIsLog)
        {
            ASSERT(msSingleton, "Log singleton is not allocated");
            msSingleton->ofs << str_ << endl;
        }
        stream_ << str_ << endl;
        return;
    }

    Log::Log(const String& log_file_name_)
    {
        ofs.open(log_file_name_.c_str());
    }

    Log::~Log()
    {
        ofs.close();
    }
}

