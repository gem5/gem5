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

#ifndef __LOG_H__
#define __LOG_H__

#include <cstdio>
#include <iostream>
#include <fstream>

#include "String.h"

#ifndef LIBUTIL_IS_LOG
#define LIBUTIL_IS_LOG false
#endif

namespace LibUtil
{
    using std::cerr;

    class Log
    {
        public:
            static void allocate(const String& log_file_name_);
            static void release();

            static void print(const String& str_);
            static void print(std::ostream& stream_, const String& str_);
            static void printLine(const String& str_);
            static void printLine(std::ostream& stream_, const String& str_);

        protected:
            static Log* msSingleton;
            static const bool msIsLog;

        protected:
            Log(const String& log_file_name_);
            ~Log();

        protected:
            std::ofstream ofs;
    };
}

#endif // __LOG_H__

