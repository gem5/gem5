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

