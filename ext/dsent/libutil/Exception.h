#ifndef __EXCEPTION_H__
#define __EXCEPTION_H__

#include <exception>

#include "String.h"

namespace LibUtil
{
    using std::exception;

    // Exception class handles the all exception messages in the program
    class Exception : public exception
    {
        public:
            // All constructors/destructors/functions in this class don't throw any events
            Exception(const String& exception_msg_) throw();
            ~Exception() throw();

            // Derived from std::exception class that returns a null-terminated char string
            const char* what() const throw();

        private:
            String mExceptionMsg;
    };
}

#endif // __EXCEPTION_H__

