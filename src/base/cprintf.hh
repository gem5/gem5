/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Nathan Binkert
 *          Steve Reinhardt
 */

#ifndef __CPRINTF_HH__
#define __CPRINTF_HH__

#include <iostream>
#include <list>
#include <string>

#include "base/cprintf_formats.hh"

namespace cp {

class ArgList
{
  private:
    class Base
    {
      public:
        virtual ~Base() {}
        virtual void process(std::ostream &out, Format &fmt) = 0;
    };

    template <typename T>
    class Node : public Base
    {
      public:
        const T &data;

      public:
        Node(const T &d) : data(d) {}
        virtual void process(std::ostream &out, Format &fmt) {
            switch (fmt.format) {
              case Format::character:
                format_char(out, data, fmt);
                break;

              case Format::integer:
                format_integer(out, data, fmt);
                break;

              case Format::floating:
                format_float(out, data, fmt);
                break;

              case Format::string:
                format_string(out, data, fmt);
                break;

              default:
                out << "<bad format>";
                break;
            }
        }
    };

    typedef std::list<Base *> list_t;

  protected:
    list_t objects;
    std::ostream *stream;

  public:
    ArgList() : stream(&std::cout) {}
    ~ArgList();

    template<class T>
    void append(const T &data) {
        Base *obj = new ArgList::Node<T>(data);
        objects.push_back(obj);
    }

    template<class T>
    void prepend(const T &data) {
        Base *obj = new ArgList::Node<T>(data);
        objects.push_front(obj);
    }

    void dump(const std::string &format);
    void dump(std::ostream &strm, const std::string &fmt)
        { stream = &strm; dump(fmt); }

    std::string dumpToString(const std::string &format);

    friend ArgList &operator<<(std::ostream &str, ArgList &list);
};

template<class T>
inline ArgList &
operator,(ArgList &alist, const T &data)
{
    alist.append(data);
    return alist;
}

class ArgListNull {
};

inline ArgList &
operator,(ArgList &alist, ArgListNull)
{ return alist; }

//
// cprintf(format, args, ...) prints to cout
// (analogous to printf())
//
inline void
__cprintf(const std::string &format, ArgList &args)
{ args.dump(format); delete &args; }
#define __cprintf__(format, ...) \
    cp::__cprintf(format, (*(new cp::ArgList), __VA_ARGS__))
#define cprintf(...) \
    __cprintf__(__VA_ARGS__, cp::ArgListNull())

//
// ccprintf(stream, format, args, ...) prints to the specified stream
// (analogous to fprintf())
//
inline void
__ccprintf(std::ostream &stream, const std::string &format, ArgList &args)
{ args.dump(stream, format); delete &args; }
#define __ccprintf__(stream, format, ...) \
    cp::__ccprintf(stream, format, (*(new cp::ArgList), __VA_ARGS__))
#define ccprintf(stream, ...) \
    __ccprintf__(stream, __VA_ARGS__, cp::ArgListNull())

//
// csprintf(format, args, ...) returns a string
// (roughly analogous to sprintf())
//
inline std::string
__csprintf(const std::string &format, ArgList &args)
{ std::string s = args.dumpToString(format); delete &args; return s; }
#define __csprintf__(format, ...) \
    cp::__csprintf(format, (*(new cp::ArgList), __VA_ARGS__))
#define csprintf(...) \
    __csprintf__(__VA_ARGS__, cp::ArgListNull())

}

#endif // __CPRINTF_HH__
