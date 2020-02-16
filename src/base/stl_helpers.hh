/*
 * Copyright (c) 2010 The Hewlett-Packard Development Company
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
 */

#ifndef __BASE_STL_HELPERS_HH__
#define __BASE_STL_HELPERS_HH__

#include <algorithm>
#include <iostream>

namespace m5 {
namespace stl_helpers {

template <typename T>
void
deletePointer(T &ptr)
{
    delete ptr;
    ptr = NULL;
}

template <class T>
class ContainerPrint
{
  private:
    std::ostream &out;
    bool first;

  public:
    ContainerPrint(std::ostream &out)
        : out(out), first(true)
    {}

    void
    operator()(const T &elem)
    {
        // First one doesn't get a space before it.  The rest do.
        if (first)
            first = false;
        else
            out << " ";

        out << elem;
    }
};

// Treat all objects in an stl container as pointers to heap objects,
// calling delete on each one and zeroing the pointers along the way
template <template <typename T, typename A> class C, typename T, typename A>
void
deletePointers(C<T,A> &container)
{
    std::for_each(container.begin(), container.end(), deletePointer<T>);
}

// Write out all elements in an stl container as a space separated
// list enclosed in square brackets
template <template <typename T, typename A> class C, typename T, typename A>
std::ostream &
operator<<(std::ostream& out, const C<T,A> &vec)
{
    out << "[ ";
    std::for_each(vec.begin(), vec.end(), ContainerPrint<T>(out));
    out << " ]";
    out << std::flush;
    return out;
}

} // namespace stl_helpers
} // namespace m5

#endif // __BASE_STL_HELPERS_HH__
