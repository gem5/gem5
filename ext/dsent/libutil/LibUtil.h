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

#ifndef __LIBUTIL_H__
#define __LIBUTIL_H__

#include <vector>

#include "String.h"
#include "Exception.h"
#include "Assert.h"
#include "Map.h"
#include "Log.h"
#include "Config.h"
#include "MathUtil.h"

namespace LibUtil
{
    template<class T> void clearPtrVector(std::vector<T*>* vec_)
    {
        for(typename std::vector<T*>::iterator it = vec_->begin(); it != vec_->end(); ++it)
        {
            T* temp_T = (*it);
            delete temp_T;
        }
        vec_->clear();
        return;
    }

    template<class T> void deletePtrVector(std::vector<T*>* vec_)
    {
        clearPtrVector<T>(vec_);
        delete vec_;
        return;
    }

} // namespace LibUtil

#endif // __LIBUTIL_H__

