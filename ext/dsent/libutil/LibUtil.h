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

