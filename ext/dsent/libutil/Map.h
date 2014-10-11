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

#ifndef __MAP_H__
#define __MAP_H__

#include <iostream>
#include <map>

#include "String.h"
#include "Assert.h"

namespace LibUtil
{
    using std::map;

    template<class T> class Map
    {
        public:
            typedef typename map<String, T>::iterator       Iterator;
            typedef typename map<String, T>::const_iterator ConstIterator;
            typedef typename map<String, T>::size_type      SizeType;

        public:
            Map();
            virtual ~Map();

        public:
            // Return a new copy of this Map instance
            Map* clone() const;
            // Copy map_ to this instance
            void copyFrom(const Map<T>* map_);
            // Return the size of the map
            SizeType size() const;
            // Check if the map is empty
            bool isEmpty() const;
            // Check if the key exists
            bool keyExist(const String& key_) const;
            // Get the value_ corresponding to the key_
            const T& get(const String& key_) const;
            // Get the value_ corresponding to the key_ if the key_ exist, otherwise, the default_value_is returned
            const T& getIfKeyExist(const String& key_, const T& default_value_ = T()) const;
            // Add/Update a <key_, value_> entry
            void set(const String& key_, const T& value_);
            // Get iterator to the element
            Iterator find(const String& key_);
            ConstIterator find(const String& key_) const;
            // Remove an entry corresponding to key_
            void remove(const String& key_);
            // Remove an entry at 'it' 
            void remove(Iterator it);
            // Remove all keys
            void clear();
            // Merge a map. Values with same key will be overwritten.
            void merge(const Map<T>* map_);
            // Returns a MapIterator referring to the first element in the map
            Iterator begin();
            ConstIterator begin() const;
            // Returns a MapIterator referring to the past-the-end element in the map
            Iterator end();
            ConstIterator end() const;

        protected:
            Map(const Map& map_);

        protected:
            map<String, T> mMap;
    };

    template<class T> Map<T>::Map()
    {}

    template<class T> Map<T>::~Map()
    {}

    template<class T> Map<T>* Map<T>::clone() const
    {
        return new Map<T>(*this);
    }

    template<class T> void Map<T>::copyFrom(const Map<T>* map_)
    {
        // Remove all keys (it won't free the content if T is a pointer)
        mMap.clear();

        // Copy the contents
        mMap = map_->mMap;
    }

    template<class T> typename Map<T>::SizeType Map<T>::size() const
    {
        return mMap.size();
    }

    template<class T> bool Map<T>::isEmpty() const
    {
        return (mMap.empty());
    }

    template<class T> bool Map<T>::keyExist(const String& key_) const
    {
        ConstIterator it = mMap.find(key_);
        return (it != mMap.end());
    }

    template<class T> const T& Map<T>::get(const String& key_) const
    {
        ConstIterator it;

        it = mMap.find(key_);
        ASSERT((it != mMap.end()), "Key not found: " + key_);
        return (it->second);
    }

    template<class T> const T& Map<T>::getIfKeyExist(const String& key_, const T& default_value_) const
    {
        if(keyExist(key_))
        {
            return get(key_);
        }
        else
        {
            return default_value_;
        }
    }

    template<class T> void Map<T>::set(const String& key_, const T& value_)
    {
        mMap[key_] = value_;
        return;
    }

    template<class T> typename Map<T>::Iterator Map<T>::find(const String& key_)
    {
        return mMap.find(key_);
    }

    template<class T> typename Map<T>::ConstIterator Map<T>::find(const String& key_) const
    {
        return mMap.find(key_);
    }

    template<class T> void Map<T>::remove(const String& key_)
    {
        mMap.erase(key_);
        return;
    }

    template<class T> void Map<T>::remove(Iterator it)
    {
        mMap.erase(it);
        return;
    }

    template<class T> void Map<T>::clear()
    {
        mMap.clear();
        return;
    }

    template<class T> void Map<T>::merge(const Map<T>* map_)
    {
        ConstIterator it;
        for(it = map_->begin(); it != map_->end(); it++)
        {
            const String& key = it->first;
            const T& value = it->second;
            set(key, value);
        }
        return;
    }

    template<class T> typename Map<T>::Iterator Map<T>::begin()
    {
        return mMap.begin();
    }

    template<class T> typename Map<T>::ConstIterator Map<T>::begin() const
    {
        return mMap.begin();
    }

    template<class T> typename Map<T>::Iterator Map<T>::end()
    {
        return mMap.end();
    }

    template<class T> typename Map<T>::ConstIterator Map<T>::end() const
    {
        return mMap.end();
    }

    inline std::ostream& operator<<(std::ostream& ost_, const Map<String>& map_)
    {
        Map<String>::ConstIterator it;
        for(it = map_.begin(); it != map_.end(); it++)
        {
            ost_ << it->first << " = " << it->second << std::endl;
        }
        return ost_;
    }

    template<class T> Map<T>::Map(const Map<T>& map_)
        : mMap(map_.mMap)
    {}

    typedef Map<String> StringMap;


    // Handy function to delete all pointers in a map
    template<class T> void clearPtrMap(Map<T*>* map_)
    {
        for(typename Map<T*>::Iterator it = map_->begin(); it != map_->end(); ++it)
        {
            T* temp_T = it->second;
            delete temp_T;
        }
        map_->clear();
        return;
    }

    // Handy function to delete all pointers in a map and the map itself
    template<class T> void deletePtrMap(Map<T*>* map_)
    {
        clearPtrMap<T>(map_);
        delete map_;
        return;
    }

    // Handy function to clone all pointers in a map
    template<class T> Map<T*>* clonePtrMap(const Map<T*>* map_)
    {
        Map<T*>* new_T_map = new Map<T*>;
        for(typename Map<T*>::ConstIterator it = map_->begin(); it != map_->end(); ++it)
        {
            const String& temp_name = it->first;
            const T* temp_T = it->second;
            new_T_map->set(temp_name, temp_T->clone());
        }
        return new_T_map;
    }
}

#endif // __MAP_H__

