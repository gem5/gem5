/*
 * Copyright (c) 1999-2005 Mark D. Hill and David A. Wood
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

/*
 * $Id$
 *
 */

#ifndef MAP_H
#define MAP_H

#include <cassert>
#include <iostream>
#include <vector>

#include "base/hashmap.hh"

template <class KEY_TYPE, class VALUE_TYPE>
class Map
{
  private:
    typedef typename m5::hash_map<KEY_TYPE, VALUE_TYPE> MapType;
    typedef typename MapType::iterator iterator;
    typedef typename MapType::const_iterator const_iterator;

public:
  Map() { /* empty */ }
  ~Map() { /* empty */ }

  void add(const KEY_TYPE& key, const VALUE_TYPE& value);
  bool exist(const KEY_TYPE& key) const;
  int size() const { return m_map.size(); }
  void erase(const KEY_TYPE& key) { assert(exist(key)); m_map.erase(key); }
  std::vector<KEY_TYPE> keys() const;
  std::vector<VALUE_TYPE> values() const;
  void deleteKeys();
  void deleteValues();
  VALUE_TYPE& lookup(const KEY_TYPE& key) const;
  void clear() { m_map.clear(); }
    void print(std::ostream& out) const;

  // Synonyms
  void remove(const KEY_TYPE& key) { erase(key); }
  void deallocate(const KEY_TYPE& key) { erase(key); }
  void allocate(const KEY_TYPE& key) { add(key, VALUE_TYPE()); }
  void insert(const KEY_TYPE& key, const VALUE_TYPE& value) { add(key, value); }

  // Use default copy constructor and assignment operator
private:
  // Data members

  // m_map is declared mutable because some methods from the STL "map"
  // class that should be const are not.  Thus we define this as
  // mutable so we can still have conceptually const accessors.
  mutable m5::hash_map<KEY_TYPE, VALUE_TYPE> m_map;
};

template <class KEY_TYPE, class VALUE_TYPE>
std::ostream&
operator<<(std::ostream& out, const Map<KEY_TYPE, VALUE_TYPE>& map);

// *********************

template <class KEY_TYPE, class VALUE_TYPE>
void Map<KEY_TYPE, VALUE_TYPE>::add(const KEY_TYPE& key, const VALUE_TYPE& value)
{
  // Update or add a new key/value pair
  m_map[key] = value;
}

template <class KEY_TYPE, class VALUE_TYPE>
bool Map<KEY_TYPE, VALUE_TYPE>::exist(const KEY_TYPE& key) const
{
  return (m_map.count(key) != 0);
}

template <class KEY_TYPE, class VALUE_TYPE>
VALUE_TYPE& Map<KEY_TYPE, VALUE_TYPE>::lookup(const KEY_TYPE& key) const
{
  if (!exist(key))
    std::cerr << *this << " is looking for " << key << std::endl;
  assert(exist(key));
  return m_map[key];
}

template <class KEY_TYPE, class VALUE_TYPE>
std::vector<KEY_TYPE> Map<KEY_TYPE, VALUE_TYPE>::keys() const
{
  std::vector<KEY_TYPE> keys(m_map.size());
  const_iterator iter = m_map.begin();
  for (int i = 0; i < m_map.size(); ++i) {
      keys[i] = iter->first;
      ++iter;
  }
  assert(iter == m_map.end());
  return keys;
}

template <class KEY_TYPE, class VALUE_TYPE>
std::vector<VALUE_TYPE> Map<KEY_TYPE, VALUE_TYPE>::values() const
{
  std::vector<VALUE_TYPE> values(m_map.size());
  const_iterator iter = m_map.begin();

  for (int i = 0; i < m_map.size(); ++i) {
      values[i] = iter->second;
      ++iter;
  }
  assert(iter == m_map.end());
  return values;
}

template <class KEY_TYPE, class VALUE_TYPE>
void Map<KEY_TYPE, VALUE_TYPE>::deleteKeys()
{
  const_iterator iter;
  std::pair<KEY_TYPE, VALUE_TYPE> p;

  for (iter = m_map.begin(); iter != m_map.end(); iter++) {
    p = *iter;
    delete p.first;
  }
}

template <class KEY_TYPE, class VALUE_TYPE>
void Map<KEY_TYPE, VALUE_TYPE>::deleteValues()
{
  const_iterator iter;
  std::pair<KEY_TYPE, VALUE_TYPE> p;

  for (iter = m_map.begin(); iter != m_map.end(); iter++) {
    p = *iter;
    delete p.second;
  }
}

template <class KEY_TYPE, class VALUE_TYPE>
void Map<KEY_TYPE, VALUE_TYPE>::print(std::ostream& out) const
{
  const_iterator iter;
  std::pair<KEY_TYPE, VALUE_TYPE> p;

  out << "[";
  for (iter = m_map.begin(); iter != m_map.end(); iter++) {
    // unparse each basic block
    p = *iter;
    out << " " << p.first << "=" << p.second;
  }
  out << " ]";
}

template <class KEY_TYPE, class VALUE_TYPE>
std::ostream&
operator<<(std::ostream& out, const Map<KEY_TYPE, VALUE_TYPE>& map)
{
  map.print(out);
  return out;
}

#endif //MAP_H
