/*
 * Copyright (c) 1999 Mark D. Hill and David A. Wood
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

#ifndef ALLOCATOR_H
#define ALLOCATOR_H

#include "mem/gems_common/Vector.hh"

template <class TYPE>
class Allocator {
public:
  // Constructors
  Allocator() { m_counter = 0; }

  // Destructor
  ~Allocator() { for(int i=0; i<m_pool_vec.size(); i++) { delete m_pool_vec[i]; }}

  // Public Methods
  TYPE* allocate(const TYPE& obj);
  void deallocate(TYPE* obj_ptr);
private:
  // Private copy constructor and assignment operator
  Allocator(const Allocator& obj);
  Allocator& operator=(const Allocator& obj);

  // Private Methods

  // Data Members (m_ prefix)
  Vector<TYPE*> m_pool_vec;
  int m_counter;
};

template <class TYPE>
inline
TYPE* Allocator<TYPE>::allocate(const TYPE& obj)
{
  m_counter++;
  DPRINTF(GemsCommon, "couter %d", m_counter);
  TYPE* new_obj_ptr;

  // See if we need to allocate any new objects
  if (m_pool_vec.size() == 0) {
    // Allocate a new item
    m_pool_vec.insertAtBottom(new TYPE);
  }

  // Pop the pointer from the stack/pool
  int size = m_pool_vec.size();
  new_obj_ptr = m_pool_vec[size-1];
  m_pool_vec.setSize(size-1);

  // Copy the object
  *new_obj_ptr = obj;
  return new_obj_ptr;
}

template <class TYPE>
inline
void Allocator<TYPE>::deallocate(TYPE* obj)
{
  m_pool_vec.insertAtBottom(obj);
}

#endif //ALLOCATOR_H
