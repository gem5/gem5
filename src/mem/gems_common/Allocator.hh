/*
 * Copyright (c) 1999 by Mark Hill and David Wood for the Wisconsin
 * Multifacet Project.  ALL RIGHTS RESERVED.
 *
 * ##HEADER##
 *
 * This software is furnished under a license and may be used and
 * copied only in accordance with the terms of such license and the
 * inclusion of the above copyright notice.  This software or any
 * other copies thereof or any derivative works may not be provided or
 * otherwise made available to any other persons.  Title to and
 * ownership of the software is retained by Mark Hill and David Wood.
 * Any use of this software must include the above copyright notice.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS".  THE LICENSOR MAKES NO
 * WARRANTIES ABOUT ITS CORRECTNESS OR PERFORMANCE.
 * */

/*
 * $Id$
 */

#ifndef ALLOCATOR_H
#define ALLOCATOR_H

#include "Vector.hh"

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
  DEBUG_EXPR(ALLOCATOR_COMP, LowPrio, m_counter);
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
