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
 * Description: The Vector class is a generic container which acts
 * much like an array.  The Vector class handles dynamic sizing and
 * resizing as well as performing bounds checking on each access.  An
 * "insertAtBottom" operation is supported to allow adding elements to
 * the Vector much like you would add new elements to a linked list or
 * queue.
 */

#ifndef VECTOR_H
#define VECTOR_H

#include "std-includes.hh"

template <class TYPE>
class Vector
{
public:
  Vector();
  explicit Vector(int initial_size);  // Construct with an initial max size
  ~Vector();
  const TYPE& ref(int index) const;   // Get an element of the vector
  TYPE& ref(int index);               // Get an element of the vector
  void clear();                       // remove all elements of the vector
  void sortVector();                  // sort all elements using < operator
  int size() const { return m_size; }
  void setSize(int new_size);         // Increase size, reallocates memory as needed
  void expand(int num) { setSize(m_size+num); } // Increase size by num
  void increaseSize(int new_size, const TYPE& reset);    // and adds num of slots at the bottom set to reset value
  void insertAtTop(const TYPE& element);  // Increase size by one and set last element
    // FIXME - WARNING: insertAtTop is currently O(n) and needs to be fixed
  void insertAtBottom(const TYPE& element);  // Increase size by one and set last element
  TYPE sum() const;  // Uses the += operator to sum all the elements of the vector
  void deletePointers(); // Walks the Vector calling delete on all
                         // elements and sets them to NULL, can only
                         // be used when the TYPE is a pointer type.
  void removeFromTop(int num);  // removes elements from top
  void print(ostream& out) const;


  // Array Reference operator overloading
  const TYPE& operator[](int index) const { return ref(index); }
  TYPE& operator[](int index) { return ref(index); }

  // Public copy constructor and assignment operator
  Vector(const Vector& vec);
  Vector<TYPE>& operator=(const Vector& vec);
private:

  void grow(int new_max_size);  // Expands vector to new_max_size

  // Data members
  TYPE* m_vec;           // Array to hold the elements
  int m_size;            // Number of elements in use
  int m_max_size;        // Size of allocated array
};

template <class TYPE>
ostream& operator<<(ostream& out, const Vector<TYPE>& vec);

// *********************

template <class TYPE>
Vector<TYPE>::Vector()
{
  m_size = 0;
  m_max_size = 0;
  m_vec = NULL;
}

template <class TYPE>
Vector<TYPE>::Vector(int initial_size)
{
  m_size = 0;
  m_max_size = initial_size;
  m_vec = NULL;
  grow(initial_size);
}

template <class TYPE>
Vector<TYPE>::~Vector()
{
  delete [] m_vec;
}

template <class TYPE>
const TYPE& Vector<TYPE>::ref(int index) const
{
#ifndef NO_VECTOR_BOUNDS_CHECKS
  assert(m_size != 0);
  assert(index < m_size);
  assert(index >= 0);
#endif
  return m_vec[index];
}

template <class TYPE>
TYPE& Vector<TYPE>::ref(int index)
{
#ifndef NO_VECTOR_BOUNDS_CHECKS
  assert(m_size != 0);
  assert(index < m_size);
  assert(index >= 0);
#endif
  return m_vec[index];
}


template <class TYPE>
void Vector<TYPE>::setSize(int new_size)
{
  // FIXME - this should also decrease or shrink the size of the array at some point.
  if (new_size > m_max_size) {
    grow(max((m_max_size+1)*2, new_size));
  }
  m_size = new_size;
#ifndef NO_VECTOR_BOUNDS_CHECKS
  assert(m_size <= m_max_size);
  assert(m_size >= 0);
#endif
}

template <class TYPE>
inline
void Vector<TYPE>::increaseSize(int new_size, const TYPE& reset)
{
  assert(new_size >= m_size);
  if (new_size >= m_max_size) {
    grow(max((m_max_size+1)*2, new_size));
  }
  int old_size = m_size;
  m_size = new_size;
  for (int j = old_size; j < m_size; j++) {
    ref(j) = reset;
  }

#ifndef NO_VECTOR_BOUNDS_CHECKS
    assert(m_size <= m_max_size);
    assert(m_size >= 0);
#endif
}

template <class TYPE>
inline
void Vector<TYPE>::clear()
{
  m_size = 0;
  m_max_size = 0;
  delete [] m_vec;
  m_vec = NULL;
}

template <class TYPE>
inline
void Vector<TYPE>::sortVector()
{
  sort(&m_vec[0], &m_vec[m_size]);
}

template <class TYPE>
inline
void Vector<TYPE>::insertAtTop(const TYPE& element)
{
  setSize(m_size+1);
  for (int i = m_size-1; i >= 1; i--) {
    ref(i) = ref(i-1);
  }
  ref(0) = element;
}

template <class TYPE>
inline
void Vector<TYPE>::removeFromTop(int num)
{
  if (num > m_size) {
    num = m_size;
  }
  for (int i = 0; i < m_size - num; i++) {
    m_vec[i] = m_vec[i+num];
  }
  m_size = m_size - num;

}

template <class TYPE>
void Vector<TYPE>::insertAtBottom(const TYPE& element)
{
  setSize(m_size+1);
  ref(m_size-1) = element;
}

template <class TYPE>
TYPE Vector<TYPE>::sum() const
{
  assert(m_size > 0);
  TYPE sum = ref(0);
  for(int i=1; i<m_size; i++) {
    sum += ref(i);
  }
  return sum;
}

template <class TYPE>
void Vector<TYPE>::deletePointers()
{
  assert(m_size >= 0);
  for(int i=0; i<m_size; i++) {
    // FIXME this function should be non-member function, otherwise this
    // prevent template instantiation for non-pointer types
    //
    // Also, there is warning of Switch.cc which use void* here
    delete ref(i);
    ref(i) = NULL;
  }
}

template <class TYPE>
void Vector<TYPE>::print(ostream& out) const
{
  out << "[ ";
  for(int i=0; i<m_size; i++) {
    if (i != 0) {
      out << " ";
    }
    out << ref(i);
  }
  out << " ]";
  out << flush;
}

// Copy constructor
template <class TYPE>
Vector<TYPE>::Vector(const Vector& vec)
{
  // Setup the new memory
  m_size = vec.m_size;
  m_max_size = vec.m_max_size;
  if (m_max_size != 0) {
    m_vec = new TYPE[m_max_size];
    assert(m_vec != NULL);
  } else {
    m_vec = NULL;
  }

  // Copy the elements of the array
  for(int i=0; i<m_size; i++) {
    m_vec[i] = vec.m_vec[i];  // Element copy
  }
}

template <class TYPE>
Vector<TYPE>& Vector<TYPE>::operator=(const Vector& vec)
{
  if (this == &vec) {
    //    assert(0);
  } else {
    // Free the old memory
    delete [] m_vec;

    // Setup the new memory
    m_size = vec.m_size;
    m_max_size = vec.m_max_size;

    if (m_max_size != 0) {
      m_vec = new TYPE[m_max_size];
      assert(m_vec != NULL);
    } else {
      m_vec = NULL;
    }

    // Copy the elements of the array
    for(int i=0; i<m_size; i++) {
      m_vec[i] = vec.m_vec[i];  // Element copy
    }
  }
  return *this;
}

template <class TYPE>
void Vector<TYPE>::grow(int new_max_size)
{
  TYPE* temp_vec;
  m_max_size = new_max_size;
  if (new_max_size != 0) {
    temp_vec = new TYPE[new_max_size];
    assert(temp_vec != NULL);
  } else {
    temp_vec = NULL;
  }

  // Copy the elements of the array
  for(int i=0; i<m_size; i++) {
    temp_vec[i] = m_vec[i];  // Element copy
  }
  delete [] m_vec;
  m_vec = temp_vec;
}

template <class TYPE>
ostream& operator<<(ostream& out, const Vector<TYPE>& vec)
{
  vec.print(out);
  return out;
}

#endif //VECTOR_H
