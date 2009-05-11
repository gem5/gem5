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

#ifndef REFCNT_H
#define REFCNT_H

template <class TYPE>
class RefCnt {
public:
  // Constructors
  RefCnt();
  RefCnt(const TYPE& data);

  // Destructor
  ~RefCnt();

  // Public Methods
  const TYPE* ref() const { return m_data_ptr; }
  TYPE* ref() { return m_data_ptr; }
  TYPE* mod_ref() const { return m_data_ptr; }
  void freeRef();
  void print(ostream& out) const;

  // Public copy constructor and assignment operator
  RefCnt(const RefCnt& obj);
  RefCnt& operator=(const RefCnt& obj);

private:
  // Private Methods

  // Data Members (m_ prefix)
  TYPE* m_data_ptr;
  //  int* m_count_ptr; // Not used yet
};

// Output operator declaration
template <class TYPE>
inline
ostream& operator<<(ostream& out, const RefCnt<TYPE>& obj);

// ******************* Definitions *******************

// Constructors
template <class TYPE>
inline
RefCnt<TYPE>::RefCnt()
{
  m_data_ptr = NULL;
}

template <class TYPE>
inline
RefCnt<TYPE>::RefCnt(const TYPE& data)
{
  m_data_ptr = data.clone();
  m_data_ptr->setRefCnt(1);
}

template <class TYPE>
inline
RefCnt<TYPE>::~RefCnt()
{
  freeRef();
}

template <class TYPE>
inline
void RefCnt<TYPE>::freeRef()
{
  if (m_data_ptr != NULL) {
    m_data_ptr->decRefCnt();
    if (m_data_ptr->getRefCnt() == 0) {
      m_data_ptr->destroy();
    }
    m_data_ptr = NULL;
  }
}

template <class TYPE>
inline
void RefCnt<TYPE>::print(ostream& out) const
{
  if (m_data_ptr == NULL) {
    out << "[RefCnt: Null]";
  } else {
    out << "[RefCnt: ";
    m_data_ptr->print(out);
    out << "]";
  }
}

// Copy constructor
template <class TYPE>
inline
RefCnt<TYPE>::RefCnt(const RefCnt<TYPE>& obj)
{
  //  m_data_ptr = obj.m_data_ptr->clone();
  m_data_ptr = obj.m_data_ptr;

  // Increment the reference count
  if (m_data_ptr != NULL) {
    m_data_ptr->incRefCnt();
  }
}

// Assignment operator
template <class TYPE>
inline
RefCnt<TYPE>& RefCnt<TYPE>::operator=(const RefCnt<TYPE>& obj)
{
  if (this == &obj) {
    // If this is the case, do nothing
    //    assert(false);
  } else {
    freeRef();
    m_data_ptr = obj.m_data_ptr;
    if (m_data_ptr != NULL) {
      m_data_ptr->incRefCnt();
    }
  }
  return *this;
}


// Output operator definition
template <class TYPE>
inline
ostream& operator<<(ostream& out, const RefCnt<TYPE>& obj)
{
  obj.print(out);
  out << flush;
  return out;
}



#endif //REFCNT_H
