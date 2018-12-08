/*****************************************************************************

  Licensed to Accellera Systems Initiative Inc. (Accellera) under one or
  more contributor license agreements.  See the NOTICE file distributed
  with this work for additional information regarding copyright ownership.
  Accellera licenses this file to you under the Apache License, Version 2.0
  (the "License"); you may not use this file except in compliance with the
  License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
  implied.  See the License for the specific language governing
  permissions and limitations under the License.

 *****************************************************************************/

//
// To the LRM writer : this class is purely an artifact of the implementation.
//

#ifndef __CIRCULAR_BUFFER_H__
#define __CIRCULAR_BUFFER_H__

#include <iostream>

namespace tlm {

template < typename T >
class circular_buffer
{
public:

  explicit
  circular_buffer( int size = 0 );
  ~circular_buffer();

  void resize( int size );
  void clear();

  T read();
  void write( const T & );

  bool is_empty() const { return used() == 0; }
  bool is_full() const { return free() == 0; }

  int size() const { return m_size; }
  int used() const { return m_used; }
  int free() const { return m_free; }

  const T& read_data() const
    { return buf_read( m_buf, m_ri ); }

  const T& peek_data( int i ) const
    { return buf_read( m_buf, (m_ri + i) % size() ); }

  T & poke_data( int i )
    { return buf_read( m_buf , (m_wi + i) % size() ); }

  void debug() const;

private:
  void increment_write_pos( int i = 1 );
  void increment_read_pos( int i = 1 );

  void init();

  circular_buffer( const circular_buffer<T> &b );              // disabled
  circular_buffer<T> &operator=( const circular_buffer<T> & ); // disabled

  void* buf_alloc( int size );
  void  buf_free( void*& buf );
  void  buf_write( void* buf, int n, const T & t );
  T&    buf_read( void* buf, int n ) const;
  void  buf_clear( void* buf, int n );

private:
  int    m_size;                   // size of the buffer
  void*  m_buf;                    // the buffer
  int    m_free;                   // number of free spaces
  int    m_used;                   // number of used spaces
  int    m_ri;                     // index of next read
  int    m_wi;                     // index of next write

};

template< typename T >
void
circular_buffer<T>::debug() const
{

  std::cout << "Buffer debug" << std::endl;
  std::cout << "Size : " << size() << std::endl;
  std::cout << "Free/Used " << free() << "/" << used() << std::endl;
  std::cout << "Indices : r/w = " << m_ri << "/" << m_wi << std::endl;

  if( is_empty() ) {

    std::cout << "empty" << std::endl;

  }

  if( is_full() ) {

    std::cout << "full" << std::endl;

  }

  std::cout << "Data : " << std::endl;
  for( int i = 0; i < used(); i++ ) {

    std::cout << peek_data( i ) << std::endl;

  }


}

template < typename T >
circular_buffer<T>::circular_buffer( int size )
  : m_size(size)
  , m_buf(0)
{
  init();

}

template < typename T >
void
circular_buffer<T>::clear()
{
  for( int i=0; i < used(); i++ ) {
    buf_clear( m_buf, (m_ri + i) % m_size );
  }
  m_free = m_size;
  m_used = m_ri = m_wi = 0;
}

template < typename T >
circular_buffer<T>::~circular_buffer()
{
  clear();
  buf_free( m_buf );
}

template < typename T >
void
circular_buffer<T>::resize( int size )
{

  int i;
  void * new_buf = buf_alloc(size);

  for( i = 0; i < size && i < used(); i++ ) {

    buf_write( new_buf, i, peek_data( i ) );
    buf_clear( m_buf, (m_ri + i) % m_size );

  }

  buf_free( m_buf );

  m_size = size;
  m_ri   = 0;
  m_wi   = i % m_size;
  m_used = i;
  m_free = m_size - m_used;

  m_buf  = new_buf;
}


template < typename T >
void
circular_buffer<T>::init() {

  if( m_size > 0 ) {
    m_buf = buf_alloc( m_size );
  }

  m_free = m_size;
  m_used = 0;
  m_ri = 0;
  m_wi = 0;

}

template < typename T >
T
circular_buffer<T>::read()
{
  T t = read_data();

  buf_clear( m_buf, m_ri );
  increment_read_pos();

  return t;
}

template < typename T >
void
circular_buffer<T>::write( const T &t )
{
  buf_write( m_buf, m_wi, t );
  increment_write_pos();
}


template < typename T >
void
circular_buffer<T>::increment_write_pos( int i ) {

  m_wi = ( m_wi + i ) % m_size;
  m_used += i;
  m_free -= i;

}

template < typename T >
void
circular_buffer<T>::increment_read_pos( int i ) {

  m_ri = ( m_ri + i ) % m_size;
  m_used -= i;
  m_free += i;

}

template < typename T >
inline void*
circular_buffer<T>::buf_alloc( int size )
    { return new unsigned char[ size * sizeof(T) ]; }

template < typename T >
inline void
circular_buffer<T>::buf_free( void* & buf )
    { delete [] static_cast<unsigned char*>(buf); buf = 0; }

template < typename T >
inline void
circular_buffer<T>::buf_write( void* buf, int n, const T & t )
{
  T* p = static_cast<T*>(buf) + n;
  new (p) T(t);
}

template < typename T >
inline T&
circular_buffer<T>::buf_read( void* buf, int n ) const
{
  T* p = static_cast<T*>(buf) + n;
  return *p;
}

template < typename T >
inline void
circular_buffer<T>::buf_clear( void* buf, int n )
{
  T* p = static_cast<T*>(buf) + n;
  p->~T();
}

} // namespace tlm

#endif

