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

/*****************************************************************************

  scfx_string.h - 

  Original Author: Robert Graulich, Synopsys, Inc.
                   Martin Janssen,  Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/
// $Log: scfx_string.h,v $
// Revision 1.1.1.1  2006/12/15 20:20:04  acg
// SystemC 2.3
//
// Revision 1.2  2006/01/03 23:18:34  acg
// Changed copyright to include 2006.
//
// Revision 1.1.1.1  2005/12/19 23:16:43  acg
// First check in of SystemC 2.1 into its own archive.
//
// Revision 1.9  2005/09/15 23:02:03  acg
// Added std:: prefix to appropriate methods and types to get around
// issues with the Edison Front End.
//
// Revision 1.8  2005/06/07 17:27:02  acg
// Fixed bug in scfx_string::operator += where an array reference was used
// rather than the [] operator.  This meant that the buffer may have been
// accessed beyond its allocated storage.
//

#ifndef SCFX_STRING_H
#define SCFX_STRING_H

#include <cstdio>


namespace sc_dt
{

// classes defined in this module
class scfx_string;


// ----------------------------------------------------------------------------
//  CLASS : scfx_string
//
//  Simple string class for internal use.
// ----------------------------------------------------------------------------

class scfx_string
{
    void resize( std::size_t );

public:

    scfx_string();

    ~scfx_string();

    int length() const;

    void clear();

    char& operator [] ( int );

    void append( int );
    void discard( int );
    void remove( int );

    void operator += ( char );
    void operator += ( const char* );

    operator const char* ();

private:

    std::size_t m_len;
    std::size_t m_alloc;
    char*  m_buffer;
};


// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII

inline
void
scfx_string::resize( std::size_t i )
{
    do {
	m_alloc *= 2;
    } while( i >= m_alloc );

    char* temp = new char[m_alloc];

    for( int j = 0; j < (int) m_len; ++ j ) {
	temp[j] = m_buffer[j];
    }
    temp[m_len] = 0;

    delete [] m_buffer;
    m_buffer = temp;
}


inline
scfx_string::scfx_string()
: m_len( 0 ), m_alloc( BUFSIZ ), m_buffer( new char[m_alloc] )
{
    m_buffer[m_len] = 0;
}


inline
scfx_string::~scfx_string()
{
    delete [] m_buffer;
}


inline
int
scfx_string::length() const
{
    return m_len;
}


inline
void
scfx_string::clear()
{
    m_len = 0;
    m_buffer[m_len] = 0;
}


inline
char&
scfx_string::operator [] ( int i )
{
    if( i >= (int) m_alloc ) {
	resize( i );
    }
    return m_buffer[i];
}


inline
void
scfx_string::append( int n )
{
    m_len += n;
    m_buffer[m_len] = 0;
}

inline
void
scfx_string::discard( int n )
{
    m_len -= n;
    m_buffer[m_len] = 0;
}

inline
void
scfx_string::remove( int i )
{
    for( int j = i + 1; j < (int) m_len; ++ j )
	m_buffer[j - 1] = m_buffer[j];
    -- m_len;
    m_buffer[m_len] = 0;
}


inline
void
scfx_string::operator += ( char c )
{
    this->operator [] ( m_len ) = c;
    m_len ++;
    this->operator [] ( m_len ) = 0;
}

inline
void
scfx_string::operator += ( const char* s )
{
    while( *s )
	(*this) += *s ++;
}


inline
scfx_string::operator const char*()
{
    m_buffer[m_len] = 0;
    return m_buffer;
}

} // namespace sc_dt


#endif

// Taf!
