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

  sc_string.cpp -- Implementation of a simple string class.

  Original Author: Stan Y. Liao, Synopsys, Inc.

  CHANGE LOG AT END OF FILE
 *****************************************************************************/

#include <assert.h>
#include <ctype.h>
#include <cstdio>
#include <stdarg.h>
#include <string.h>

#include "sysc/utils/sc_iostream.h"
#include "sysc/utils/sc_string.h"
#include "sysc/utils/sc_utils_ids.h"

namespace sc_dt {

inline static int
sc_roundup( int n, int m )
{
    return ((n - 1) / m + 1) * m;
}


// ----------------------------------------------------------------------------
//  CLASS : sc_string_rep
//
//  Reference counting string implementation class.
// ----------------------------------------------------------------------------

class sc_string_rep
{
    friend class sc_string_old;
    friend ::std::ostream& operator<<( ::std::ostream&, const sc_string_old& );
    friend ::std::istream& operator>>( ::std::istream&, sc_string_old& );
    friend sc_string_old operator+( const char*, const sc_string_old& );

    sc_string_rep( int size = 16 ) :
        ref_count(1), alloc( sc_roundup( size, 16 ) ), str( new char[alloc] )
    {
        *str = '\0';
    }

    sc_string_rep( const char* s ) : ref_count(1), alloc(0), str(0)
    {
        if (s) {
            alloc = 1 + strlen(s);
            str = strcpy( new char[alloc], s );
        }
        else {
            alloc = 16;
            str = strcpy( new char[alloc], "" );
        }
    }

    sc_string_rep( const char* s, int n); // get first n chars from the string

    ~sc_string_rep()
    {
        assert( ref_count == 0 );
        delete[] str;
    }

    void resize( int new_size );
    void set_string( const char* s );

    int ref_count;
    int alloc;
    char* str;
};


// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII

sc_string_rep::sc_string_rep( const char* s, int n) :
    ref_count(1), alloc(0), str(0)
{
    if (s && n>0) {
        alloc = 1 + n;
        str = strncpy( new char[alloc], s,n );
        str[n] = 00;
    }
    else {
        alloc = 16;
        str = strcpy( new char[alloc], "" );
    }
}

void
sc_string_rep::resize( int new_size )
{
    if (new_size <= alloc) return;
    alloc = sc_roundup( new_size, 16 );
    char* new_str = strcpy( new char[alloc], str );
    delete[] str;
    str = new_str;
}

void
sc_string_rep::set_string( const char* s )
{
    int len = strlen(s);
    resize( len + 1 );
    strcpy( str, s );
}


// ----------------------------------------------------------------------------
//  CLASS : sc_string_old
//
//  String class (yet another).
// ----------------------------------------------------------------------------

// constructors

sc_string_old::sc_string_old( int size ) : rep( new sc_string_rep(size) )
{
}

sc_string_old::sc_string_old( const char* s ) : rep( new sc_string_rep(s) )
{
}

sc_string_old::sc_string_old( const char* s, int n ) : 
    rep( new sc_string_rep( s, n ) )
{
}

sc_string_old::sc_string_old( const sc_string_old& s ) : rep( s.rep )
{
    rep->ref_count ++;
}

sc_string_old::sc_string_old( sc_string_rep* r ) : rep(r)
{
}


// destructor

sc_string_old::~sc_string_old()
{
    if( -- (rep->ref_count) == 0 ) {
        delete rep;
    }
}


int
sc_string_old::length() const
{
    return strlen(rep->str);
}

sc_string_old
sc_string_old::operator+( const char* s ) const
{
    int len = length();
    sc_string_rep* r = new sc_string_rep( len + strlen(s) + 1 );
    strcpy( r->str, rep->str );
    strcpy( r->str + len, s );
    return sc_string_old(r);
}

sc_string_old sc_string_old::operator+(char c) const
{
    int len = length();
    sc_string_rep* r = new sc_string_rep( len + 2 );
    strcpy( r->str, rep->str );
    r->str[len] = c;
    r->str[len+1] = 00;
    return sc_string_old(r);
}

sc_string_old
operator+( const char* s, const sc_string_old& t )
{
    int len = strlen(s);
    sc_string_rep* r = new sc_string_rep( len + t.length() + 1 );
    strcpy( r->str, s );
    strcpy( r->str + len, t );
    return sc_string_old(r);
}

sc_string_old
sc_string_old::operator+( const sc_string_old& s ) const
{
    int len = length();
    sc_string_rep* r = new sc_string_rep( len + s.length() + 1 );
    strcpy( r->str, rep->str );
    strcpy( r->str + len, s.rep->str );
    return sc_string_old(r);
}

sc_string_old&
sc_string_old::operator=( const char* s )
{
    if (rep->ref_count > 1) {
        --rep->ref_count;
        rep = new sc_string_rep(s);
    }
    else {
        rep->set_string(s);
    }
    return *this;
}

sc_string_old&
sc_string_old::operator=( const sc_string_old& s )
{
    if (&s == this)
        return *this;
    if (--(rep->ref_count) == 0)
        delete rep;
    rep = s.rep;
    rep->ref_count++;
    return *this;
}

sc_string_old&
sc_string_old::operator+=( const char* s )
{
    int oldlen = length();
    int slen   = strlen(s);
    if (rep->ref_count > 1) {
        sc_string_rep* oldrep = rep;
        --rep->ref_count;
        rep = new sc_string_rep( oldlen + slen + 1 );
        strcpy( rep->str, oldrep->str );
        strcpy( rep->str + oldlen, s );
    }
    else {
        rep->resize( oldlen + slen + 1 );
        strcpy( rep->str + oldlen, s );
    }
    return *this;
}

sc_string_old& sc_string_old::operator+=(char c)
{
    int oldlen = length();
    if (rep->ref_count > 1) {
        sc_string_rep* oldrep = rep;
        --rep->ref_count;
        rep = new sc_string_rep( oldlen + 2 );
        strcpy( rep->str, oldrep->str );
        rep->str[oldlen]=c;
        rep->str[oldlen+1]=00;
    }
    else {
        rep->resize( oldlen + 2 );
        rep->str[oldlen]=c;
        rep->str[oldlen+1]=00;
    }
    return *this;
}

sc_string_old&
sc_string_old::operator+=( const sc_string_old& s )
{
    return this->operator+=( s.rep->str );
}

int
sc_string_old::cmp( const char* s ) const
{
    return strcmp( rep->str, s );
}

int
sc_string_old::cmp( const sc_string_old& s ) const
{
    return strcmp( rep->str, s.rep->str );
}

const char* sc_string_old::c_str() const
{
  return rep->str;
}

// get substring
sc_string_old sc_string_old::substr(int first,int last) const
{
  if(first<0 || last<0 || first>last || first>=length() || last>=length())
    return "";
  return sc_string_old(rep->str+first, last-first+1);
}


sc_string_old sc_string_old::make_str(long n) // convert integer to string
{
  char buf[32];
  ::std::sprintf(buf,"%ld",n);
  return sc_string_old(buf);
}


#define DEFINE_RELOP(op) \
bool sc_string_old::operator op( const char* s ) const \
{						\
    return strcmp( rep->str, s ) op 0;		\
}						\
bool sc_string_old::operator op( const sc_string_old& s ) const \
{						\
    return strcmp( rep->str, s.rep->str ) op 0;	\
}

DEFINE_RELOP(==)
DEFINE_RELOP(!=)
DEFINE_RELOP(<)
DEFINE_RELOP(<=)
DEFINE_RELOP(>)
DEFINE_RELOP(>=)

sc_string_old::operator const char*() const
{
    return rep->str;
}

char
sc_string_old::operator[]( int i ) const
{
    return rep->str[i];
}

char& sc_string_old::operator[]( int i )
{
    if (rep->ref_count > 1) {
        rep->ref_count--;
        rep = new sc_string_rep(rep->str);
    }
    return rep->str[i];
}

void
sc_string_old::set( int i, char c )
{
    if (rep->ref_count > 1) {
        rep->ref_count--;
        rep = new sc_string_rep(rep->str);
    }
    rep->str[i] = c;
}

#if defined(_MSC_VER)
   // Windows provides safer implementation
#  define sc_vsnprintf _vsnprintf
#else
#  define sc_vsnprintf vsnprintf
#endif

sc_string_old sc_string_old::to_string(const char* format, ...)
{
   va_list argptr;
   sc_string_old result;
   char buffer[1024]; // static string buffer
   buffer[1023]=000;

   va_start(argptr, format);
   int cnt = sc_vsnprintf(buffer, 1024, format, argptr);
   if(cnt>1023) // string too long
   {
     int buf_size = 1024;
     const int max_size = 65000;
     char* buf = 0; // dynamic string buffer
     do
     {
       delete[] buf;
       buf_size*=2;
       buf = new char[buf_size];
       cnt = sc_vsnprintf(buf, buf_size, format, argptr);
     }
     while( buf_size<max_size && cnt>=buf_size);
     if(cnt>=buf_size)
     {
       // string is longer the the maximum buffer size (max_size)
       SC_REPORT_WARNING( sc_core::SC_ID_STRING_TOO_LONG_, "truncated" );
       buf[buf_size-1] = 000;
     }
     result = buf;
     delete[] buf;
   }
   else
     result = buffer;

   va_end(argptr);

   return result;
}

void
sc_string_old::print( ::std::ostream& os ) const
{
    os << rep->str;
}

void sc_string_old::test(int position)const
{
	if(position<0 || position>=length())
		SC_REPORT_ERROR( sc_core::SC_ID_OUT_OF_BOUNDS_, "sc_string_old::test" );
}

// TODO: conveniece formatting functions for common types
//       e.g. sc_string_old("a=%d, s is %s").fmt(1).fmt("string")
//       should produce a=1, s is string
//       it should be safe: if less arguments specified
//       it should print %specifier; extra arguments should be ignored
//       if the type of the argument is incompatible with format 
//       specifier it should be ignored
//

unsigned
sc_string_old::fmt_length()const
{
    unsigned result=0;
    if((*this)[0]!='%')
	return 0;
    else
	result++;
    if(is_delimiter("-+0 #",result)) // flags
	result++;
    while(is_delimiter("0123456789*",result)) // width
	result++;
    if(rep->str[result]=='.') // precision
    {
	result++;
	unsigned old_result = result;
	while(is_delimiter("0123456789*",result)) result++;
	if(old_result == result) //error in format
	    return 0;
    }
    if(is_delimiter("hlL",result)) result++; // I64 is not supported
    if(is_delimiter("cCdiouxXeEfgGnpsS",result)) 
	result++;
    else // error in format
	return 0;
    return result;
}

sc_string_old&
sc_string_old::fmt(const sc_string_old& s)
{
    return fmt(s.c_str());
}

int
sc_string_old::pos( const sc_string_old& sub_string ) const
{
    int sub_len = sub_string.length();
    if( sub_len == 0 ) {
        return 0; // empty string always matches
    }
    int ind = 0;
    int len = length();
    bool found = false;
    while( ind < len && ! found )
    {
        found = ( sub_string == substr( ind, ind + sub_len - 1 ) );
        ++ ind;
    }
    if( found ) {
        return -- ind;
    } else {
        return -1;
    }
}

sc_string_old&
sc_string_old::remove(unsigned index, unsigned length)
{
    test((int)index);
    if(length!=0)
	(*this) = substr(0,index-1) + substr(index+length,this->length()-1);
    return *this;
}

sc_string_old&
sc_string_old::insert(const sc_string_old& sub_string, unsigned index)
{
    if(index>(unsigned)length())   
	SC_REPORT_ERROR( sc_core::SC_ID_OUT_OF_BOUNDS_, "sc_string_old::insert" );
    return (*this) = substr(0,index-1)+sub_string+substr(index,length()-1);
}

bool
sc_string_old::is_delimiter(const sc_string_old& str, unsigned index)const
{
    test((int)index);
    return str.contains(rep->str[index]);
}

bool
sc_string_old::contains(char c)const
{
    int len = length();
    int i=0;
    bool found = false;
    while(!found && i<len)
	found = rep->str[i++]==c;
    return found;
}

sc_string_old
sc_string_old::uppercase()const
{
    int len = length();
    sc_string_old temp(*this);
    for(int i=0; i<len; i++)
    {
	char c = temp.rep->str[i];
	if(c>='a' && c<='z')
	    temp.rep->str[i] = static_cast<char>( c-32 );
    }
    return temp;
}

sc_string_old
sc_string_old::lowercase()const
{
    int len = length();
    sc_string_old temp(*this);
    for(int i=0; i<len; i++)
    {
	char c = temp.rep->str[i];
	if(c>='A' && c<='Z')
	    temp.rep->str[i] = static_cast<char>( c+32 );
    }
    return temp;
}


// ----------------------------------------------------------------------------

::std::istream&
operator >> ( ::std::istream& is, sc_string_old& s )
{
    if( s.rep->ref_count > 1 ) {
        -- s.rep->ref_count;
        s.rep = new sc_string_rep;
    }

    int i = 0;
    char* p = s.rep->str;
    char c;

    // skip white spaces
    while( is.get( c ) && isspace( c ) )
        ;

    for( ; is.good() && ! isspace( c ); is.get( c ) ) {
        if( i > s.rep->alloc - 2 ) {
	    s.rep->str[i] = '\0';
            s.rep->resize( (int) (s.rep->alloc * 1.5) );
            p = s.rep->str + i;
        }
        *p ++ = c;
        i ++;
    }
    *p = '\0';

    return is;
}
 } // namespace sc_dt

// $Log: sc_string.cpp,v $
// Revision 1.6  2011/08/29 18:04:32  acg
//  Philipp A. Hartmann: miscellaneous clean ups.
//
// Revision 1.5  2011/08/26 22:49:42  acg
//  Torsten Maehne: remove redudant assignment.
//
// Revision 1.4  2011/08/26 20:46:19  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
// Revision 1.3  2011/08/24 22:05:56  acg
//  Torsten Maehne: initialization changes to remove warnings.
//
// Revision 1.2  2011/02/18 20:38:44  acg
//  Andy Goodrich: Updated Copyright notice.
//
// Revision 1.1.1.1  2006/12/15 20:20:06  acg
// SystemC 2.3
//
// Revision 1.3  2006/01/13 18:53:11  acg
// Andy Goodrich: Added $Log command so that CVS comments are reproduced in
// the source.

// taf
