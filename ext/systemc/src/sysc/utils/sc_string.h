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

  sc_string.h -- Implementation of a simple string class.

  Original Author: Stan Y. Liao, Synopsys, Inc.

  CHANGE LOG AT END OF FILE
 *****************************************************************************/

// $Log: sc_string.h,v $
// Revision 1.3  2011/08/26 20:46:19  acg
//  Andy Goodrich: moved the modification log to the end of the file to
//  eliminate source line number skew when check-ins are done.
//
#ifndef SC_STRING_H
#define SC_STRING_H


#include "sysc/utils/sc_iostream.h"
#include "sysc/utils/sc_report.h"

namespace sc_dt {
	class sc_string_old;
}

#ifdef SC_USE_SC_STRING_OLD
	typedef sc_dt::sc_string_old sc_string;
#endif
#ifdef SC_USE_STD_STRING
	typedef ::std::string sc_string;
#endif

namespace sc_dt {

// forward class declarations
class sc_string_rep;

// friend operator declarations
sc_string_old operator + ( const char* s, const sc_string_old& t );


// ----------------------------------------------------------------------------
//  CLASS : sc_string
//
//  String class (yet another).
// ----------------------------------------------------------------------------

class sc_string_old
{
    friend systemc_ostream& operator << (systemc_ostream& os, const sc_string_old& a);
    friend systemc_istream& operator >> ( systemc_istream& is, sc_string_old& a );

public:

    //  constructors

    explicit sc_string_old( int size = 16 );
    sc_string_old( const char* s );
    sc_string_old( const char* s, int n ); // get first n chars from the string
    sc_string_old( const sc_string_old& s );


    // destructor

    ~sc_string_old();


    // concatenation and assignment

    sc_string_old& operator = ( const char* s );
    sc_string_old& operator = ( const sc_string_old& s );

    sc_string_old& operator += ( const char* s );
    sc_string_old& operator += ( char c );
    sc_string_old& operator += ( const sc_string_old& s );

    sc_string_old operator + ( const char* s ) const;
    sc_string_old operator + ( char c ) const;
    sc_string_old operator + ( const sc_string_old& s ) const;

    friend sc_string_old operator + ( const char* s, const sc_string_old& t );


    // returns substring [first,last]

    sc_string_old substr( int first, int last ) const;


    // string comparison operators

    bool operator == ( const char* s ) const;
    bool operator != ( const char* s ) const;
    bool operator <  ( const char* s ) const;
    bool operator <= ( const char* s ) const;
    bool operator >  ( const char* s ) const;
    bool operator >= ( const char* s ) const;
    bool operator == ( const sc_string_old& s ) const;
    bool operator != ( const sc_string_old& s ) const;
    bool operator <  ( const sc_string_old& s ) const;
    bool operator <= ( const sc_string_old& s ) const;
    bool operator >  ( const sc_string_old& s ) const;
    bool operator >= ( const sc_string_old& s ) const;

    //
    // returns length of the string (excluding trailing \0)
    //
    int length() const;

    //
    // returns c-style string
    //
    const char* c_str() const;
    //
    // returns c-style string
    //
    operator const char*() const;
    //
    // returns character at "index" position
    //
    char operator[](int index) const;
    //
    // l-value subscript
    //
    char& operator[](int index);

    // formatted string (see printf description)
    static sc_string_old to_string(const char* format, ...);
    //
    //       conveniece formatting functions for common types
    //       e.g. sc_string_old("a=%d, s is %s").fmt(1).fmt("string")
    //       should produce: a=1, s is string
    //       it should be safe: if less arguments specified
    //       it should print %specifier; extra arguments should be ignored
    // TODO: if the type of the argument is incompatible with format
    //       specifier it should be ignored
    //
    // must have it inlined because of some compilers
    template<class T> sc_string_old& fmt(const T& t)
	{
	    // search %
	    int index;
	    int last_char = length()-1;
	    sc_string_old temp(*this);
	    do
	    {
		index = temp.pos("%");
		if(index == last_char)
		    return *this;
		temp = substr(index,last_char);
	    } while(temp[0] != '%');
	    int f_len = (int)temp.fmt_length(); // length of format field
	    temp = to_string(substr(0,index+f_len-1).c_str(),t);
	    return (*this) = temp + substr(index+f_len,last_char);
	}
    sc_string_old& fmt(const sc_string_old& s);
    //
    // find position of substring in this string
    // returns -1 if not found
    //
    int pos(const sc_string_old& sub_string)const;
    //
    // remove "count" characters from "index"
    //
    sc_string_old& remove(unsigned index, unsigned length);
    //
    // insert "substring" before "index"
    //
    sc_string_old& insert(const sc_string_old& sub_string, unsigned index);
    //
    // returns true if the character at byte index in this string matches
    // any character in the delimiters string
    //
    bool is_delimiter(const sc_string_old& str, unsigned index)const;
    //
    // returns true if string contains the character
    //
    bool contains(char c)const;
    //
    // produce upper case string from this one
    //
    sc_string_old uppercase()const;
    //
    // produce lower case string from this one
    //
    sc_string_old lowercase()const;
    //
    // legacy methods
    //
    static sc_string_old make_str(long n);
    void set( int index, char c );
    int cmp( const char* s ) const;
    int cmp( const sc_string_old& s ) const;


    void print( systemc_ostream& os = ::std::cout ) const;

private:

    sc_string_old( sc_string_rep* r );

    sc_string_rep* rep;

    void test(int position)const;
    unsigned fmt_length()const;
};


// IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII

inline
systemc_ostream&
operator << ( systemc_ostream& os, const sc_string_old& a )
{
    a.print( os );
    return os;
}

} // namespace sc_dt

// Revision 1.2  2011/02/18 20:38:44  acg
//  Andy Goodrich: Updated Copyright notice.
//
// Revision 1.1.1.1  2006/12/15 20:20:06  acg
// SystemC 2.3
//
// Revision 1.4  2006/05/08 17:50:51  acg
//   Andy Goodrich: added David Long's forward declarations for friend
//   functions, methods, and operators to keep the Microsoft compiler happy.
//
// Revision 1.3  2006/01/13 18:53:11  acg
// Andy Goodrich: Added $Log command so that CVS comments are reproduced in
// the source.
//

#endif
