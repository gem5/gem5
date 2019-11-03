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

  sc_vcd_trace.cpp - Implementation of VCD tracing.

  Original Author - Abhijit Ghosh, Synopsys, Inc.

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date: Ali Dasdan, Synopsys, Inc.
  Description of Modification: - Replaced 'width' of sc_(u)int with their
                                 'bitwidth()'.
    
      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/*****************************************************************************

   Acknowledgement: The tracing mechanism is based on the tracing
   mechanism developed at Infineon (formerly Siemens HL). Though this
   code is somewhat different, and significantly enhanced, the basics
   are identical to what was originally contributed by Infineon.  The
   contribution of Infineon in the development of this tracing
   technology is hereby acknowledged.

 *****************************************************************************/


#include <cstdlib>
#include <string.h>
#include <vector>

#define SC_DISABLE_API_VERSION_CHECK // for in-library sc_ver.h inclusion

#include "sysc/kernel/sc_simcontext.h"
#include "sysc/kernel/sc_ver.h"
#include "sysc/datatypes/bit/sc_bit.h"
#include "sysc/datatypes/bit/sc_logic.h"
#include "sysc/datatypes/bit/sc_lv_base.h"
#include "sysc/datatypes/int/sc_signed.h"
#include "sysc/datatypes/int/sc_unsigned.h"
#include "sysc/datatypes/int/sc_int_base.h"
#include "sysc/datatypes/int/sc_uint_base.h"
#include "sysc/datatypes/fx/fx.h"
#include "sysc/tracing/sc_vcd_trace.h"

namespace sc_core {

// Forward declarations for functions that come later in the file
// Map sc_dt::sc_logic to printable VCD
static char map_sc_logic_state_to_vcd_state(char in_char);

// Remove name problems associated with [] in vcd names
static void remove_vcd_name_problems(vcd_trace const* vcd, std::string& name);

const char* vcd_types[vcd_trace_file::VCD_LAST]={"wire","real"};


// ----------------------------------------------------------------------------
//  CLASS : vcd_trace
//
//  Base class for VCD traces.
// ----------------------------------------------------------------------------

class vcd_trace
{
public:

    vcd_trace(const std::string& name_, const std::string& vcd_name_);

    // Needs to be pure virtual as has to be defined by the particular
    // type being traced
    virtual void write(FILE* f) = 0;
    
    virtual void set_width();

    static const char* strip_leading_bits(const char* originalbuf);

    // Comparison function needs to be pure virtual too
    virtual bool changed() = 0;

    // Make this virtual as some derived classes may overwrite
    virtual void print_variable_declaration_line(FILE* f);

    void compose_data_line(char* rawdata, char* compdata);
    std::string compose_line(const std::string& data);

    virtual ~vcd_trace();

    const std::string name;
    const std::string vcd_name;
    const char* vcd_var_typ_name;
    int bit_width; 
};


vcd_trace::vcd_trace(const std::string& name_, const std::string& vcd_name_)
: name(name_), vcd_name(vcd_name_), vcd_var_typ_name(0), bit_width(0)
{
    /* Intentionally blank */
}
        
void
vcd_trace::compose_data_line(char* rawdata, char* compdata)
{
    sc_assert(rawdata != compdata);

    if(bit_width == 0)
    {
        compdata[0] = '\0';
    }
    else
    {
        if(bit_width == 1)
        {
            compdata[0] = rawdata[0];
            strcpy(&(compdata[1]), vcd_name.c_str());
        }
        else
        {
            const char* effective_begin = strip_leading_bits(rawdata);
            std::sprintf(compdata, "b%s %s", effective_begin, vcd_name.c_str());
        }
    }
}

// same as above but not that ugly
std::string
vcd_trace::compose_line(const std::string& data)
{
  if(bit_width == 0)
    return "";
  if(bit_width == 1)
    return data + vcd_name;
  return std::string("b")+strip_leading_bits(data.c_str())+" "+vcd_name;
}

void
vcd_trace::print_variable_declaration_line(FILE* f)
{
    char buf[2000];

    if ( bit_width <= 0 )
    {
        std::stringstream ss;
        ss << "'" << name << "' has 0 bits";
        SC_REPORT_ERROR( SC_ID_TRACING_OBJECT_IGNORED_
                       , ss.str().c_str() );
        return;
    }

    std::string namecopy = name; 
    remove_vcd_name_problems(this, namecopy);
    if ( bit_width == 1 )
    {
        std::sprintf(buf, "$var %s  % 3d  %s  %s       $end\n",
                     vcd_var_typ_name,
                     bit_width,
                     vcd_name.c_str(),
                     namecopy.c_str());
    }
    else
    {
        std::sprintf(buf, "$var %s  % 3d  %s  %s [%d:0]  $end\n",
                     vcd_var_typ_name,
                     bit_width,
                     vcd_name.c_str(),
                     namecopy.c_str(),
                     bit_width-1);
    }
    std::fputs(buf, f);
}

void
vcd_trace::set_width()
{
  /* Intentionally Blank, should be defined for each type separately */
}

const char*
vcd_trace::strip_leading_bits(const char* originalbuf)
{
    //*********************************************************************
    // - Remove multiple leading 0,z,x, and replace by only one
    // - For example,
    //    b000z100    -> b0z100
    //    b00000xxx   -> b0xxx
    //    b000        -> b0
    //    bzzzzz1     -> bz1
    //    bxxxz10     -> xz10
    // - For leading 0's followed by 1, remove all leading 0's
    //    b0000010101 -> b10101  

    const char* position = originalbuf;

    if( strlen(originalbuf) < 2 ||
	(originalbuf[0] != 'z' && originalbuf[0] != 'x' &&
	 originalbuf[0] != '0' ))
      return originalbuf;

    char first_char = *position;
    while(*position == first_char)
    {
        position++;
    }

    if(first_char == '0' && *position == '1')
        return position;
    // else
    return position-1;
}

vcd_trace::~vcd_trace()
{
  /* Intentionally Blank */
}


template <class T>
class vcd_T_trace : public vcd_trace
{
  public:

    vcd_T_trace( const T& object_,
		 const std::string& name_,
		 const std::string& vcd_name_,
		 vcd_trace_file::vcd_enum type_ )
    : vcd_trace( name_, vcd_name_ ),
      object( object_ ),
      old_value( object_ )
    {
        vcd_var_typ_name = vcd_types[type_];
    }

    void write( FILE* f )
    {
        std::fprintf( f, "%s", compose_line( object.to_string() ).c_str() );
        old_value = object;
    }

    bool changed()
        { return !(object == old_value); }

    void set_width()
        { bit_width = object.length(); }

protected:

    const T& object;
    T        old_value;
};

typedef vcd_T_trace<sc_dt::sc_bv_base> vcd_sc_bv_trace;
typedef vcd_T_trace<sc_dt::sc_lv_base> vcd_sc_lv_trace;

// Trace sc_dt::sc_bv_base (sc_dt::sc_bv)
void
vcd_trace_file::trace(
    const sc_dt::sc_bv_base& object, const std::string& name)
{
   traceT(object,name);
}   

// Trace sc_dt::sc_lv_base (sc_dt::sc_lv)
void
vcd_trace_file::trace(
    const sc_dt::sc_lv_base& object, const std::string& name)
{
   traceT(object,name);
}

/*****************************************************************************/

class vcd_bool_trace : public vcd_trace {
public:
    vcd_bool_trace(const bool& object_,
		   const std::string& name_,
		   const std::string& vcd_name_);
    void write(FILE* f);
    bool changed();

protected:
    const bool& object;
    bool old_value;
};

vcd_bool_trace::vcd_bool_trace(const bool& object_,
			       const std::string& name_,
			       const std::string& vcd_name_)
: vcd_trace(name_, vcd_name_), object(object_), old_value(object_)
{
    vcd_var_typ_name = "wire";
    bit_width = 1;
}

bool
vcd_bool_trace::changed()
{
    return object != old_value;
}

void
vcd_bool_trace::write(FILE* f)
{
    if (object == true) std::fputc('1', f);
    else std::fputc('0', f);

    std::fprintf(f,"%s", vcd_name.c_str());

    old_value = object;
}

//*****************************************************************************

class vcd_sc_bit_trace : public vcd_trace {
public:
    vcd_sc_bit_trace(const sc_dt::sc_bit& , const std::string& , 
    	const std::string& );
    void write(FILE* f);
    bool changed();

protected:
    const sc_dt::sc_bit& object;
    sc_dt::sc_bit old_value;
};

vcd_sc_bit_trace::vcd_sc_bit_trace( const sc_dt::sc_bit& object_,
				    const std::string& name,
				    const std::string& vcd_name)
: vcd_trace(name, vcd_name), object( object_ ), old_value( object_ )
{
    vcd_var_typ_name = "wire";
    bit_width = 1;
}

bool
vcd_sc_bit_trace::changed()
{
    return object != old_value;
}

void
vcd_sc_bit_trace::write(FILE* f)
{
    if (object == true) std::fputc('1', f);
    else std::fputc('0', f);

    std::fprintf(f,"%s", vcd_name.c_str());

    old_value = object;
}

/*****************************************************************************/

class vcd_sc_logic_trace : public vcd_trace {
public:
    vcd_sc_logic_trace(const sc_dt::sc_logic& object_,
		       const std::string& name_,
		       const std::string& vcd_name_);
    void write(FILE* f);
    bool changed();

protected:    
    const sc_dt::sc_logic& object;
    sc_dt::sc_logic old_value;
};


vcd_sc_logic_trace::vcd_sc_logic_trace(const sc_dt::sc_logic& object_,
				       const std::string& name_,
				       const std::string& vcd_name_) 
: vcd_trace(name_, vcd_name_), object(object_), old_value(object_)
{
    vcd_var_typ_name = "wire";
    bit_width = 1;
}


bool
vcd_sc_logic_trace::changed()
{
    return object != old_value;
}


void
vcd_sc_logic_trace::write(FILE* f)
{
    char out_char;
    out_char = map_sc_logic_state_to_vcd_state(object.to_char());
    std::fputc(out_char, f); 

    std::fprintf(f,"%s", vcd_name.c_str());

    old_value = object;
}


/*****************************************************************************/

class vcd_sc_unsigned_trace : public vcd_trace {
public:
    vcd_sc_unsigned_trace(const sc_dt::sc_unsigned& object,
			  const std::string& name_,
			  const std::string& vcd_name_);
    void write(FILE* f);
    bool changed();
    void set_width();

protected:    
    const sc_dt::sc_unsigned& object;
    sc_dt::sc_unsigned old_value;
};


vcd_sc_unsigned_trace::vcd_sc_unsigned_trace(const sc_dt::sc_unsigned& object_,
					     const std::string& name_,
					     const std::string& vcd_name_) 
: vcd_trace(name_, vcd_name_), object(object_), old_value(object_.length())
// The last may look strange, but is correct
{
    vcd_var_typ_name = "wire";
    old_value = object;
}

bool
vcd_sc_unsigned_trace::changed()
{
    return object != old_value;
}

void
vcd_sc_unsigned_trace::write(FILE* f)
{
    static std::vector<char> compdata(1024), rawdata(1024);
    typedef std::vector<char>::size_type size_t;

    if ( compdata.size() < (size_t)object.length() ) {
        size_t sz = ( (size_t)object.length() + 4096 ) & (~(size_t)(4096-1));
        std::vector<char>( sz ).swap( compdata ); // resize without copying values
        std::vector<char>( sz ).swap( rawdata );
    }
    char *rawdata_ptr  = &rawdata[0];

    for (int bitindex = object.length() - 1; bitindex >= 0; --bitindex) {
        *rawdata_ptr++ = "01"[object[bitindex].to_bool()];
    }
    *rawdata_ptr = '\0';
    compose_data_line(&rawdata[0], &compdata[0]);

    std::fputs(&compdata[0], f);
    old_value = object;
}

void
vcd_sc_unsigned_trace::set_width()
{
    bit_width = object.length();
}


/*****************************************************************************/

class vcd_sc_signed_trace : public vcd_trace {
public:
    vcd_sc_signed_trace(const sc_dt::sc_signed& object,
			const std::string& name_,
			const std::string& vcd_name_);
    void write(FILE* f);
    bool changed();
    void set_width();

protected:    
    const sc_dt::sc_signed& object;
    sc_dt::sc_signed old_value;
};


vcd_sc_signed_trace::vcd_sc_signed_trace(const sc_dt::sc_signed& object_,
					 const std::string& name_,
					 const std::string& vcd_name_) 
: vcd_trace(name_, vcd_name_), object(object_), old_value(object_.length())
{
    vcd_var_typ_name = "wire";
    old_value = object;
}

bool
vcd_sc_signed_trace::changed()
{
    return object != old_value;
}

void
vcd_sc_signed_trace::write(FILE* f)
{
    static std::vector<char> compdata(1024), rawdata(1024);
    typedef std::vector<char>::size_type size_t;

    if ( compdata.size() < (size_t)object.length() ) {
        size_t sz = ( (size_t)object.length() + 4096 ) & (~(size_t)(4096-1));
        std::vector<char>( sz ).swap( compdata ); // resize without copying values
        std::vector<char>( sz ).swap( rawdata );
    }
    char *rawdata_ptr  = &rawdata[0];

    for (int bitindex = object.length() - 1; bitindex >= 0; --bitindex) {
        *rawdata_ptr++ = "01"[object[bitindex].to_bool()];
    }
    *rawdata_ptr = '\0';
    compose_data_line(&rawdata[0], &compdata[0]);

    std::fputs(&compdata[0], f);
    old_value = object;
}

void
vcd_sc_signed_trace::set_width()
{
    bit_width = object.length();
}

/*****************************************************************************/

class vcd_sc_uint_base_trace : public vcd_trace {
public:
    vcd_sc_uint_base_trace(const sc_dt::sc_uint_base& object,
			   const std::string& name_,
			   const std::string& vcd_name_);
    void write(FILE* f);
    bool changed();
    void set_width();

protected:    
    const sc_dt::sc_uint_base& object;
    sc_dt::sc_uint_base old_value;
};


vcd_sc_uint_base_trace::vcd_sc_uint_base_trace(
                                          const sc_dt::sc_uint_base& object_,
					  const std::string& name_,
					  const std::string& vcd_name_)
: vcd_trace(name_, vcd_name_), object(object_), old_value(object_.length())
// The last may look strange, but is correct
{
    vcd_var_typ_name = "wire";
    old_value = object;
}

bool
vcd_sc_uint_base_trace::changed()
{
    return object != old_value;
}

void
vcd_sc_uint_base_trace::write(FILE* f)
{
    char rawdata[1000], *rawdata_ptr = rawdata;
    char compdata[1000];

    int bitindex;
    for (bitindex = object.length()-1; bitindex >= 0; --bitindex) {
        *rawdata_ptr++ = "01"[object[bitindex].to_bool()];
    }
    *rawdata_ptr = '\0';
    compose_data_line(rawdata, compdata);

    std::fputs(compdata, f);
    old_value = object;
}

void
vcd_sc_uint_base_trace::set_width()
{
    bit_width = object.length();
}


/*****************************************************************************/

class vcd_sc_int_base_trace : public vcd_trace {
public:
    vcd_sc_int_base_trace(const sc_dt::sc_int_base& object,
			  const std::string& name_,
			  const std::string& vcd_name_);
    void write(FILE* f);
    bool changed();
    void set_width();

protected:    
    const sc_dt::sc_int_base& object;
    sc_dt::sc_int_base old_value;
};


vcd_sc_int_base_trace::vcd_sc_int_base_trace(const sc_dt::sc_int_base& object_,
					     const std::string& name_,
					     const std::string& vcd_name_) 
: vcd_trace(name_, vcd_name_), object(object_), old_value(object_.length())
{
    vcd_var_typ_name = "wire";
    old_value = object;
}

bool
vcd_sc_int_base_trace::changed()
{
    return object != old_value;
}

void
vcd_sc_int_base_trace::write(FILE* f)
{
    char rawdata[1000], *rawdata_ptr = rawdata;
    char compdata[1000];

    int bitindex;
    for (bitindex = object.length()-1; bitindex >= 0; --bitindex) {
        *rawdata_ptr++ = "01"[object[bitindex].to_bool()];
    }
    *rawdata_ptr = '\0';
    compose_data_line(rawdata, compdata);

    std::fputs(compdata, f);
    old_value = object;
}

void
vcd_sc_int_base_trace::set_width()
{
    bit_width = object.length();
}


/*****************************************************************************/

class vcd_sc_fxval_trace : public vcd_trace
{
public:

    vcd_sc_fxval_trace( const sc_dt::sc_fxval& object,
			const std::string& name_,
			const std::string& vcd_name_ );
    void write( FILE* f );
    bool changed();

protected:

    const sc_dt::sc_fxval& object;
    sc_dt::sc_fxval old_value;

};

vcd_sc_fxval_trace::vcd_sc_fxval_trace( const sc_dt::sc_fxval& object_,
				        const std::string& name_,
					const std::string& vcd_name_ )
: vcd_trace( name_, vcd_name_ ),
  object( object_ ), old_value( object_ )
{
    vcd_var_typ_name = "real";
    bit_width = 1;
}

bool
vcd_sc_fxval_trace::changed()
{
    return object != old_value;
}

void
vcd_sc_fxval_trace::write( FILE* f )
{
    std::fprintf( f, "r%.16g %s", object.to_double(), vcd_name.c_str() );
    old_value = object;
}

/*****************************************************************************/

class vcd_sc_fxval_fast_trace : public vcd_trace
{
public:

    vcd_sc_fxval_fast_trace( const sc_dt::sc_fxval_fast& object,
			     const std::string& name_,
			     const std::string& vcd_name_ );
    void write( FILE* f );
    bool changed();

protected:

    const sc_dt::sc_fxval_fast& object;
    sc_dt::sc_fxval_fast old_value;

};

vcd_sc_fxval_fast_trace::vcd_sc_fxval_fast_trace( 
                                        const sc_dt::sc_fxval_fast& object_,
					const std::string& name_,
					const std::string& vcd_name_ )
: vcd_trace( name_, vcd_name_ ),
  object( object_ ), old_value( object_ )
{
    vcd_var_typ_name = "real";
    bit_width = 1;
}

bool
vcd_sc_fxval_fast_trace::changed()
{
    return object != old_value;
}

void
vcd_sc_fxval_fast_trace::write( FILE* f )
{
    std::fprintf( f, "r%.16g %s", object.to_double(), vcd_name.c_str() );
    old_value = object;
}

/*****************************************************************************/

class vcd_sc_fxnum_trace : public vcd_trace
{
public:

    vcd_sc_fxnum_trace( const sc_dt::sc_fxnum& object,
			const std::string& name_,
			const std::string& vcd_name_ );
    void write( FILE* f );
    bool changed();
    void set_width();

protected:

    const sc_dt::sc_fxnum& object;
    sc_dt::sc_fxnum old_value;

};

vcd_sc_fxnum_trace::vcd_sc_fxnum_trace( const sc_dt::sc_fxnum& object_,
				        const std::string& name_,
					const std::string& vcd_name_ )
: vcd_trace( name_, vcd_name_ ),
  object( object_ ),
  old_value( object_.m_params.type_params(),
	     object_.m_params.enc(),
	     object_.m_params.cast_switch(),
	     0 )
{
    vcd_var_typ_name = "wire";
    old_value = object;
}

bool
vcd_sc_fxnum_trace::changed()
{
    return object != old_value;
}

void
vcd_sc_fxnum_trace::write( FILE* f )
{
    static std::vector<char> compdata(1024), rawdata(1024);
    typedef std::vector<char>::size_type size_t;

    if ( compdata.size() < (size_t)object.wl() ) {
        size_t sz = ( (size_t)object.wl() + 4096 ) & (~(size_t)(4096-1));
        std::vector<char>( sz ).swap( compdata ); // resize without copying values
        std::vector<char>( sz ).swap( rawdata );
    }
    char *rawdata_ptr  = &rawdata[0];

    for(int bitindex = object.wl() - 1; bitindex >= 0; -- bitindex )
    {
        *rawdata_ptr ++ = "01"[object[bitindex]];
    }
    *rawdata_ptr = '\0';
    compose_data_line( &rawdata[0], &compdata[0] );

    std::fputs( &compdata[0], f );
    old_value = object;
}

void
vcd_sc_fxnum_trace::set_width()
{
    bit_width = object.wl();
}

/*****************************************************************************/

class vcd_sc_fxnum_fast_trace : public vcd_trace
{
public:

    vcd_sc_fxnum_fast_trace( const sc_dt::sc_fxnum_fast& object,
			     const std::string& name_,
			     const std::string& vcd_name_ );
    void write( FILE* f );
    bool changed();
    void set_width();

protected:

    const sc_dt::sc_fxnum_fast& object;
    sc_dt::sc_fxnum_fast old_value;

};

vcd_sc_fxnum_fast_trace::vcd_sc_fxnum_fast_trace( 
                                        const sc_dt::sc_fxnum_fast& object_,
					const std::string& name_,
					const std::string& vcd_name_ )
: vcd_trace( name_, vcd_name_ ),
  object( object_ ),
  old_value( object_.m_params.type_params(),
	     object_.m_params.enc(),
	     object_.m_params.cast_switch(),
	     0 )
{
    vcd_var_typ_name = "wire";
    old_value = object;
}

bool
vcd_sc_fxnum_fast_trace::changed()
{
    return object != old_value;
}

void
vcd_sc_fxnum_fast_trace::write( FILE* f )
{
    static std::vector<char> compdata(1024), rawdata(1024);
    typedef std::vector<char>::size_type size_t;

    if ( compdata.size() < (size_t)object.wl() ) {
        size_t sz = ( (size_t)object.wl() + 4096 ) & (~(size_t)(4096-1));
        std::vector<char>( sz ).swap( compdata ); // resize without copying values
        std::vector<char>( sz ).swap( rawdata );
    }
    char *rawdata_ptr  = &rawdata[0];

    for(int bitindex = object.wl() - 1; bitindex >= 0; -- bitindex )
    {
        *rawdata_ptr ++ = "01"[object[bitindex]];
    }
    *rawdata_ptr = '\0';
    compose_data_line( &rawdata[0], &compdata[0] );

    std::fputs( &compdata[0], f );
    old_value = object;
}

void
vcd_sc_fxnum_fast_trace::set_width()
{
    bit_width = object.wl();
}


/*****************************************************************************/

class vcd_unsigned_int_trace : public vcd_trace {
public:
    vcd_unsigned_int_trace(const unsigned& object,
			   const std::string& name_,
			   const std::string& vcd_name_,
			   int width_);
    void write(FILE* f);
    bool changed();

protected:
    const unsigned& object;
    unsigned old_value;
    unsigned mask; 
};


vcd_unsigned_int_trace::vcd_unsigned_int_trace(
                                            const unsigned& object_,
					    const std::string& name_,
					    const std::string& vcd_name_,
					    int width_) 
: vcd_trace(name_, vcd_name_), object(object_), old_value( object_ ),
  mask((unsigned)-1)
{
    bit_width = width_;
    if (bit_width < 32) mask = ~(-1 << bit_width);

    vcd_var_typ_name = "wire";
}


bool
vcd_unsigned_int_trace::changed()
{
    return object != old_value;
}


void
vcd_unsigned_int_trace::write(FILE* f)
{
    char rawdata[1000];
    char compdata[1000];
    int bitindex;

    // Check for overflow
    if ((object & mask) != object) {
        for (bitindex = 0; bitindex < bit_width; bitindex++){
            rawdata[bitindex] = 'x';
        }
    }
    else{
        unsigned bit_mask = 1 << (bit_width-1);
        for (bitindex = 0; bitindex < bit_width; bitindex++) {
            rawdata[bitindex] = (object & bit_mask)? '1' : '0';
            bit_mask = bit_mask >> 1;
        }
    }
    rawdata[bitindex] = '\0';
    compose_data_line(rawdata, compdata);
    std::fputs(compdata, f);
    old_value = object;
}

/*****************************************************************************/

class vcd_unsigned_short_trace : public vcd_trace {
public:
    vcd_unsigned_short_trace(const unsigned short& object,
			     const std::string& name_,
			     const std::string& vcd_name_,
			     int width_);
    void write(FILE* f);
    bool changed();

protected:
    const unsigned short& object;
    unsigned short old_value;
    unsigned short mask; 
};


vcd_unsigned_short_trace::vcd_unsigned_short_trace(
                                        const unsigned short& object_,
					const std::string& name_,
					const std::string& vcd_name_,
					int width_) 
: vcd_trace(name_, vcd_name_), object(object_), old_value(object_), mask(0xffff)
{
    bit_width = width_;
    if (bit_width < 16) mask = (unsigned short)~(-1 << bit_width);

    vcd_var_typ_name = "wire";
}


bool
vcd_unsigned_short_trace::changed()
{
    return object != old_value;
}


void
vcd_unsigned_short_trace::write(FILE* f)
{
    char rawdata[1000];
    char compdata[1000];
    int bitindex;

    // Check for overflow
    if ((object & mask) != object) {
        for (bitindex = 0; bitindex < bit_width; bitindex++){
            rawdata[bitindex] = 'x';
        }
    }
    else{
        unsigned bit_mask = 1 << (bit_width-1);
        for (bitindex = 0; bitindex < bit_width; bitindex++) {
            rawdata[bitindex] = (object & bit_mask)? '1' : '0';
            bit_mask = bit_mask >> 1;
        }
    }
    rawdata[bitindex] = '\0';
    compose_data_line(rawdata, compdata);
    std::fputs(compdata, f);
    old_value = object;
}

/*****************************************************************************/

class vcd_unsigned_char_trace : public vcd_trace {
public:
    vcd_unsigned_char_trace(const unsigned char& object,
			    const std::string& name_,
			    const std::string& vcd_name_,
			    int width_);
    void write(FILE* f);
    bool changed();

protected:
    const unsigned char& object;
    unsigned char old_value;
    unsigned char mask; 
};


vcd_unsigned_char_trace::vcd_unsigned_char_trace(
                                const unsigned char& object_,
				const std::string& name_,
				const std::string& vcd_name_,
				int width_)
: vcd_trace(name_, vcd_name_), object(object_), old_value(object_), mask(0xff)
{
    bit_width = width_;
    if (bit_width < 8) mask = (unsigned char)~(-1 << bit_width);
    vcd_var_typ_name = "wire";
}


bool vcd_unsigned_char_trace::changed()
{
    return object != old_value;
}


void vcd_unsigned_char_trace::write(FILE* f)
{
    char rawdata[1000];
    char compdata[1000];
    int bitindex;

    // Check for overflow
    if ((object & mask) != object) {
        for (bitindex = 0; bitindex < bit_width; bitindex++){
            rawdata[bitindex] = 'x';
        }
    }
    else{
        unsigned bit_mask = 1 << (bit_width-1);
        for (bitindex = 0; bitindex < bit_width; bitindex++) {
            rawdata[bitindex] = (object & bit_mask)? '1' : '0';
            bit_mask = bit_mask >> 1;
        }
    }
    rawdata[bitindex] = '\0';
    compose_data_line(rawdata, compdata);
    std::fputs(compdata, f);
    old_value = object;
}

/*****************************************************************************/

class vcd_unsigned_long_trace : public vcd_trace {
public:
    vcd_unsigned_long_trace(const unsigned long& object,
			    const std::string& name_,
			    const std::string& vcd_name_,
			    int width_);
    void write(FILE* f);
    bool changed();

protected:
    const unsigned long& object;
    unsigned long old_value;
    unsigned long mask; 
};


vcd_unsigned_long_trace::vcd_unsigned_long_trace(
                                const unsigned long& object_,
				const std::string& name_,
				const std::string& vcd_name_,
				int width_)
: vcd_trace(name_, vcd_name_), object(object_), old_value(object_), 
  mask((unsigned long)-1)
{
    bit_width = width_;
    if ( bit_width < (int)(sizeof(unsigned long)*8) ) 
        mask = ~(-1L << bit_width);

    vcd_var_typ_name = "wire";
}


bool vcd_unsigned_long_trace::changed()
{
    return object != old_value;
}


void vcd_unsigned_long_trace::write(FILE* f)
{
    char rawdata[1000];
    char compdata[1000];
    int bitindex;

    // Check for overflow
    if ((object & mask) != object) {
        for (bitindex = 0; bitindex < bit_width; bitindex++){
            rawdata[bitindex] = 'x';
        }
    }
    else{
        unsigned long bit_mask = 1ul << (bit_width-1);
        for (bitindex = 0; bitindex < bit_width; bitindex++) {
            rawdata[bitindex] = (object & bit_mask)? '1' : '0';
            bit_mask = bit_mask >> 1;
        }
    }
    rawdata[bitindex] = '\0';
    compose_data_line(rawdata, compdata);
    std::fputs(compdata, f);
    old_value = object;
}

/*****************************************************************************/

class vcd_signed_int_trace : public vcd_trace {
public:
    vcd_signed_int_trace(const int& object,
			 const std::string& name_,
			 const std::string& vcd_name_,
			 int width_);
    void write(FILE* f);
    bool changed();

protected:
    const int& object;
    int old_value;
    unsigned mask; 
};


vcd_signed_int_trace::vcd_signed_int_trace(const signed& object_,
					   const std::string& name_,
					   const std::string& vcd_name_,
					   int width_)
: vcd_trace(name_, vcd_name_), object(object_), old_value(object_),  
  mask((unsigned)-1)
{
    bit_width = width_;
    if (bit_width < 32) mask = ~(-1 << bit_width);

    vcd_var_typ_name = "wire";
}


bool vcd_signed_int_trace::changed()
{
    return object != old_value;
}


void vcd_signed_int_trace::write(FILE* f)
{
    char rawdata[1000];
    char compdata[1000];
    int bitindex;

    // Check for overflow
    if (((unsigned) object & mask) != (unsigned) object) {
        for (bitindex = 0; bitindex < bit_width; bitindex++){
            rawdata[bitindex] = 'x';
        }
    }
    else{
        unsigned bit_mask = 1 << (bit_width-1);
        for (bitindex = 0; bitindex < bit_width; bitindex++) {
            rawdata[bitindex] = (object & bit_mask)? '1' : '0';
            bit_mask = bit_mask >> 1;
        }
    }
    rawdata[bitindex] = '\0';
    compose_data_line(rawdata, compdata);
    std::fputs(compdata, f);
    old_value = object;
}

/*****************************************************************************/

class vcd_signed_short_trace : public vcd_trace {
public:
    vcd_signed_short_trace(const short& object,
			   const std::string& name_,
			   const std::string& vcd_name_,
			   int width_);
    void write(FILE* f);
    bool changed();

protected:
    const short& object;
    short old_value;
    unsigned short mask; 
};


vcd_signed_short_trace::vcd_signed_short_trace(
					const short& object_,
					const std::string& name_,
					const std::string& vcd_name_,
					int width_)
: vcd_trace(name_, vcd_name_), object(object_), old_value(object_), mask(0xffff)
{
    bit_width = width_;
    if (bit_width < 16) mask = (unsigned short)~(-1 << bit_width);

    vcd_var_typ_name = "wire";
}


bool vcd_signed_short_trace::changed()
{
    return object != old_value;
}


void vcd_signed_short_trace::write(FILE* f)
{
    char rawdata[1000];
    char compdata[1000];
    int bitindex;

    // Check for overflow
    if (((unsigned short) object & mask) != (unsigned short) object) {
        for (bitindex = 0; bitindex < bit_width; bitindex++){
            rawdata[bitindex] = 'x';
        }
    }
    else{
        unsigned bit_mask = 1 << (bit_width-1);
        for (bitindex = 0; bitindex < bit_width; bitindex++) {
            rawdata[bitindex] = (object & bit_mask)? '1' : '0';
            bit_mask = bit_mask >> 1;
        }
    }
    rawdata[bitindex] = '\0';
    compose_data_line(rawdata, compdata);
    std::fputs(compdata, f);
    old_value = object;
}

/*****************************************************************************/

class vcd_signed_char_trace : public vcd_trace {
public:
    vcd_signed_char_trace(const char& object,
			  const std::string& name_,
			  const std::string& vcd_name_,
			  int width_);
    void write(FILE* f);
    bool changed();

protected:
    const char& object;
    char old_value;
    unsigned char mask; 
};


vcd_signed_char_trace::vcd_signed_char_trace(const char& object_,
					     const std::string& name_,
					     const std::string& vcd_name_,
					     int width_)
: vcd_trace(name_, vcd_name_), object(object_), old_value(object_), mask(0xff)
{
    bit_width = width_;
    if (bit_width < 8) mask = (unsigned char)~(-1 << bit_width);

    vcd_var_typ_name = "wire";
}


bool vcd_signed_char_trace::changed()
{
    return object != old_value;
}


void vcd_signed_char_trace::write(FILE* f)
{
    char rawdata[1000];
    char compdata[1000];
    int bitindex;

    // Check for overflow
    if (((unsigned char) object & mask) != (unsigned char) object) {
        for (bitindex = 0; bitindex < bit_width; bitindex++){
            rawdata[bitindex] = 'x';
        }
    }
    else{
        unsigned bit_mask = 1 << (bit_width-1);
        for (bitindex = 0; bitindex < bit_width; bitindex++) {
            rawdata[bitindex] = (object & bit_mask)? '1' : '0';
            bit_mask = bit_mask >> 1;
        }
    }
    rawdata[bitindex] = '\0';
    compose_data_line(rawdata, compdata);
    std::fputs(compdata, f);
    old_value = object;
}

/*****************************************************************************/

class vcd_int64_trace : public vcd_trace {
public:
    vcd_int64_trace(const sc_dt::int64& object,
			  const std::string& name_,
			  const std::string& vcd_name_,
			  int width_);
    void write(FILE* f);
    bool changed();

protected:
    const sc_dt::int64& object;
    sc_dt::int64 old_value;
    sc_dt::uint64 mask; 
};


vcd_int64_trace::vcd_int64_trace(const sc_dt::int64& object_,
					     const std::string& name_,
					     const std::string& vcd_name_,
					     int width_)
: vcd_trace(name_, vcd_name_), object(object_), old_value(object_), 
  mask((sc_dt::uint64)-1)
{
    bit_width = width_;
    if (bit_width < 64)  mask = ~(mask << bit_width);
    vcd_var_typ_name = "wire";
}


bool vcd_int64_trace::changed()
{
    return object != old_value;
}


void vcd_int64_trace::write(FILE* f)
{
    char rawdata[1000];
    char compdata[1000];
    int bitindex;

    // Check for overflow
    if (((sc_dt::uint64) object & mask) != (sc_dt::uint64) object) 
	{
        for (bitindex = 0; bitindex < bit_width; bitindex++)
		{
            rawdata[bitindex] = 'x';
        }
    }
    else
	{
        sc_dt::uint64 bit_mask = 1;
        bit_mask = bit_mask << (bit_width-1);
        for (bitindex = 0; bitindex < bit_width; bitindex++) {
            rawdata[bitindex] = (object & bit_mask)? '1' : '0';
            bit_mask = bit_mask >> 1;
        }
    }
    rawdata[bitindex] = '\0';
    compose_data_line(rawdata, compdata);
    std::fputs(compdata, f);
    old_value = object;
}


/*****************************************************************************/

class vcd_uint64_trace : public vcd_trace {
public:
    vcd_uint64_trace(const sc_dt::uint64& object,
		     const std::string& name_,
		     const std::string& vcd_name_,
		     int width_);
    void write(FILE* f);
    bool changed();

protected:
    const sc_dt::uint64& object;
    sc_dt::uint64 old_value;
    sc_dt::uint64 mask; 
};


vcd_uint64_trace::vcd_uint64_trace(  const sc_dt::uint64& object_,
				     const std::string& name_,
				     const std::string& vcd_name_,
				     int width_)
: vcd_trace(name_, vcd_name_), object(object_), old_value(object_),
  mask((sc_dt::uint64)-1)
{
    bit_width = width_;
    if ( bit_width < 64 ) mask = ~(mask << bit_width);
    vcd_var_typ_name = "wire";
}


bool vcd_uint64_trace::changed()
{
    return object != old_value;
}


void vcd_uint64_trace::write(FILE* f)
{
    char rawdata[1000];
    char compdata[1000];
    int bitindex;

    // Check for overflow
    if ((object & mask) != object) 
	{
        for (bitindex = 0; bitindex < bit_width; bitindex++){
            rawdata[bitindex] = 'x';
        }
    }
    else
	{
        sc_dt::uint64 bit_mask = 1;
        bit_mask = bit_mask << (bit_width-1);
        for (bitindex = 0; bitindex < bit_width; bitindex++) 
		{
            rawdata[bitindex] = (object & bit_mask)? '1' : '0';
            bit_mask = bit_mask >> 1;
        }
    }
    rawdata[bitindex] = '\0';
    compose_data_line(rawdata, compdata);
    std::fputs(compdata, f);
    old_value = object;
}


/*****************************************************************************/

class vcd_signed_long_trace : public vcd_trace {
public:
    vcd_signed_long_trace(const long& object,
			  const std::string& name_,
			  const std::string& vcd_name_,
			  int width_);
    void write(FILE* f);
    bool changed();

protected:
    const long& object;
    long old_value;
    unsigned long mask; 
};


vcd_signed_long_trace::vcd_signed_long_trace(const long& object_,
					     const std::string& name_,
					     const std::string& vcd_name_,
					     int width_)
: vcd_trace(name_, vcd_name_), object(object_), old_value(object_),
  mask((unsigned long)-1)
{
    bit_width = width_;
    if ( bit_width < (int)(sizeof(long)*8) ) 
        mask = ~(-1L << bit_width);
    vcd_var_typ_name = "wire";
}


bool vcd_signed_long_trace::changed()
{
    return object != old_value;
}


void vcd_signed_long_trace::write(FILE* f)
{
    char rawdata[1000];
    char compdata[1000];
    int bitindex;

    // Check for overflow
    if (((unsigned long) object & mask) != (unsigned long) object) {
        for (bitindex = 0; bitindex < bit_width; bitindex++){
            rawdata[bitindex] = 'x';
        }
    }
    else{
        unsigned long bit_mask = 1ul << (bit_width-1);
        for (bitindex = 0; bitindex < bit_width; bitindex++) {
            rawdata[bitindex] = (object & bit_mask)? '1' : '0';
            bit_mask = bit_mask >> 1;
        }
    }
    rawdata[bitindex] = '\0';
    compose_data_line(rawdata, compdata);
    std::fputs(compdata, f);
    old_value = object;
}


/*****************************************************************************/

class vcd_float_trace : public vcd_trace {
public:
    vcd_float_trace(const float& object,
		    const std::string& name_,
		    const std::string& vcd_name_);
    void write(FILE* f);
    bool changed();

protected:    
    const float& object;
    float old_value;
};

vcd_float_trace::vcd_float_trace(const float& object_,
				 const std::string& name_,
				 const std::string& vcd_name_)
: vcd_trace(name_, vcd_name_), object(object_)
{
    vcd_var_typ_name = "real";
    bit_width = 1;
    old_value = object;
}

bool vcd_float_trace::changed()
{
    return object != old_value;
}

void vcd_float_trace::write(FILE* f)
{
    std::fprintf(f, "r%.16g %s", object, vcd_name.c_str());
    old_value = object;
}

/*****************************************************************************/

class vcd_double_trace : public vcd_trace {
public:
    vcd_double_trace(const double& object,
		     const std::string& name_,
		     const std::string& vcd_name_);
    void write(FILE* f);
    bool changed();

protected:    
    const double& object;
    double old_value;
};

vcd_double_trace::vcd_double_trace(const double& object_,
				   const std::string& name_,
				   const std::string& vcd_name_)
: vcd_trace(name_, vcd_name_), object(object_), old_value(object_)
{
    vcd_var_typ_name = "real";
    bit_width = 1;
}

bool vcd_double_trace::changed()
{
    return object != old_value;
}

void vcd_double_trace::write(FILE* f)
{
    std::fprintf(f, "r%.16g %s", object, vcd_name.c_str());
    old_value = object;
}


/*****************************************************************************/

class vcd_enum_trace : public vcd_trace {
public:
    vcd_enum_trace(const unsigned& object_,
		   const std::string& name_,
		   const std::string& vcd_name_,
		   const char** enum_literals);
    void write(FILE* f);
    bool changed();

protected:
    const unsigned& object;
    unsigned old_value;
    unsigned mask;
    const char** literals;
    unsigned nliterals;
};


vcd_enum_trace::vcd_enum_trace(const unsigned& object_,
			       const std::string& name_,
			       const std::string& vcd_name_,
			       const char** enum_literals_)
: vcd_trace(name_, vcd_name_), object(object_), old_value(object_),
  mask(0xffffffff), literals(enum_literals_), nliterals(0)
{
    // find number of bits required to represent enumeration literal - counting loop
    for (nliterals = 0; enum_literals_[nliterals]; nliterals++) continue;

    // Figure out number of bits required to represent the number of literals
    bit_width = 0;
    unsigned shifted_maxindex = nliterals-1;
    while(shifted_maxindex != 0){
        shifted_maxindex >>= 1;
        bit_width++;
    }

    // Set the mask
    if (bit_width < 32) {
      mask = ~(-1 << bit_width);
    } else {
      mask = 0xffffffff;
    }

    vcd_var_typ_name = "wire";
}       

bool vcd_enum_trace::changed()
{
    return object != old_value;
}

void vcd_enum_trace::write(FILE* f)
{
    char rawdata[1000];
    char compdata[1000];
    int bitindex;

    // Check for overflow
    if ((object & mask) != object) {
        for (bitindex = 0; bitindex < bit_width; bitindex++){
            rawdata[bitindex] = 'x';
        }
    } else {
        unsigned long bit_mask = 1ul << (bit_width-1);
        for (bitindex = 0; bitindex < bit_width; bitindex++) {
            rawdata[bitindex] = (object & bit_mask)? '1' : '0';
            bit_mask = bit_mask >> 1;
        }
    }
    rawdata[bitindex] = '\0';
    compose_data_line(rawdata, compdata);
    std::fputs(compdata, f);
    old_value = object;
}


/*****************************************************************************
           vcd_trace_file functions
 *****************************************************************************/

vcd_trace_file::vcd_trace_file(const char *name)
  : sc_trace_file_base( name, "vcd" )
  , vcd_name_index(0)
  , previous_time_units_low(0)
  , previous_time_units_high(0)
  , traces()
{}


void
vcd_trace_file::do_initialize()
{
    char buf[2000];

    //date:
    std::fprintf(fp, "$date\n     %s\n$end\n\n", localtime_string().c_str() );

    //version:
    std::fprintf(fp, "$version\n %s\n$end\n\n", sc_version());

    //timescale:
    static struct SC_TIMESCALE_TO_TEXT {
        double       unit;
        const char*  text;
    } timescale_to_text [] = {
        { sc_time(1, SC_FS).to_seconds(), "1 fs" },
        { sc_time(10, SC_FS).to_seconds(), "10 fs" },
        { sc_time(100, SC_FS).to_seconds(),"100 fs" },
        { sc_time(1, SC_PS).to_seconds(),  "1 ps" },
        { sc_time(10, SC_PS).to_seconds(), "10 ps" },
        { sc_time(100, SC_PS).to_seconds(),"100 ps" },
        { sc_time(1, SC_NS).to_seconds(),  "1 ns" },
        { sc_time(10, SC_NS).to_seconds(), "10 ns" },
        { sc_time(100, SC_NS).to_seconds(),"100 ns" },
        { sc_time(1, SC_US).to_seconds(),  "1 us" },
        { sc_time(10, SC_US).to_seconds(), "10 us" },
        { sc_time(100, SC_US).to_seconds(),"100 us" },
        { sc_time(1, SC_MS).to_seconds(),  "1 ms" },
        { sc_time(10, SC_MS).to_seconds(), "10 ms" },
        { sc_time(100, SC_MS).to_seconds(),"100 ms" },
        { sc_time(1, SC_SEC).to_seconds(),  "1 sec" },
        { sc_time(10, SC_SEC).to_seconds(), "10 sec" },
        { sc_time(100, SC_SEC).to_seconds(),"100 sec" }
    };
    static int timescale_to_text_n =
        sizeof(timescale_to_text)/sizeof(SC_TIMESCALE_TO_TEXT);

    for ( int time_i = 0; time_i < timescale_to_text_n; time_i++ )
    {
        if (timescale_unit == timescale_to_text[time_i].unit)
        {
            std::fprintf(fp,"$timescale\n     %s\n$end\n\n",
                timescale_to_text[time_i].text);
            break;
        }
    }

    // Create a dummy scope
    std::fputs("$scope module SystemC $end\n", fp);

    //variable definitions:
    for (int i = 0; i < (int)traces.size(); i++) {
        vcd_trace* t = traces[i];
        t->set_width(); // needed for all vectors
        t->print_variable_declaration_line(fp);
    }

    std::fputs("$upscope $end\n", fp);

    std::fputs("$enddefinitions  $end\n\n", fp);

    // double inittime = sc_simulation_time();
    double inittime = sc_time_stamp().to_seconds();
    
    std::sprintf(buf,
            "All initial values are dumped below at time "
            "%g sec = %g timescale units.",
            inittime, inittime/timescale_unit
            );
    write_comment(buf);

    double_to_special_int64(inittime/timescale_unit,
                            &previous_time_units_high,
                            &previous_time_units_low );


    std::fputs("$dumpvars\n",fp);
    for (int i = 0; i < (int)traces.size(); i++) {
        traces[i]->write(fp);
        std::fputc('\n', fp);
    }
    std::fputs("$end\n\n", fp);
}


// ----------------------------------------------------------------------------

#define DEFN_TRACE_METHOD(tp)                                                 \
void                                                                          \
vcd_trace_file::trace(const tp& object_, const std::string& name_)            \
{                                                                             \
    if( add_trace_check(name_) )                                              \
        traces.push_back( new vcd_ ## tp ## _trace( object_,                  \
                                                    name_,                    \
                                                    obtain_name() ) );        \
}

DEFN_TRACE_METHOD(bool)
DEFN_TRACE_METHOD(float)
DEFN_TRACE_METHOD(double)

#undef DEFN_TRACE_METHOD
#define DEFN_TRACE_METHOD(tp)                                                 \
void                                                                          \
vcd_trace_file::trace(const sc_dt::tp& object_, const std::string& name_)     \
{                                                                             \
    if( add_trace_check(name_) )                                              \
        traces.push_back( new vcd_ ## tp ## _trace( object_,                  \
                                                    name_,                    \
                                                    obtain_name() ) );        \
}

DEFN_TRACE_METHOD(sc_bit)
DEFN_TRACE_METHOD(sc_logic)

DEFN_TRACE_METHOD(sc_signed)
DEFN_TRACE_METHOD(sc_unsigned)
DEFN_TRACE_METHOD(sc_int_base)
DEFN_TRACE_METHOD(sc_uint_base)

DEFN_TRACE_METHOD(sc_fxval)
DEFN_TRACE_METHOD(sc_fxval_fast)
DEFN_TRACE_METHOD(sc_fxnum)
DEFN_TRACE_METHOD(sc_fxnum_fast)

#undef DEFN_TRACE_METHOD


#define DEFN_TRACE_METHOD_SIGNED(tp)                                          \
void                                                                          \
vcd_trace_file::trace( const tp&          object_,                            \
                       const std::string& name_,                              \
                       int                width_ )                            \
{                                                                             \
    if( add_trace_check(name_) )                                              \
        traces.push_back( new vcd_signed_ ## tp ## _trace( object_,           \
                                                           name_,             \
                                                           obtain_name(),     \
                                                           width_ ) );        \
}

#define DEFN_TRACE_METHOD_UNSIGNED(tp)                                        \
void                                                                          \
vcd_trace_file::trace( const unsigned tp& object_,                            \
                       const std::string& name_,                              \
                       int                width_ )                            \
{                                                                             \
    if( add_trace_check(name_) )                                              \
        traces.push_back( new vcd_unsigned_ ## tp ## _trace( object_,         \
                                                             name_,           \
                                                             obtain_name(),   \
                                                             width_ ) );      \
}

DEFN_TRACE_METHOD_SIGNED(char)
DEFN_TRACE_METHOD_SIGNED(short)
DEFN_TRACE_METHOD_SIGNED(int)
DEFN_TRACE_METHOD_SIGNED(long)

DEFN_TRACE_METHOD_UNSIGNED(char)
DEFN_TRACE_METHOD_UNSIGNED(short)
DEFN_TRACE_METHOD_UNSIGNED(int)
DEFN_TRACE_METHOD_UNSIGNED(long)

#undef DEFN_TRACE_METHOD_SIGNED
#undef DEFN_TRACE_METHOD_UNSIGNED

#define DEFN_TRACE_METHOD_LONG_LONG(tp)                                       \
void                                                                          \
vcd_trace_file::trace( const sc_dt::tp&   object_,                            \
                       const std::string& name_,                              \
                       int                width_ )                            \
{                                                                             \
    if( add_trace_check(name_) )                                              \
        traces.push_back( new vcd_ ## tp ## _trace( object_,                  \
                                                    name_,                    \
                                                    obtain_name(),            \
                                                    width_ ) );               \
}

DEFN_TRACE_METHOD_LONG_LONG(int64)
DEFN_TRACE_METHOD_LONG_LONG(uint64)

#undef DEFN_TRACE_METHOD_LONG_LONG

void
vcd_trace_file::trace( const unsigned&    object_,
                       const std::string& name_,
                       const char**       enum_literals_ )
{
    if( add_trace_check(name_) )
        traces.push_back( new vcd_enum_trace( object_,
                                              name_,
                                              obtain_name(),
                                              enum_literals_ ) );
}


void
vcd_trace_file::write_comment(const std::string& comment)
{
    if(!fp) open_fp();
    //no newline in comments allowed, as some viewers may crash
    std::fputs("$comment\n", fp);
    std::fputs(comment.c_str(), fp);
    std::fputs("\n$end\n\n", fp);
}

void
vcd_trace_file::cycle(bool this_is_a_delta_cycle)
{
    unsigned this_time_units_high, this_time_units_low;

    // Just to make g++ shut up in the optimized mode
    this_time_units_high = this_time_units_low = 0;

    // Trace delta cycles only when enabled
    if (!delta_cycles() && this_is_a_delta_cycle) return;

    // Check for initialization
    if( initialize() ) {
        return;
    };


    double now_units = sc_time_stamp().to_seconds() / timescale_unit;
    unsigned now_units_high, now_units_low;
    double_to_special_int64(now_units, &now_units_high, &now_units_low );

    bool now_later_than_previous_time = false;
    if( (now_units_low > previous_time_units_low 
        && now_units_high == previous_time_units_high)
        || now_units_high > previous_time_units_high){
        now_later_than_previous_time = true;
    }

    bool now_equals_previous_time = false;
    if(now_later_than_previous_time){
        this_time_units_high = now_units_high;
        this_time_units_low = now_units_low;
    } else {
        if( now_units_low == previous_time_units_low
	    && now_units_high == previous_time_units_high){
	    now_equals_previous_time = true;
            this_time_units_high = now_units_high;
            this_time_units_low = now_units_low;
	}
    }

    // Since VCD does not understand 0 time progression, we have to fake
    // delta cycles with progressing time by one unit
    if(this_is_a_delta_cycle){
        this_time_units_high = previous_time_units_high;
        this_time_units_low = previous_time_units_low + 1;
        if(this_time_units_low == 1000000000){
            this_time_units_high++;
            this_time_units_low=0;
        }
        static bool warned = false;
        if(!warned){
            SC_REPORT_INFO( SC_ID_TRACING_VCD_DELTA_CYCLE_
                          , sc_time( timescale_unit, SC_SEC )
                              .to_string().c_str() );
            warned = true;
        }
    }


    // Not a delta cycle and time has not progressed
    if( ! this_is_a_delta_cycle && now_equals_previous_time &&
	( now_units_high != 0 || now_units_low != 0 ) ) {
	// Don't print the message at time zero
        static bool warned = false;
        if( ! warned ) {
            std::stringstream ss;
            ss << "units count: " << now_units_low << "\n"
               "\tWaveform viewers will only show the states of the last one.\n"
               "\tUse `tracefile->set_time_unit(double, sc_time_unit);'"
                  " to increase the time resolution.";
            SC_REPORT_WARNING( SC_ID_TRACING_VCD_DUPLICATE_TIME_
                             , ss.str().c_str() );
            // warned = true;
        }
    }

    // Not a delta cycle and time has gone backward
    // This will happen with large number of delta cycles between two real
    // advances of time
    if(!this_is_a_delta_cycle && !now_equals_previous_time && 
        !now_later_than_previous_time){
        static bool warned = false;
        if(!warned) {
            std::stringstream ss;
            ss << "units count (" 
               << previous_time_units_low << "->" << now_units_low << ")\n"
               "\tThis can occur when delta cycling is activated."
                  " Cycles with falling time are not shown.\n"
               "\tUse `tracefile->set_time_unit(double, sc_time_unit);'"
                  " to increase the time resolution.";
            SC_REPORT_WARNING( SC_ID_TRACING_VCD_DUPLICATE_TIME_
                             , ss.str().c_str() );
            // warned = true;
        }
        // Note that we don't set this_time_units_high/low to any value only
        // in this case because we are not going to do any tracing. In the
        // optimized mode, the compiler complains because of this. Therefore,
        // we include the lines at the very beginning of this function to make
        // the compiler shut up.
        return; 
    }

    // Now do the actual printing 
    bool time_printed = false;
    vcd_trace* const* const l_traces = &traces[0];
    for (int i = 0; i < (int)traces.size(); i++) {
        vcd_trace* t = l_traces[i];
        if(t->changed()){
            if(time_printed == false){
                char buf[200];
                if(this_time_units_high){
                    std::sprintf(buf, "#%u%09u", this_time_units_high, this_time_units_low);
                }
                else{ 
                    std::sprintf(buf, "#%u", this_time_units_low);
                }
                std::fputs(buf, fp);
                std::fputc('\n', fp);
                time_printed = true;
            }

	    // Write the variable
            t->write(fp);
            std::fputc('\n', fp);
        }
    }
    // Put another newline after all values are printed
    if(time_printed) std::fputc('\n', fp);

    if(time_printed){
        // We update previous_time_units only when we print time because
        // this field stores the previous time that was printed, not the
        // previous time this function was called
        previous_time_units_high = this_time_units_high;
        previous_time_units_low = this_time_units_low;
    }
}

#if 0
void
vcd_trace_file::create_vcd_name(std::string* p_destination)
{
    obtain_name.swap( *p_destination );
}
#endif

// Create a VCD name for a variable
std::string
vcd_trace_file::obtain_name()
{
    const char first_type_used = 'a';
    const int used_types_count = 'z' - 'a' + 1;
    int result;

    result = vcd_name_index;
    char char6 = (char)(vcd_name_index % used_types_count);

    result = result / used_types_count;
    char char5 = (char)(result % used_types_count);

    result = result / used_types_count;
    char char4 = (char)(result % used_types_count);

    result = result / used_types_count;
    char char3 = (char)(result % used_types_count);

    result = result / used_types_count;
    char char2 = (char)(result % used_types_count);

    char buf[20];
    std::sprintf(buf, "%c%c%c%c%c",
            char2 + first_type_used,
            char3 + first_type_used,
            char4 + first_type_used,
            char5 + first_type_used,
            char6 + first_type_used);
    vcd_name_index++;
    return std::string(buf);
}

vcd_trace_file::~vcd_trace_file()
{
    for( int i = 0; i < (int)traces.size(); i++ ) {
        vcd_trace* t = traces[i];
        delete t;
    }
}


// Functions specific to VCD tracing

static char
map_sc_logic_state_to_vcd_state(char in_char)
{
    char out_char;

    switch(in_char){
        case 'U':
        case 'X': 
        case 'W':
        case 'D':
            out_char = 'x';
            break;
        case '0':
        case 'L':
            out_char = '0';
            break;
        case  '1':
        case  'H': 
            out_char = '1';
            break;
        case  'Z': 
            out_char = 'z';
            break;
        default:
            out_char = '?';
    }

    return out_char;
}




static
void
remove_vcd_name_problems(vcd_trace const* vcd, std::string& name)
{
    static bool warned = false;
    bool braces_removed = false;
    for (unsigned int i = 0; i< name.length(); i++) {
      if (name[i] == '[') {
	name[i] = '(';
	braces_removed = true;
      }
      else if (name[i] == ']') {
	name[i] = ')';
	braces_removed = true;
      }
    }

    if(braces_removed && !warned){
        std::stringstream ss;
        ss << vcd->name << ":\n"
            "\tTraced objects found with name containing [], which may be\n"
            "\tinterpreted by the waveform viewer in unexpected ways.\n"
            "\tSo the [] is automatically replaced by ().";

        SC_REPORT_WARNING( SC_ID_TRACING_OBJECT_NAME_FILTERED_
                         , ss.str().c_str() );
    }
}

// ----------------------------------------------------------------------------

sc_trace_file*
sc_create_vcd_trace_file(const char * name)
{
    sc_trace_file * tf = new vcd_trace_file(name);
    return tf;
}

void
sc_close_vcd_trace_file( sc_trace_file* tf )
{
    vcd_trace_file* vcd_tf = static_cast<vcd_trace_file*>(tf);
    delete vcd_tf; 
}

} // namespace sc_core
