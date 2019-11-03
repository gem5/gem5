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

  global.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#ifndef GLOBALH
#define GLOBALH


// #include <iostream.h>
#include <stdio.h>

#define MAXBUFLEN 64

// cos constants, factor 512 
#define c1d4 362L
#define c1d8 473L
#define c3d8 196L
#define c1d16 502L
#define c3d16 426L
#define c5d16 284L
#define c7d16 100L

// correct digits 
#define MSCALE(expr)   (COEFF)((expr)>>9)

typedef unsigned char  BYTE;
typedef unsigned short WORD;
typedef unsigned long  DWORD;

typedef BYTE  BLOCK[8][8];
typedef BYTE  COMPRESSED[MAXBUFLEN];
typedef WORD  MATRIX64x12[64];

// type of the coefficient arrays 
typedef short COEFF;

// typedefs for huffman tables 
typedef struct {
	BYTE size;
	WORD code;
} HUFFMTBL_ENTRY;


struct Block {
  BYTE b[8][8];
  
  Block();
  Block(BLOCK *);
  Block(const Block&);
  void operator=(const Block&);
  int operator==(const Block&) const;

  BYTE   get(int x, int y) const;
  void   put(int x, int y, BYTE val);
  BLOCK* get_ptr() const;
};

struct Compressed {
  BYTE c[MAXBUFLEN];
  
  Compressed();
  Compressed(const Compressed&);
  void operator=(const Compressed&);
  int operator==(const Compressed&) const;

  void clear();
  BYTE get(int x) const;
  void put(int x, BYTE val);
};

struct  Matrix64x12 {
  WORD m[64];
  
  Matrix64x12();
  Matrix64x12(const Matrix64x12&);
  void operator=(const Matrix64x12&);
  int operator==(const Matrix64x12&) const;

  WORD get(int x) const;
  void put(int x, WORD val);
};

struct Coeff8 {
  COEFF c[8];
  
  Coeff8();
  Coeff8(const Coeff8&);
  void operator=(const Coeff8&);
  int operator==(const Coeff8&) const;

  COEFF get(int x) const;
  void put(int x, COEFF val);
};

struct Coeff8x8 {
  COEFF c[8][8];
  
  Coeff8x8();
  Coeff8x8(const Coeff8x8&);
  void operator=(const Coeff8x8&);
  int operator==(const Coeff8x8&) const;

  COEFF get(int x, int y) const;
  void  put(int x, int y, COEFF val);
};

inline
void
sc_trace( sc_trace_file*, const Coeff8x8&, const std::string& )
{
    // NOT IMPLEMENTED
}


// quantization table 8-bit unsigned integer
static const unsigned char coeff_quant[8][8] = {  // v is row 
    {   16,   11,   10,   16,   24,   40,   51,   61},
    {   12,   12,   14,   19,   26,   58,   60,   55},
    {   14,   13,   16,   24,   40,   57,   69,   56},
    {   14,   17,   22,   29,   51,   87,   80,   82},
    {   18,   22,   37,   56,   68,  109,  103,   77},
    {   24,   35,   55,   64,   81,  104,  113,   92},
    {   99,   64,   78,   87,  103,  121,  120,  101},
    {   72,   92,   95,   98,  112,  100,  103,   99}
};


// table of Huffman DC coefficients 
static const HUFFMTBL_ENTRY huffm_dc[12] = {
    {  2, 0X0000 }, {  3, 0X0002 }, {  3, 0X0003 }, {  3, 0X0004 },
    {  3, 0X0005 }, {  3, 0X0006 }, {  4, 0X000E }, {  5, 0X001E },
    {  6, 0X003E }, {  7, 0X007E }, {  8, 0X00FE }, {  9, 0X01FE } 
};


// table of Huffman AC coefficients 
static const HUFFMTBL_ENTRY huffm_ac[256] = {
    {  4, 0x000a }, {  2, 0x0000 }, {  2, 0x0001 }, {  3, 0x0004 }, 
    {  4, 0x000b }, {  5, 0x001a }, {  7, 0x0078 }, {  8, 0x00f8 }, 
    { 10, 0x03f6 }, { 16, 0xff82 }, { 16, 0xff83 }, {  0, 0x0000 }, 
    {  0, 0x0000 }, {  0, 0x0000 }, {  0, 0x0000 }, {  0, 0x0000 }, 
    {  0, 0x0000 }, {  4, 0x000c }, {  5, 0x001b }, {  7, 0x0079 }, 
    {  9, 0x01f6 }, { 11, 0x07f6 }, { 16, 0xff84 }, { 16, 0xff85 }, 
    { 16, 0xff86 }, { 16, 0xff87 }, { 16, 0xff88 }, {  0, 0x0000 }, 
    {  0, 0x0000 }, {  0, 0x0000 }, {  0, 0x0000 }, {  0, 0x0000 }, 
    {  0, 0x0000 }, {  5, 0x001c }, {  8, 0x00f9 }, { 10, 0x03f7 }, 
    { 12, 0x0ff4 }, { 16, 0xff89 }, { 16, 0xff8a }, { 16, 0xff8b }, 
    { 16, 0xff8c }, { 16, 0xff8d }, { 16, 0xff8e }, {  0, 0x0000 }, 
    {  0, 0x0000 }, {  0, 0x0000 }, {  0, 0x0000 }, {  0, 0x0000 }, 
    {  0, 0x0000 }, {  6, 0x003a }, {  9, 0x01f7 }, { 12, 0x0ff5 }, 
    { 16, 0xff8f }, { 16, 0xff90 }, { 16, 0xff91 }, { 16, 0xff92 }, 
    { 16, 0xff93 }, { 16, 0xff94 }, { 16, 0xff95 }, {  0, 0x0000 }, 
    {  0, 0x0000 }, {  0, 0x0000 }, {  0, 0x0000 }, {  0, 0x0000 }, 
    {  0, 0x0000 }, {  6, 0x003b }, { 10, 0x03f8 }, { 16, 0xff96 }, 
    { 16, 0xff97 }, { 16, 0xff98 }, { 16, 0xff99 }, { 16, 0xff9a }, 
    { 16, 0xff9b }, { 16, 0xff9c }, { 16, 0xff9d }, {  0, 0x0000 }, 
    {  0, 0x0000 }, {  0, 0x0000 }, {  0, 0x0000 }, {  0, 0x0000 }, 
    {  0, 0x0000 }, {  7, 0x007a }, { 11, 0x07f7 }, { 16, 0xff9e }, 
    { 16, 0xff9f }, { 16, 0xffa0 }, { 16, 0xffa1 }, { 16, 0xffa2 }, 
    { 16, 0xffa3 }, { 16, 0xffa4 }, { 16, 0xffa5 }, {  0, 0x0000 }, 
    {  0, 0x0000 }, {  0, 0x0000 }, {  0, 0x0000 }, {  0, 0x0000 }, 
    {  0, 0x0000 }, {  7, 0x007b }, { 12, 0x0ff6 }, { 16, 0xffa6 }, 
    { 16, 0xffa7 }, { 16, 0xffa8 }, { 16, 0xffa9 }, { 16, 0xffaa }, 
    { 16, 0xffab }, { 16, 0xffac }, { 16, 0xffad }, {  0, 0x0000 }, 
    {  0, 0x0000 }, {  0, 0x0000 }, {  0, 0x0000 }, {  0, 0x0000 }, 
    {  0, 0x0000 }, {  8, 0x00fa }, { 12, 0x0ff7 }, { 16, 0xffae }, 
    { 16, 0xffaf }, { 16, 0xffb0 }, { 16, 0xffb1 }, { 16, 0xffb2 }, 
    { 16, 0xffb3 }, { 16, 0xffb4 }, { 16, 0xffb5 }, {  0, 0x0000 }, 
    {  0, 0x0000 }, {  0, 0x0000 }, {  0, 0x0000 }, {  0, 0x0000 }, 
    {  0, 0x0000 }, {  9, 0x01f8 }, { 15, 0x7fc0 }, { 16, 0xffb6 }, 
    { 16, 0xffb7 }, { 16, 0xffb8 }, { 16, 0xffb9 }, { 16, 0xffba }, 
    { 16, 0xffbb }, { 16, 0xffbc }, { 16, 0xffbd }, {  0, 0x0000 }, 
    {  0, 0x0000 }, {  0, 0x0000 }, {  0, 0x0000 }, {  0, 0x0000 }, 
    {  0, 0x0000 }, {  9, 0x01f9 }, { 16, 0xffbe }, { 16, 0xffbf }, 
    { 16, 0xffc0 }, { 16, 0xffc1 }, { 16, 0xffc2 }, { 16, 0xffc3 }, 
    { 16, 0xffc4 }, { 16, 0xffc5 }, { 16, 0xffc6 }, {  0, 0x0000 }, 
    {  0, 0x0000 }, {  0, 0x0000 }, {  0, 0x0000 }, {  0, 0x0000 }, 
    {  0, 0x0000 }, {  9, 0x01fa }, { 16, 0xffc7 }, { 16, 0xffc8 }, 
    { 16, 0xffc9 }, { 16, 0xffca }, { 16, 0xffcb }, { 16, 0xffcc }, 
    { 16, 0xffcd }, { 16, 0xffce }, { 16, 0xffcf }, {  0, 0x0000 }, 
    {  0, 0x0000 }, {  0, 0x0000 }, {  0, 0x0000 }, {  0, 0x0000 }, 
    {  0, 0x0000 }, { 10, 0x03f9 }, { 16, 0xffd0 }, { 16, 0xffd1 }, 
    { 16, 0xffd2 }, { 16, 0xffd3 }, { 16, 0xffd4 }, { 16, 0xffd5 }, 
    { 16, 0xffd6 }, { 16, 0xffd7 }, { 16, 0xffd8 }, {  0, 0x0000 }, 
    {  0, 0x0000 }, {  0, 0x0000 }, {  0, 0x0000 }, {  0, 0x0000 }, 
    {  0, 0x0000 }, { 10, 0x03fa }, { 16, 0xffd9 }, { 16, 0xffda }, 
    { 16, 0xffdb }, { 16, 0xffdc }, { 16, 0xffdd }, { 16, 0xffde }, 
    { 16, 0xffdf }, { 16, 0xffe0 }, { 16, 0xffe1 }, {  0, 0x0000 }, 
    {  0, 0x0000 }, {  0, 0x0000 }, {  0, 0x0000 }, {  0, 0x0000 }, 
    {  0, 0x0000 }, { 11, 0x07f8 }, { 16, 0xffe2 }, { 16, 0xffe3 }, 
    { 16, 0xffe4 }, { 16, 0xffe5 }, { 16, 0xffe6 }, { 16, 0xffe7 }, 
    { 16, 0xffe8 }, { 16, 0xffe9 }, { 16, 0xffea }, {  0, 0x0000 }, 
    {  0, 0x0000 }, {  0, 0x0000 }, {  0, 0x0000 }, {  0, 0x0000 }, 
    {  0, 0x0000 }, { 16, 0xffeb }, { 16, 0xffec }, { 16, 0xffed }, 
    { 16, 0xffee }, { 16, 0xffef }, { 16, 0xfff0 }, { 16, 0xfff1 }, 
    { 16, 0xfff2 }, { 16, 0xfff3 }, { 16, 0xfff4 }, {  0, 0x0000 }, 
    {  0, 0x0000 }, {  0, 0x0000 }, {  0, 0x0000 }, {  0, 0x0000 }, 
    { 11, 0x07f9 }, { 16, 0xfff5 }, { 16, 0xfff6 }, { 16, 0xfff7 }, 
    { 16, 0xfff8 }, { 16, 0xfff9 }, { 16, 0xfffa }, { 16, 0xfffb }, 
    { 16, 0xfffc }, { 16, 0xfffd }, { 16, 0xfffe }, {  0, 0x0000 }, 
    {  0, 0x0000 }, {  0, 0x0000 }, {  0, 0x0000 }, {  0, 0x0000 } 
  };


#endif

