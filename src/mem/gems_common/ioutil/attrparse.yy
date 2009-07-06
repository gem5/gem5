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

%{
/*------------------------------------------------------------------------*/
/* Includes                                                               */
/*------------------------------------------------------------------------*/

#include "mem/ruby/common/Global.hh"

using namespace std;
#include <string>
#include <map>
#include <stdlib.h>

// Maurice
// extern "C" {
// #include "simics/api.hh"
// };

#include "mem/gems_common/ioutil/FakeSimicsDataTypes.hh"

#include "mem/gems_common/ioutil/confio.hh"

// CM FIX: if I wasn't working on a paper: I'd re-write the grammer to
//         be left (or right) recursive, which ever is more efficient
//         This only affects extremely large cache sizes / BP sizes on read in.
#define YYMAXDEPTH 100000

extern char* attext;

extern void aterror(const char *);
extern int  atlex();
attr_value_t    g_attr_map;

extern  void fprintAttr( FILE *fp, attr_value_t attr );

%}

/*------------------------------------------------------------------------*/
/* Union declarations                                                     */
/*------------------------------------------------------------------------*/
// The types of the tokens and nonterminals
%union {
  attr_value_t  *attrval;
};

%token <attrval> STRING INTEGER
%token           MY_END LPAREN RPAREN

%type  <attrval> confmapping confpair attributes attrlist
%%

conffile      : confmapping
{
  g_attr_map = *($1);
  free( $1 )
}

confmapping   : confmapping confpair
{
  attr_value_t  *newattr = mallocAttribute(1);
  newattr->kind          = Sim_Val_List;
  if ( $1 == NULL ) {
    newattr->u.list.size   = 1;
    newattr->u.list.vector = $2;
  } else {
    // add the latest mapping to the return mapping
    uint32         newsize = $1->u.list.size + 1;
    attr_value_t  *vector  = mallocAttribute( newsize );
    newattr->u.list.size   = newsize;
    newattr->u.list.vector = vector;
    for (uint32 i = 0; i < newsize - 1; i++) {
      vector[i] = $1->u.list.vector[i];
    }
    vector[newsize - 1] = *($2);
    free( $1->u.list.vector );
    free( $1 );
    free( $2 );
  }
  $$ = newattr;
}
                | // nothing
{
  $$ = NULL;
}

confpair       : STRING ':' attributes
{
  attr_value_t  *newattr = mallocAttribute(1);
  newattr->kind          = Sim_Val_List;
  newattr->u.list.size   = 2;
  newattr->u.list.vector = mallocAttribute(2);
  newattr->u.list.vector[0] = *($1);
  newattr->u.list.vector[1] = *($3);
  free( $1 );
  free( $3 );
  $$ = newattr;
}

attributes     : INTEGER
{
  $$ = $1;
}
                | STRING
{
  $$ = $1;
}
                | LPAREN attrlist RPAREN
{
  attr_value_t  *newattr = mallocAttribute(1);
  newattr->kind          = Sim_Val_List;
  if ( $2->kind != CONF_ATTR_SINGLE &&
       $2->kind != CONF_ATTR_PAIR ) {
    newattr->u.list.size   = 1;
    newattr->u.list.vector = $2;
  } else {
    newattr->u.list.size   = $2->u.list.size;
    newattr->u.list.vector = mallocAttribute(newattr->u.list.size);
    attr_value_t  *curattr = $2;
    uint32 i = 0;
    while ( i < newattr->u.list.size ) {
      if (curattr->kind == CONF_ATTR_SINGLE) {
        newattr->u.list.vector[i] = curattr->u.list.vector[0];
        i++;

        curattr = NULL;
      } else if (curattr->kind == CONF_ATTR_PAIR) {
        newattr->u.list.vector[i] = curattr->u.list.vector[0];
        i++;
        if ( i < newattr->u.list.size )
          curattr = &(curattr->u.list.vector[1]);
        else
          curattr = NULL;

      } else {
        ERROR_OUT("error: unknown kind in pair: %d\n", curattr->kind);
        ASSERT(0);
      }
    }
    // FIX memory leak: around 600 KB
    // freeAttribute( $2 );  // with gcc-3.4 this free call tries to free memory from the stack
  }
  $$ = newattr;
}

attrlist        : attributes
{
  attr_value_t  *newattr = mallocAttribute(1);
  newattr->kind          = CONF_ATTR_SINGLE;
  newattr->u.list.size    = 1;
  newattr->u.list.vector  = $1;
  $$ = newattr;
}
                | attributes ',' attrlist
{
    // allocate the pair ( x , y ) attribute
    attr_value_t  *newattr = mallocAttribute(1);
    int            newsize = $3->u.list.size + 1;
    attr_value_t  *vector  = mallocAttribute(2);
    newattr->kind          = CONF_ATTR_PAIR;
    newattr->u.list.size   = newsize;
    newattr->u.list.vector = vector;

    // assign the LH attribute
    vector[0] = *($1);
    vector[1] = *($3);
    free( $1 );
    free( $3 );
    $$ = newattr;
}
