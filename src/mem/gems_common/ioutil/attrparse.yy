/*
    Copyright (C) 1999-2005 by Mark D. Hill and David A. Wood for the
    Wisconsin Multifacet Project.  Contact: gems@cs.wisc.edu
    http://www.cs.wisc.edu/gems/

    --------------------------------------------------------------------

    This file a component of the Multifacet GEMS (General
    Execution-driven Multiprocessor Simulator) software toolset
    originally developed at the University of Wisconsin-Madison.

    Ruby was originally developed primarily by Milo Martin and Daniel
    Sorin with contributions from Ross Dickson, Carl Mauer, and Manoj
    Plakal.

    SLICC was originally developed by Milo Martin with substantial
    contributions from Daniel Sorin.

    Opal was originally developed by Carl Mauer based upon code by
    Craig Zilles.

    Substantial further development of Multifacet GEMS at the
    University of Wisconsin was performed by Alaa Alameldeen, Brad
    Beckmann, Ross Dickson, Pacia Harper, Milo Martin, Michael Marty,
    Carl Mauer, Kevin Moore, Manoj Plakal, Daniel Sorin, Min Xu, and
    Luke Yen.

    --------------------------------------------------------------------

    If your use of this software contributes to a published paper, we
    request that you (1) cite our summary paper that appears on our
    website (http://www.cs.wisc.edu/gems/) and (2) e-mail a citation
    for your published paper to gems@cs.wisc.edu.

    If you redistribute derivatives of this software, we request that
    you notify us and either (1) ask people to register with us at our
    website (http://www.cs.wisc.edu/gems/) or (2) collect registration
    information and periodically send it to us.

    --------------------------------------------------------------------

    Multifacet GEMS is free software; you can redistribute it and/or
    modify it under the terms of version 2 of the GNU General Public
    License as published by the Free Software Foundation.

    Multifacet GEMS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with the Multifacet GEMS; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
    02111-1307, USA

    The GNU General Public License is contained in the file LICENSE.

### END HEADER ###
*/



%{
/*------------------------------------------------------------------------*/
/* Includes                                                               */
/*------------------------------------------------------------------------*/

#ifdef IS_RUBY
#include "Global.hh"
#endif

using namespace std;
#include <string>
#include <map>
#include <stdlib.h>

// Maurice
// extern "C" {
// #include "simics/api.h"
// };

#include "FakeSimicsDataTypes.hh"

#include "confio.hh"

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
