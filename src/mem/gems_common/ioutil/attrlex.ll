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


%option noyywrap

ALPHADIGIT      [^\:\,\(\)\n\t\(\) \0\#]
HEXDIGIT        [0-9a-fA-Fx]
NEWLINE         [\n]
WHITESPACE      [ \t]

%{

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

// CM: simics 1.6.5 API redefines fwrite, much to my chagrin
#undef   fwrite
#undef   printf
#include "attrparse.h"

#define MAX_INCLUDE_DEPTH 10

/** global result of parsing file */
extern attr_value_t g_attr_map;

extern int  atparse(void);

static int linenum=1;          /* the current line number */
static int colnum=1;           /* the current column number */
static YY_BUFFER_STATE  include_stack[MAX_INCLUDE_DEPTH];
static int              include_stack_ptr = 0;
static char             g_relative_include_path[256];


// forward declaration of aterror
void aterror(const char *msg);
%}

%x SLASHCOMMENT INCLUDE

%%

%{ /* PATTERNS FOR STRING TOKENS */
%}

"//".*[\n] { linenum++; colnum=1; }  /* C++ style comments */

\#include                   { colnum+=yyleng; BEGIN(INCLUDE); }
<INCLUDE>{WHITESPACE}*      { colnum+=yyleng; }
<INCLUDE>[^ \t\n]+          {
                    // should really be FILEIO_MAX_FILENAME or MAX_NAME
                    char str[256];
                    if ( include_stack_ptr >= MAX_INCLUDE_DEPTH )
                      {
                        ERROR_OUT( "Includes nested too deeply" );
                        exit( 1 );
                      }
                    include_stack[include_stack_ptr++] = YY_CURRENT_BUFFER;

                    yyin = fopen( yytext, "r" );
                    if ( ! yyin ) {
                      sprintf( str, "%s%s", g_relative_include_path, yytext );
                      yyin = fopen( str, "r" );
                    }
                    if ( ! yyin ) {
                      sprintf( str, "%s%s%s", g_relative_include_path, "config/", yytext );
                      yyin = fopen( str, "r" );
                    }
                    if ( ! yyin ) {
                      ERROR_OUT("unable to open included file: %s or %s\n", yytext, str);
                      aterror("file open error.\n");
                    }
                    yy_switch_to_buffer(yy_create_buffer( yyin, YY_BUF_SIZE ));
                    BEGIN(INITIAL);
                            }
<<EOF>>                     {
                      if ( --include_stack_ptr < 0 ) {
                        yyterminate();
                      } else {
                        yy_delete_buffer( YY_CURRENT_BUFFER );
                        fclose(yyin);
                        yy_switch_to_buffer(include_stack[include_stack_ptr] );
                      }
                            }

\(                          { colnum+=yyleng; return (LPAREN); }
\)                          { colnum+=yyleng; return (RPAREN); }
\:                          { colnum+=yyleng; return (':'); }
\,                          { colnum+=yyleng; return (','); }
{HEXDIGIT}+                 {
                              colnum+=yyleng;
                              attr_value_t *val = (attr_value_t *)
                                       malloc( sizeof(attr_value_t) );
                              memset( val, 0, sizeof(attr_value_t) );
                              atlval.attrval = val;
                              val->kind = Sim_Val_Integer;
                              val->u.integer = strtoull( yytext, NULL, 0 );
                              return (INTEGER); }
{ALPHADIGIT}+               {
                              colnum+=yyleng;
                              attr_value_t *val = (attr_value_t *)
                                       malloc( sizeof(attr_value_t) );
                              memset( val, 0, sizeof(attr_value_t) );
                              atlval.attrval = val;
                              val->kind = Sim_Val_String;
                              val->u.string = strdup(yytext);
                              return (STRING); }

%{ /* OTHER PATTERNS */
%}

{WHITESPACE}+               {colnum += yyleng;}
{NEWLINE}                   {linenum++; colnum = 1;}

%%

extern "C" void parseInitialize( void )
{
  // since no global variables are set in simics, we must do it manually
  // this is also necessary now that the parser can be used more than once.
  // (it is used to parse the defaults, and can be used after that)
  linenum = 1;
  colnum = 1;
  include_stack_ptr = 0;
}

extern "C" int parseAttrFile( FILE *inputFile, const char *relative_include_path, attr_value_t *myTable )
{
  parseInitialize();
  strncpy( g_relative_include_path, relative_include_path, 255 );

  int result;
  yyin = inputFile;
  YY_BUFFER_STATE scan_state = yy_create_buffer( yyin, YY_BUF_SIZE );
  yy_switch_to_buffer( scan_state );
  result = atparse();
  *myTable = g_attr_map;
  yy_delete_buffer( scan_state );
  return (result);
}

extern "C" int parseAttrString( const char *str, attr_value_t *myTable )
{
  parseInitialize();

  int result;
  YY_BUFFER_STATE scan_state = yy_scan_string( str );
  result = atparse();
  *myTable = g_attr_map;
  yy_delete_buffer( scan_state );
  return (result);
}

extern void aterror(const char *msg)
{
  ERROR_OUT("%d:%d: ERROR while parsing config file%s\n", linenum, colnum, msg );
}

