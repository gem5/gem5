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

%option noyywrap

ALPHADIGIT      [^\:\,\(\)\n\t\(\) \0\#]
HEXDIGIT        [0-9a-fA-Fx]
NEWLINE         [\n]
WHITESPACE      [ \t]

%{

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

// CM: simics 1.6.5 API redefines fwrite, much to my chagrin
#undef   fwrite
#undef   printf
#include "mem/gems_common/ioutil/attrparse.hh"

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

