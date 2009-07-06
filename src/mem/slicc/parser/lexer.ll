
/*
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
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

/*
 * $Id$
 */

%{

#include <assert.h>
#include "mem/slicc/ast/ASTs.hh"
#include "parser.hh"
#include <string>

extern "C" int yylex();
extern "C" void yyerror();
extern "C" int yywrap()
{
    return 1;
}

%}
%x CMNT
%x IMBEDED
%%

[\t ]+   /* Ignore whitespace */
[\n]       { g_line_number++; }
"//".*[\n] { g_line_number++; }  /* C++ style comments */

"/*" BEGIN CMNT;
<CMNT>. ;
<CMNT>\n { g_line_number++; }
<CMNT>"*/" { BEGIN INITIAL; }

true       { yylval.str_ptr = new string(yytext); return LIT_BOOL; }
false      { yylval.str_ptr = new string(yytext); return LIT_BOOL; }
global     { return GLOBAL_DECL; }
machine    { return MACHINE_DECL; }
in_port    { return IN_PORT_DECL; }
out_port   { return OUT_PORT_DECL; }
action     { return ACTION_DECL; }
transition { return TRANSITION_DECL; }
structure  { return STRUCT_DECL; }
external_type { return EXTERN_TYPE_DECL; }
enumeration { return ENUM_DECL; }
peek       { return PEEK; }
enqueue    { return ENQUEUE; }
copy_head  { return COPY_HEAD; }
check_allocate  { return CHECK_ALLOCATE; }
check_stop_slots  { return CHECK_STOP_SLOTS; }
if         { return IF; }
else       { return ELSE; }
return     { return RETURN; }
THIS       { return THIS; }
CHIP       { return CHIP; }
void       { yylval.str_ptr = new string(yytext); return VOID; }
new        { return NEW; }

==        { yylval.str_ptr = new string(yytext); return EQ; }
!=        { yylval.str_ptr = new string(yytext); return NE; }
[<]       { yylval.str_ptr = new string(yytext); return '<'; }
[>]       { yylval.str_ptr = new string(yytext); return '>'; }
[<][<]    { yylval.str_ptr = new string(yytext); return LEFTSHIFT; }
[>][>]    { yylval.str_ptr = new string(yytext); return RIGHTSHIFT; }
[<][=]    { yylval.str_ptr = new string(yytext); return LE; }
[>][=]    { yylval.str_ptr = new string(yytext); return GE; }
[!]       { yylval.str_ptr = new string(yytext); return NOT; }
[&][&]    { yylval.str_ptr = new string(yytext); return AND; }
[|][|]    { yylval.str_ptr = new string(yytext); return OR; }
[+]       { yylval.str_ptr = new string(yytext); return PLUS; }
[-]       { yylval.str_ptr = new string(yytext); return DASH; }
[*]       { yylval.str_ptr = new string(yytext); return STAR; }
[/]       { yylval.str_ptr = new string(yytext); return SLASH; }
::        { return DOUBLE_COLON; }
[:]       { return ':'; }
[;]       { return SEMICOLON; }
[[]       { return '['; }
[]]       { return ']'; }
[{]       { return '{'; }
[}]       { return '}'; }
[(]       { return '('; }
[)]       { return ')'; }
[,]       { return ','; }
[=]       { return '='; }
:=        { return ASSIGN; }
[.]       { return DOT; }

[0-9]*[.][0-9]*     { yylval.str_ptr = new string(yytext); return FLOATNUMBER; }
[0-9]*     { yylval.str_ptr = new string(yytext); return NUMBER; }

[a-zA-Z_][a-zA-Z_0-9]{0,50}    { yylval.str_ptr = new string(yytext); return IDENT; }
\"[^"\n]*\"  { yytext[strlen(yytext)-1] = '\0';  yylval.str_ptr = new string(yytext+1); return STRING; }
\'[^'\n]*\'  { yytext[strlen(yytext)-1] = '\0';  yylval.str_ptr = new string(yytext+1); return STRING; }

.         { return OTHER; }  /* Need so that we handle all characters */

%%

