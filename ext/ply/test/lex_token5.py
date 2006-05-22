# lex_token.py
#
# Return a bad token name

import lex

tokens = [
    "PLUS",
    "MINUS",
    "NUMBER",
    ]

t_PLUS = r'\+'
t_MINUS = r'-'

def t_NUMBER(t):
    r'\d+'
    t.type = "NUM"
    return t

def t_error(t):
    pass

import sys
sys.tracebacklimit = 0

lex.lex()
lex.input("1234")
t = lex.token()


