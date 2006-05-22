# lex_token.py
#
# t_error defined as function, but too many args

import lex

tokens = [
    "PLUS",
    "MINUS",
    "NUMBER",
    ]

t_PLUS = r'\+'
t_MINUS = r'-'
t_NUMBER = r'\d+'

def t_error(t,s):
    pass

import sys
sys.tracebacklimit = 0

lex.lex()


