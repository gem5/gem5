# lex_token.py
#
# Tests for absence of tokens variable

import lex

t_PLUS = r'\+'
t_MINUS = r'-'
t_NUMBER = r'\d+'

def t_error(t):
    pass

import sys
sys.tracebacklimit = 0

lex.lex()


