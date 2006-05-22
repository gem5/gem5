# lex_token.py
#
# Bad regular expression in a string

import lex

tokens = [
    "PLUS",
    "MINUS",
    "NUMBER",
    ]

t_PLUS = r'\+'
t_MINUS = r'-'
t_NUMBER = r'(\d+'

def t_error(t):
    pass

import sys
sys.tracebacklimit = 0

lex.lex()


