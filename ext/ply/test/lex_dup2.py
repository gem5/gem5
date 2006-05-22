# lex_token.py
#
# Duplicated rule specifiers

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
    pass

def t_NUMBER(t):
    r'\d+'
    pass

def t_error(t):
    pass

import sys
sys.tracebacklimit = 0

lex.lex()


