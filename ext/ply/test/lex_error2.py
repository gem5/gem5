# lex_token.py
#
# t_error defined, but not function

import lex

tokens = [
    "PLUS",
    "MINUS",
    "NUMBER",
    ]

t_PLUS = r'\+'
t_MINUS = r'-'
t_NUMBER = r'\d+'

t_error = "foo"

import sys
sys.tracebacklimit = 0

lex.lex()


