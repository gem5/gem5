# lex_token.py
#
# Missing t_error() rule

import lex

tokens = [
    "PLUS",
    "MINUS",
    "NUMBER",
    ]

t_PLUS = r'\+'
t_MINUS = r'-'
t_NUMBER = r'\d+'

import sys
sys.tracebacklimit = 0

lex.lex()


