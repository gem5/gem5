
#
# `Normal' configuration.
#
CC	      = gcc -ansi -Wall -pedantic

.o.s:
	as -o $@ $< 
