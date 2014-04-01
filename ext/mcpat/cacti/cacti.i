%module cacti
%{
/* Includes the header in the wrapper code */
#include "cacti_interface.h"
%}

/* Parse the header file to generate wrappers */
%include "cacti_interface.h"