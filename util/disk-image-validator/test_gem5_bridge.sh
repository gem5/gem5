#!/bin/bash

# Create the test.c file that
# maps m5 mem and calls m5 exit
# if the test passes that means that the gem5 bridge driver is working
cat << EOF > test.c
# include <stdio.h>
#include <gem5/m5ops.h>
void map_m5_mem();
int main() {
    map_m5_mem();
    m5_hypercall_addr(8);
    return 0;
}
EOF

# Compile the C file
gcc test.c -L/usr/local/lib -lm5 -o test_c.exe -no-pie

# Run the compiled executable
./test_c.exe

sleep 5
