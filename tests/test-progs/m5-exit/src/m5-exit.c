#include <stdio.h>

#ifdef M5
#include <gem5/m5ops.h>

// If you need to define this, you should have removed the -DM5OP_ADDR
// when compiling the m5op_x86.o
//void *m5_mem = (void*)0xCAFEBABE;
#endif

int main() {
    #ifdef M5
    m5_exit(0);
    #endif
    printf("FAIL!\n");
    printf("Program should have exited due to the magic m5_exit"
           " instruction!\n");
    return -1;
}
