#include <stdio.h>

int main(){
    int a,b;
    a = 0;
    b = 33;

    unsigned int result;
    long addr = (long)&b;

    asm volatile (
        "lax %0, 0(%1)"
        : "=r" (result)
        : "r" (addr)
        : "memory"
    );

    if ( result != 33 ){
            printf("\n[[FAILED]]\n");
            return -1;
    }
    printf("\n[[PASSED]]\n");
    return 0;
}
