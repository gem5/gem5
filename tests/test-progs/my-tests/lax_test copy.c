#include <stdio.h>

int main(){
    int a,b;
    a = 0;
    b = 33;

    char buffer[1000];

    buffer[0] = 'c';

    unsigned int result;
    long addr = (long)&b;

    printf("VA of b : %u(0x%x)\n", addr, addr);

    asm volatile (
        "lax %0, 0(%1)"
        : "=r" (result)
        : "r" (addr)
        : "memory"
    );

    if (result != 33){
            printf("\n[[FAILED]]\n");
            return -1;
    }
    printf("\n[[PASSED]]\n");
    return 0;
}
