#include <stdio.h>
#include <stdlib.h>

int main(){
    int* addr = malloc(10000);
    int result;

    // printf("VA of b : %u(0x%x)\n", addr, addr);

    asm volatile (
        "lw %0, 0(%1)"
        : "=r" (result)
        : "r" (addr+9900)
        : "memory"
    );

    asm volatile (
        "lax %0, 0(%1)"
        : "=r" (result)
        : "r" (addr+9900)
        : "memory"
    );

    asm volatile (
        "lax %0, 0(%1)"
        : "=r" (result)
        : "r" (addr+9900)
        : "memory"
    );

    // asm volatile (
    //     "lw %0, 0(%1)"
    //     : "=r" (result)
    //     : "r" (addr+9900)
    //     : "memory"
    // );

    // asm volatile (
    //     "lw %0, 0(%1)"
    //     : "=r" (result)
    //     : "r" (addr+9900)
    //     : "memory"
    // );

    return 0;
}
