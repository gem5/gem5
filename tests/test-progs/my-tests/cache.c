#include <stdint.h>

#define SIZE 8*1024

int main() {
    uint32_t result = 0;
    uint32_t arr[SIZE];
    for(int i = 0; i < SIZE; i+=16) {
        asm volatile (
        "lax %0, 0(%1)"
        : "=r" (result)
        : "r" (arr+i)
        : "memory"
    );
    }

    for(int i = 0; i < SIZE; i+=16) {
        asm volatile (
        "lax %0, 0(%1)"
        : "=r" (result)
        : "r" (arr+i)
        : "memory"
    );
    }
}