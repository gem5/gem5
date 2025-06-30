#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>

int main() {
    uint64_t input = 0x1234567890abcdef;
    uint64_t output;

    // Store input into VSR (vector-scalar register) using mtvsrd
    __asm__ volatile (
        "mtvsrd 0, %0\n\t" // VSR0 <- input
        :
        : "r" (input)
        : "v0" // clobber VSR0
    );

    // Load from VSR back into output using mfvsrd
    __asm__ volatile (
        "mfvsrd %0, 0\n\t" // output <- VSR0
        : "=r" (output)
        :
        : "v0" // clobber VSR0
    );

    printf("Input:  0x%016" PRIx64 "\n", input);
    printf("Output: 0x%016" PRIx64 "\n", output);

    return 0;
}
