/*
Trivial blocked matrix multiply.

Written by Charles Reiss.

To the extent possible under law, I waive all copyright and related or neighboring rights 
to this file.
 */

#define _XOPEN_SOURCE 700
#include <assert.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

// Allow easy switching between float and double
typedef float myfloat;

void random_matrix(myfloat *A, int I, int J) {
    for (int i = 0; i < I; ++i) {
        for (int j = 0; j < J; ++j) {
            A[i * J + j] = drand48();
        }
    }
}

/* computes matrix multiply C += A * B
 * A is I by K, B is K by J, C is I by J 
 */
void blocked_matmul(myfloat * restrict A, myfloat * restrict B, myfloat * restrict C, int I, int K, int J) {
    assert(I % 2 == 0);
    assert(J % 2 == 0);
    assert(K % 2 == 0);
    for (int i = 0; i < I; i += 2) {
        for (int j = 0; j < J; j += 2) {
            myfloat Ci0j0 = C[(i + 0) * J + (j + 0)];
            myfloat Ci0j1 = C[(i + 0) * J + (j + 1)];
            myfloat Ci1j0 = C[(i + 1) * J + (j + 0)];
            myfloat Ci1j1 = C[(i + 1) * J + (j + 1)];
            for (int k = 0; k < K; k += 2) {
                myfloat Ai0k0 = A[(i + 0) * K + (k + 0)];
                myfloat Ai0k1 = A[(i + 0) * K + (k + 1)];
                myfloat Ai1k0 = A[(i + 1) * K + (k + 0)];
                myfloat Ai1k1 = A[(i + 1) * K + (k + 1)];
                myfloat Bk0j0 = B[(k + 0) * J + (j + 0)];
                myfloat Bk0j1 = B[(k + 0) * J + (j + 1)];
                myfloat Bk1j0 = B[(k + 1) * J + (j + 0)];
                myfloat Bk1j1 = B[(k + 1) * J + (j + 1)];

                Ci0j0 += Ai0k0 * Bk0j0 + Ai0k1 * Bk1j0;
                Ci1j0 += Ai1k0 * Bk0j0 + Ai1k1 * Bk1j0;
                Ci0j1 += Ai0k0 * Bk0j1 + Ai0k1 * Bk1j1;
                Ci1j1 += Ai1k0 * Bk0j1 + Ai1k1 * Bk1j1;
            }
            C[(i + 0) * J + (j + 0)] = Ci0j0;
            C[(i + 0) * J + (j + 1)] = Ci0j1;
            C[(i + 1) * J + (j + 0)] = Ci1j0;
            C[(i + 1) * J + (j + 1)] = Ci1j1;
        }
    }
}

uint64_t read_tsc(void) {
    uint64_t hi, lo;
    return lo | (hi << 32);
}

#define SIZE 84

static myfloat A[SIZE * SIZE];
static myfloat B[SIZE * SIZE];
static myfloat C[SIZE * SIZE];

int main(void) {
    srand48(1); // fixed seed for reproducible results
    random_matrix(A, SIZE, SIZE);
    random_matrix(B, SIZE, SIZE);
    random_matrix(C, SIZE, SIZE);
    for (int i = 0; i < 4; ++i) {
        long start = read_tsc();
        blocked_matmul(A, B, C, SIZE, SIZE, SIZE);
        long end = read_tsc();
        printf("Iteration %d: %ld cycles\n", i, end - start);
    }
}
