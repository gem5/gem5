#include <stdio.h>
int main(){
    int a,b,c;
    a = 98;
    b = 56;
    asm volatile
    (
    "gcd   %[z], %[x], %[y]\n\t"
    : [z] "=r" (c)
    : [x] "r" (a), [y] "r" (b)
    );
    if ( c != 14 ){
            printf("\n[[FAILED]]\n");
            return -1;
    }
    printf("\n[[PASSED]]\n");
    return 0;
}