#include <stdio.h>
int main(){
    int a,b,c;
    a = 5;
    b = 2;
    asm volatile
    (
    "mod   %[z], %[x], %[y]\n\t"
    : [z] "=r" (c)
    : [x] "r" (a), [y] "r" (b)
    );
    if ( c != 1 ){
            printf("\n[[FAILED]]\n");
            return -1;
    }
    printf("\n[[PASSED]]\n");
    return 0;
}