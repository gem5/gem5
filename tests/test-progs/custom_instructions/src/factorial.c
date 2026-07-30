#include <stdio.h>
int main(){
    int a,b,c;
    a = 5;
    asm volatile
    (
    "fact   %[z], %[x]\n\t"
    : [z] "=r" (c)
    : [x] "i" (5)
    );
    if ( c != 120 ){
            printf("\n[[FAILED]]\n");
            return -1;
    }
    printf("\n[[PASSED]]\n");
    return 0;
}