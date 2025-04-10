#include <stdio.h>

int main() {
    int arr[5];

    for(int i = 0; i < 5; i++) {
        arr[i] = i*i;
    }

    int ans = 0;
    for(int i = 0; i < 5; i++) {
        ans += arr[i];
    }

    printf("Answer = %d\n", ans);
    return 0;
}
