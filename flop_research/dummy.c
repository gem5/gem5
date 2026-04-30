// dummy.c
int main() {
    volatile int a = 0;
    for(int i = 0; i < 20; i++) {
        a += i;
    }
    return 0;
}
