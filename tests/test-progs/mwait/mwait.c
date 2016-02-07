// author: Marc Orr

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

#define NUM_TRIES   1000

// Make sure that flags and wait sit in different cache lines
volatile int flags[10];
volatile int wait[10];

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

void *DoWork1(void *threadid)
{
    flags[0] = flags[0] + 1;
    wait[0] = 0;
    pthread_exit(0);
}

void *DoWork2(void *threadid)
{
    pthread_mutex_lock (&mutex);
    flags[0] = flags[0] + 1;
    pthread_mutex_unlock (&mutex);
    pthread_exit(0);
}

////////////////////////////////////////////////////////////////////////////////
// Program main
////////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv)
{
    // stuff for thread
    pthread_t threads[1];

    // initialize global variables
    flags[0] = 0;
    wait[0] = 1;

    // monitor (via gcc intrinsic)
    __builtin_ia32_monitor ((void *)&flags, 0, 0);

    // invalidate flags in this cpu's cache
    pthread_create(&threads[0], NULL, DoWork1, NULL);
    while (wait[0]);

    // launch thread to invalidate address being monitored
    pthread_create(&threads[0], NULL, DoWork2, NULL);

    // wait for other thread to modify flags
    int mwait_cnt = 0;
    do {
        pthread_mutex_lock (&mutex);
        if (flags[0] != 2) {
            pthread_mutex_unlock (&mutex);
            __builtin_ia32_mwait(0, 0);
        } else {
            pthread_mutex_unlock (&mutex);
        }
        mwait_cnt++;
    } while (flags[0] != 2 && mwait_cnt < NUM_TRIES);

    // test may hang if mwait is not working
    if (flags[0]==2) {
        printf("mwait regression PASSED, flags[0] = %d\n", flags[0]);
    } else {
        printf("mwait regression FAILED, flags[0] = %d\n", flags[0]);
    }

    return 0;
}
