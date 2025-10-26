#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>
#include <pthread.h>
#include <unistd.h>
#include <stdint.h>

// 定义内存块大小
#define MEMORY_SIZE 10  // 1KB内存块
#define NUM_THREADS 8
#define SYNC_SIZE 100

// 全局数据区域
char* shared_buffers[NUM_THREADS];
atomic_int data_ready[NUM_THREADS][NUM_THREADS];
unsigned char data_region[NUM_THREADS][SYNC_SIZE];

void* complete_thread_function(void* arg) {
    int thread_id = *((int*)arg);
    unsigned char thread_pattern = 0xF0 | (thread_id);  // 0xF1, 0xF2, 0xF3, 0xF4
        
    printf("Thread %d: Starting complete all-to-all communication\n", thread_id);

    int value_to_store = 0xFFFFFF00 | thread_pattern;


    // 1. 发送数据到其他所有线程
    for (int target = 0; target < NUM_THREADS; target++) {
        if (target != thread_id) {
            // 将自己的数据写入目标线程的数据区域
            for (int i = 0; i < SYNC_SIZE; i++) {
                data_region[target][i] = data_region[thread_id][i];
            }
            

            printf("Thread %d: Sending data to thread %d and setting flag to 0x%x\n", 
                   thread_id, target, value_to_store);
            
            __asm__ volatile (
                "stlr %w[value], [%[addr]]"
                : // no output operands
                : [addr] "r" (&data_ready[thread_id][target]), [value] "r" (value_to_store)
                : "memory"
            );
        }
    }
        
    // 2. 接收来自其他所有线程的数据
    // /*
    for (int source = 0; source < NUM_THREADS; source++) {
        if (source != thread_id) {
            int loaded_value;
            unsigned char expected_pattern = 0xF0 | (source + 1);  // 期望的模式
            printf("Thread %d: Waiting for data from thread %d (expecting pattern 0x%x)\n", 
                   thread_id, source, expected_pattern);
            
            // 使用acquire语义等待数据准备就绪
            __asm__ volatile (
                "ldar %w[result], [%[addr]]"
                : [result] "=r" (loaded_value)
                : [addr] "r" (&data_ready[source][thread_id])
                : "memory"
            );
            
            // 验证接收到的flag值是否正确
            int expected_flag = 0xFFFFFF00 | expected_pattern;
            if (loaded_value == expected_flag) {
                printf("Thread %d: Received correct flag 0x%x from thread %d\n", 
                       thread_id, loaded_value, source);
            } else {
                printf("Thread %d: ERROR! Expected flag 0x%x but got 0x%x from thread %d\n", 
                       thread_id, expected_flag, loaded_value, source);
            }
        }
    }
    // */
        
    // 3. 验证接收到的数据
    int errors = 0;
    for (int source = 0; source < NUM_THREADS; source++) {
        if (source != thread_id) {
            unsigned char expected_pattern = 0xF0 | (source + 1);
            for (int i = 0; i < SYNC_SIZE; i++) {
                if (data_region[thread_id][i] != expected_pattern) {
                    errors++;
                }
            }
        }
    }
        
    if (errors == 0) {
        printf("Thread %d: All data received correctly!\n", thread_id);
    } else {
        printf("Thread %d: Found %d errors in received data\n", thread_id, errors);
    }
        
    printf("Thread %d: Complete all-to-all communication finished\n", thread_id);
    return NULL;
}


int main(int argc, char *argv[]) {
    int thread_ids[NUM_THREADS];
    int base_id = (int)strtoul(argv[1], NULL, 0);
    printf("==========111=======================base_id:%d\n", base_id);
    printf("\n=== Complete All-to-All Memory Communication Demo ===\n");

    
    
    // 初始化数据区域 - 使用线程ID初始化内存
    for (int i = 0; i < NUM_THREADS; i++) {
        int thread_pattern = 0xF0 | i;  // 0xF1, 0xF2, 0xF3, 0xF4
        for (int j = 0; j < SYNC_SIZE; j++) {
            data_region[i][j] = thread_pattern;
        }
    }

    pthread_t p1, p2, p3, p4, p5, p6, p7, p8;
    pthread_t* threads = (pthread_t *)malloc(NUM_THREADS * sizeof(pthread_t));
    
    
    // pthread_t *threads = (pthread_t *)malloc(NUM_THREADS * sizeof(pthread_t));
    for (int i = 0; i < NUM_THREADS; i++) {
        thread_ids[i] = i + base_id;
        printf("Thread %d\n", thread_ids[i]);
        if (pthread_create(&threads[i], NULL, complete_thread_function, (void *)&thread_ids[i]) != 0) {
            perror("pthread_create failed");
            exit(EXIT_FAILURE);
        }
    }
    for (int i = 0; i < NUM_THREADS; i++) {
        pthread_join(threads[i], NULL);
    }

    printf("Complete all-to-all communication completed\n");
        
    free(threads);

    return 0;
}