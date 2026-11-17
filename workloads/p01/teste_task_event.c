#ifdef USE_M5OPS
#include <gem5/m5ops.h>
#endif
#include <errno.h>
#include <limits.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "tarefas.hh"

enum {
    TAM = 20
};

static int matriz_a[TAM][TAM];
static int matriz_b[TAM][TAM];
static int resultado[TAM][TAM];

typedef struct {
    int id;
    int primeira_linha;
    int ultima_linha;
} tarefa_t;

static void *multiplica_linhas(void *argumento)
{
    tarefa_t *tarefa = argumento;
    #ifdef USE_M5OPS
    m5_task_event(0, tarefa->id);
    #endif
    for (int linha = tarefa->primeira_linha;
         linha < tarefa->ultima_linha;
         linha++) {
        for (int coluna = 0; coluna < TAM; coluna++) {
            resultado[linha][coluna] = 0;
            for (int k = 0; k < TAM; k++) {
                resultado[linha][coluna] +=
                    matriz_a[linha][k] * matriz_b[k][coluna];
            }
        }
    }
    #ifdef USE_M5OPS
    m5_task_event(1, tarefa->id);
    #endif
    return NULL;
}

static void imprime_matriz(const char *nome, const int matriz[TAM][TAM])
{
    printf("%s:\n", nome);
    for (int linha = 0; linha < TAM; linha++) {
        for (int coluna = 0; coluna < TAM; coluna++) {
            printf("%4d ", matriz[linha][coluna]);
        }
        printf("\n");
    }
}

int main(int argc, char *argv[])
{
    if (argc != 2) {
        fprintf(stderr, "Uso: %s <num_threads>\n", argv[0]);
        return 1;
    }

    errno = 0;
    char *fim;
    long num_threads = strtol(argv[1], &fim, 10);
    if (errno == ERANGE || fim == argv[1] || *fim != '\0' ||
        num_threads < 1 || num_threads > INT_MAX) {
        fprintf(stderr, "num_threads deve ser um inteiro positivo.\n");
        return 1;
    }

    pthread_t *threads = calloc((size_t)num_threads, sizeof(*threads));
    tarefa_t *tarefas = calloc((size_t)num_threads, sizeof(*tarefas));
    if (threads == NULL || tarefas == NULL) {
        fprintf(stderr, "Erro ao alocar memoria para as threads.\n");
        free(threads);
        free(tarefas);
        return 1;
    }

    for (int i = 0; i < num_threads; i++) {
        tarefas[i].id = i;
        tarefas[i].primeira_linha = (int)((long long)i * TAM / num_threads);
        tarefas[i].ultima_linha =
            (int)((long long)(i + 1) * TAM / num_threads);
    }

    srand((unsigned int)time(NULL));
    for (int linha = 0; linha < TAM; linha++) {
        for (int coluna = 0; coluna < TAM; coluna++) {
            matriz_a[linha][coluna] = rand() % 10;
            matriz_b[linha][coluna] = rand() % 10;
        }
    }
    for (int i = 0; i < num_threads; i++) {
        int erro = pthread_create(&threads[i], NULL, multiplica_linhas,
                                  &tarefas[i]);
        if (erro != 0) {
            fprintf(stderr, "Erro ao criar thread %d: %s\n", i,
                    strerror(erro));
            for (int j = 0; j < i; j++) {
                pthread_join(threads[j], NULL);
            }
            free(threads);
            free(tarefas);
            return 1;
        }
    }

    int erro_ao_aguardar = 0;
    for (int i = 0; i < num_threads; i++) {
        int erro = pthread_join(threads[i], NULL);
        if (erro != 0) {
            fprintf(stderr, "Erro ao aguardar thread %d: %s\n", i,
                    strerror(erro));
            erro_ao_aguardar = 1;
        }
    }
    free(threads);
    free(tarefas);
    if (erro_ao_aguardar) {
        return 1;
    }

    imprime_matriz("Matriz A", matriz_a);
    imprime_matriz("Matriz B", matriz_b);
    imprime_matriz("Resultado da multiplicacao", resultado);

    return 0;
}