#include "tarefas.hh"

#include <cerrno>
#include <climits>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>
#include <vector>

/*
 * fila: fila de tarefas independentes de custo heterogêneo (calibração).
 *
 * Distribuição bimodal controlada: uma fração f das tarefas custa C unidades de
 * trabalho; as demais custam 1. Quais tarefas são pesadas é sorteado de forma
 * reprodutível a partir da semente. Todas as tarefas são submetidas no início.
 *
 * Previsão para a divisão estática circular com P trabalhadores (para o
 * relatório): custo médio por tarefa mu = 1 + f(C-1) e variância
 * s2 = f(1-f)(C-1)^2. Cada trabalhador recebe T/P tarefas, então sua carga tem
 * média mu*T/P e desvio s*sqrt(T/P); o desequilíbrio esperado (maior carga /
 * carga média) fica perto de 1 + z*s/(mu*sqrt(T/P)), com z ~ 1 (P=2) a ~ 1,4
 * (P=8) para o máximo de P cargas aproximadamente normais. Quanto maior C e
 * menor T, maior o desequilíbrio.
 *
 * Resultado = soma (módulo 2^64) das saídas das tarefas. O programa
 * "sequencial" calcula esse valor sem a biblioteca (referência).
 */

namespace {

constexpr uint64_t ITER_UNIDADE = 400;   // iterações do laço em uma "unidade"

struct Item {
    uint64_t indice;
    uint64_t unidades;                   // custo: 1 ou C
};

/* Um contador por trabalhador, alinhado a uma linha de cache. */
struct alignas(64) Contador {
    uint64_t valor = 0;
};
std::vector<Contador> contadores;

uint64_t mistura(uint64_t x)
{
    x += 0x9E3779B97F4A7C15ull;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
    return x ^ (x >> 31);
}

/* O trabalho de uma tarefa: um laço serial de multiplicações (o compilador não
   consegue eliminá-lo). Custa `unidades * ITER_UNIDADE` iterações. */
uint64_t trabalho(uint64_t indice, uint64_t unidades)
{
    uint64_t x = mistura(indice) | 1;
    for (uint64_t i = 0; i < unidades * ITER_UNIDADE; ++i)
        x = x * 6364136223846793005ull + 1442695040888963407ull;
    return x;
}

/* Função com a assinatura exigida pela biblioteca (FuncTarefa). */
void tarefa_item(void *dados, Trabalhador &t)
{
    const Item *item = static_cast<Item *>(dados);
    contadores[t.indice()].valor += trabalho(item->indice, item->unidades);
}

/* Sorteia, de forma reprodutível, quais tarefas são pesadas. */
std::vector<Item> fazer_itens(uint64_t semente, uint64_t total,
                              uint64_t f_mil, uint64_t custo)
{
    std::vector<Item> itens;
    itens.reserve(total);
    for (uint64_t i = 0; i < total; ++i) {
        const bool pesada =
            mistura(semente * 0x2545F4914F6CDD1Dull + i) % 1000 < f_mil;
        itens.push_back(Item{i, pesada ? custo : 1});
    }
    return itens;
}

bool le_inteiro(const char *texto, long minimo, long maximo, long &valor)
{
    errno = 0;
    char *fim = nullptr;
    const long convertido = std::strtol(texto, &fim, 10);
    if (errno == ERANGE || fim == texto || *fim != '\0' ||
        convertido < minimo || convertido > maximo) {
        return false;
    }
    valor = convertido;
    return true;
}

} // namespace

int main(int argc, char *argv[])
{
    if (argc < 5 || argc > 7) {
        std::fprintf(stderr,
                     "Uso: %s <num_trabalhadores> <estatica|roubo|sequencial> "
                     "<semente> <num_tarefas> [fracao_pesada_em_milesimos "
                     "[custo_pesada]]\n"
                     "  padrões: fração = 100 (10%%), custo = 100\n",
                     argv[0]);
        return 1;
    }

    long num_trabalhadores;
    long semente;
    long num_tarefas;
    long f_mil = 100;       // fração de tarefas pesadas, em milésimos
    long custo = 100;       // custo de uma tarefa pesada, em unidades
    if (!le_inteiro(argv[1], 1, UINT_MAX, num_trabalhadores) ||
        !le_inteiro(argv[3], 0, LONG_MAX, semente) ||
        !le_inteiro(argv[4], 1, LONG_MAX, num_tarefas) ||
        (argc > 5 && !le_inteiro(argv[5], 0, 1000, f_mil)) ||
        (argc > 6 && !le_inteiro(argv[6], 1, LONG_MAX, custo))) {
        std::fprintf(stderr,
                     "Trabalhadores, tarefas e custo devem ser positivos; "
                     "semente não negativa; fração entre 0 e 1000.\n");
        return 1;
    }

    const std::string nome_politica = argv[2];
    const bool sequencial = (nome_politica == "sequencial");
    Politica politica = Politica::Estatica;
    if (nome_politica == "roubo") {
        politica = Politica::Roubo;
    } else if (nome_politica != "estatica" && !sequencial) {
        std::fprintf(stderr, "Política inválida: use 'estatica', 'roubo' ou "
                             "'sequencial'.\n");
        return 1;
    }

    try {
        /* O vetor é completado antes da submissão para manter estáveis os
           endereços guardados pelo escalonador em cada tarefa. */
        std::vector<Item> itens =
            fazer_itens(static_cast<uint64_t>(semente),
                        static_cast<uint64_t>(num_tarefas),
                        static_cast<uint64_t>(f_mil),
                        static_cast<uint64_t>(custo));

        uint64_t soma = 0;
        uint64_t tarefas_executadas = 0;

        if (sequencial) {
            for (const Item &item : itens)
                soma += trabalho(item.indice, item.unidades);
            tarefas_executadas = itens.size();
        } else {
            contadores.assign(static_cast<std::size_t>(num_trabalhadores),
                              Contador{});
            Escalonador escalonador(static_cast<unsigned>(num_trabalhadores),
                                    politica,
                                    static_cast<uint64_t>(semente));
            for (Item &item : itens)
                escalonador.submeter(tarefa_item, &item);

            tarefas_executadas = escalonador.executar();
            for (const Contador &c : contadores) soma += c.valor;
        }

        std::printf("Tarefas submetidas: %zu\n", itens.size());
        std::printf("Tarefas executadas: %llu\n",
                    static_cast<unsigned long long>(tarefas_executadas));
        std::printf("Soma dos resultados: %llu\n",
                    static_cast<unsigned long long>(soma));
    } catch (const std::exception &erro) {
        std::fprintf(stderr, "Erro ao executar o workload: %s\n", erro.what());
        return 1;
    }

    return 0;
}