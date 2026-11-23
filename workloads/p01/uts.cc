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
 * uts: busca em árvore desbalanceada (inspirada no Unbalanced Tree Search).
 *
 * A árvore é gerada de forma implícita: o número de filhos de um nó depende só
 * do identificador dele (um hash), então a árvore é sempre a mesma para uma
 * dada semente, mas a forma de cada subárvore é imprevisível. Cada nó "fértil"
 * tem FILHOS filhos; os demais são folhas. Com FILHOS * PROB = 4 * 0,2475 =
 * 0,99 (quase crítico), uma subárvore tem em média ~100 nós, porém com enorme
 * variação (poucas gigantes e muitas minúsculas): exatamente o cenário em que
 * a divisão estática sofre.
 *
 * Uma tarefa = visitar um nó. Cada visita conta o nó e cria (gerar) uma tarefa
 * para cada filho. A divisão estática só distribui entre os trabalhadores os
 * filhos da RAIZ; o resto fica no trabalhador que gerou.
 *
 * Resultado = número total de nós da árvore (inclui a raiz). O programa
 * "sequencial" calcula esse valor sem a biblioteca (referência).
 */

namespace {

constexpr unsigned FILHOS = 4;              // filhos de um nó fértil
constexpr unsigned PROB_10K = 2475;         // chance de ser fértil, em 1/10000
constexpr unsigned PROFUNDIDADE_MAX = 4000; // trava de segurança

struct No {
    uint64_t id;
    unsigned profundidade;
};

/* Um contador por trabalhador, alinhado a uma linha de cache (evita falso
   compartilhamento): cada trabalhador só escreve no seu. */
struct alignas(64) Contador {
    uint64_t valor = 0;
};
std::vector<Contador> contadores;

/* Função de espalhamento determinística (finalizador do SplitMix64). */
uint64_t mistura(uint64_t x)
{
    x += 0x9E3779B97F4A7C15ull;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
    return x ^ (x >> 31);
}

unsigned num_filhos(uint64_t id, unsigned profundidade)
{
    if (profundidade >= PROFUNDIDADE_MAX) return 0;
    return (mistura(id) % 10000 < PROB_10K) ? FILHOS : 0;
}

uint64_t id_filho(uint64_t id, unsigned j)
{
    return mistura(id + 0x9E3779B97F4A7C15ull * static_cast<uint64_t>(j + 1));
}

/* Tarefa: visita um nó e gera uma tarefa por filho. */
void visita(void *dados, Trabalhador &t)
{
    No *no = static_cast<No *>(dados);
    contadores[t.indice()].valor++;                  // conta este nó

    const unsigned k = num_filhos(no->id, no->profundidade);
    for (unsigned j = 0; j < k; ++j)
        t.gerar(visita, new No{id_filho(no->id, j), no->profundidade + 1});

    delete no;                                       // o nó já foi processado
}

/* Referência sequencial: percurso com pilha explícita, sem a biblioteca. */
uint64_t conta_sequencial(uint64_t semente, uint64_t filhos_da_raiz)
{
    const uint64_t raiz = mistura(semente);
    std::vector<No> pilha;
    for (uint64_t j = 0; j < filhos_da_raiz; ++j)
        pilha.push_back(No{id_filho(raiz, static_cast<unsigned>(j)), 1});

    uint64_t nos = 1;                                // a raiz
    while (!pilha.empty()) {
        const No no = pilha.back();
        pilha.pop_back();
        ++nos;
        const unsigned k = num_filhos(no.id, no.profundidade);
        for (unsigned j = 0; j < k; ++j)
            pilha.push_back(No{id_filho(no.id, j), no.profundidade + 1});
    }
    return nos;
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
    if (argc != 5) {
        std::fprintf(stderr,
                     "Uso: %s <num_trabalhadores> <estatica|roubo|sequencial> "
                     "<semente> <filhos_da_raiz>\n",
                     argv[0]);
        return 1;
    }

    long num_trabalhadores;
    long semente;
    long filhos_da_raiz;
    if (!le_inteiro(argv[1], 1, UINT_MAX, num_trabalhadores) ||
        !le_inteiro(argv[3], 0, LONG_MAX, semente) ||
        !le_inteiro(argv[4], 1, LONG_MAX, filhos_da_raiz)) {
        std::fprintf(stderr,
                     "Trabalhadores e filhos da raiz devem ser positivos; "
                     "semente deve ser um inteiro não negativo.\n");
        return 1;
    }

    /* "sequencial" roda sem a biblioteca: é o valor de referência que todas as
       execuções devem reproduzir. */
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
        uint64_t nos = 0;
        uint64_t tarefas_executadas = 0;

        if (sequencial) {
            nos = conta_sequencial(static_cast<uint64_t>(semente),
                                   static_cast<uint64_t>(filhos_da_raiz));
            tarefas_executadas = nos - 1;            // todos os nós, menos a raiz
        } else {
            contadores.assign(static_cast<std::size_t>(num_trabalhadores),
                              Contador{});
            Escalonador escalonador(static_cast<unsigned>(num_trabalhadores),
                                    politica,
                                    static_cast<uint64_t>(semente));

            /* Tarefas iniciais: os filhos da raiz. */
            const uint64_t raiz = mistura(static_cast<uint64_t>(semente));
            for (long j = 0; j < filhos_da_raiz; ++j)
                escalonador.submeter(
                    visita, new No{id_filho(raiz, static_cast<unsigned>(j)), 1});

            tarefas_executadas = escalonador.executar();
            nos = 1;                                 // a raiz
            for (const Contador &c : contadores) nos += c.valor;
        }

        std::printf("Filhos da raiz: %ld\n", filhos_da_raiz);
        std::printf("Tarefas executadas: %llu\n",
                    static_cast<unsigned long long>(tarefas_executadas));
        std::printf("Nós da árvore: %llu\n",
                    static_cast<unsigned long long>(nos));
    } catch (const std::exception &erro) {
        std::fprintf(stderr, "Erro ao executar o workload: %s\n", erro.what());
        return 1;
    }

    return 0;
}