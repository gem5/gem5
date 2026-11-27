#include "tarefas.hh"

#include <algorithm>
#include <cerrno>
#include <climits>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>
#include <vector>

namespace {

// Parâmetros da imagem. Os valores abaixo são os padrões; podem ser trocados
// pela linha de comando (largura, altura e máximo de iterações), o que permite
// calibrar o custo da execução sem recompilar.
int LARGURA = 600;
int ALTURA = 450;
int MAX_ITERACOES = 1000;

/* Bloco QUADRADO (o enunciado pede blocos quadrados; antes eram faixas de
   linhas com a largura inteira). Os blocos da borda direita/inferior podem ser
   menores quando a imagem não é múltipla do tamanho do bloco. */
struct Bloco {
    int x0, x1;                 // colunas [x0, x1)
    int y0, y1;                 // linhas  [y0, y1)
    uint64_t soma_iteracoes = 0;
};

/* Cálculo puro do bloco, sem depender da biblioteca: serve tanto para a tarefa
   quanto para a referência sequencial. */
void calcula(Bloco &bloco)
{
    uint64_t soma = 0;

    for (int y = bloco.y0; y < bloco.y1; ++y) {
        const double ci = (static_cast<double>(y) - ALTURA / 2.0) *
                          4.0 / ALTURA;
        for (int x = bloco.x0; x < bloco.x1; ++x) {
            const double cr = (static_cast<double>(x) - LARGURA / 2.0) *
                              4.0 / LARGURA;
            double zr = 0.0;
            double zi = 0.0;
            int iteracoes = 0;

            while (zr * zr + zi * zi <= 4.0 &&
                   iteracoes < MAX_ITERACOES) {
                const double novo_zr = zr * zr - zi * zi + cr;
                zi = 2.0 * zr * zi + ci;
                zr = novo_zr;
                ++iteracoes;
            }
            soma += static_cast<uint64_t>(iteracoes);
        }
    }

    bloco.soma_iteracoes = soma;   // um único escritor por bloco: sem corrida
}

/* Função com a assinatura exigida pela biblioteca (FuncTarefa). */
void calcula_bloco(void *dados, Trabalhador &)
{
    calcula(*static_cast<Bloco *>(dados));
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
    if (argc != 5 && argc != 8) {
        std::fprintf(stderr,
                     "Uso: %s <num_trabalhadores> <estatica|roubo|sequencial> "
                     "<semente> <tamanho_bloco> [largura altura max_iteracoes]\n"
                     "  padrão: 600 450 1000\n",
                     argv[0]);
        return 1;
    }

    /* Dimensões opcionais: lidas ANTES, pois o tamanho do bloco é limitado pela
       largura. */
    if (argc == 8) {
        long largura, altura, max_iter;
        if (!le_inteiro(argv[5], 1, 8192, largura) ||
            !le_inteiro(argv[6], 1, 8192, altura) ||
            !le_inteiro(argv[7], 1, 1000000, max_iter)) {
            std::fprintf(stderr, "Largura e altura devem estar em [1, 8192] e "
                                 "max_iteracoes em [1, 1000000].\n");
            return 1;
        }
        LARGURA = static_cast<int>(largura);
        ALTURA = static_cast<int>(altura);
        MAX_ITERACOES = static_cast<int>(max_iter);
    }

    long num_trabalhadores;
    long semente;
    long tamanho_bloco;
    if (!le_inteiro(argv[1], 1, UINT_MAX, num_trabalhadores) ||
        !le_inteiro(argv[3], 0, LONG_MAX, semente) ||
        !le_inteiro(argv[4], 1, LARGURA, tamanho_bloco)) {
        std::fprintf(stderr,
                     "Trabalhadores e tamanho do bloco devem ser positivos; "
                     "semente deve ser um inteiro não negativo.\n");
        return 1;
    }

    /* "sequencial" roda sem a biblioteca: é o valor de referência que todas as
       execuções devem reproduzir (seção 10.8). */
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

    /* Monta os blocos quadrados em ordem de linhas. */
    const int lado = static_cast<int>(tamanho_bloco);
    std::vector<Bloco> blocos;
    for (int y = 0; y < ALTURA; y += lado) {
        for (int x = 0; x < LARGURA; x += lado) {
            Bloco b;
            b.x0 = x;
            b.x1 = std::min(LARGURA, x + lado);
            b.y0 = y;
            b.y1 = std::min(ALTURA, y + lado);
            blocos.push_back(b);
        }
    }
    /* A partir daqui `blocos` não cresce mais: os endereços guardados pelo
       escalonador em cada tarefa continuam válidos. */
    const std::size_t quantidade_blocos = blocos.size();

    try {
        uint64_t tarefas_executadas = 0;

        if (sequencial) {
            for (Bloco &b : blocos) calcula(b);
            tarefas_executadas = quantidade_blocos;
        } else {
            Escalonador escalonador(static_cast<unsigned>(num_trabalhadores),
                                    politica,
                                    static_cast<uint64_t>(semente));
            for (Bloco &b : blocos) escalonador.submeter(calcula_bloco, &b);
            tarefas_executadas = escalonador.executar();
        }

        uint64_t soma_iteracoes = 0;
        for (const Bloco &bloco : blocos)
            soma_iteracoes += bloco.soma_iteracoes;

        std::printf("Imagem: %dx%d (máx. %d iterações)\n", LARGURA, ALTURA,
                    MAX_ITERACOES);
        std::printf("Blocos submetidos: %zu\n", quantidade_blocos);
        std::printf("Tarefas executadas: %llu\n",
                    static_cast<unsigned long long>(tarefas_executadas));
        std::printf("Soma das iterações: %llu\n",
                    static_cast<unsigned long long>(soma_iteracoes));
    } catch (const std::exception &erro) {
        std::fprintf(stderr, "Erro ao executar o workload: %s\n", erro.what());
        return 1;
    }

    return 0;
}
