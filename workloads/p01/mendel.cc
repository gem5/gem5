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

// A imagem e o limite de iterações são fixos para manter a interface simples.
constexpr int LARGURA = 800;
constexpr int ALTURA = 600;
constexpr int MAX_ITERACOES = 1000;

struct Bloco {
    int primeira_linha;
    int ultima_linha;
    uint64_t soma_iteracoes = 0;
};

void calcula_bloco(void *dados, Trabalhador &)
{
    auto &bloco = *static_cast<Bloco *>(dados);
    uint64_t soma = 0;

    for (int y = bloco.primeira_linha; y < bloco.ultima_linha; ++y) {
        const double ci = (static_cast<double>(y) - ALTURA / 2.0) *
                          4.0 / ALTURA;
        for (int x = 0; x < LARGURA; ++x) {
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

    bloco.soma_iteracoes = soma;
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
                     "Uso: %s <num_trabalhadores> <estatica|roubo> "
                     "<semente> <tamanho_bloco>\n",
                     argv[0]);
        return 1;
    }

    long num_trabalhadores;
    long semente;
    long tamanho_bloco;
    if (!le_inteiro(argv[1], 1, UINT_MAX, num_trabalhadores) ||
        !le_inteiro(argv[3], 0, LONG_MAX, semente) ||
        !le_inteiro(argv[4], 1, ALTURA, tamanho_bloco)) {
        std::fprintf(stderr,
                     "Trabalhadores e tamanho do bloco devem ser positivos; "
                     "semente deve ser um inteiro não negativo.\n");
        return 1;
    }

    Politica politica;
    if (std::string(argv[2]) == "estatica") {
        politica = Politica::Estatica;
    } else if (std::string(argv[2]) == "roubo") {
        politica = Politica::Roubo;
    } else {
        std::fprintf(stderr, "Política inválida: use 'estatica' ou 'roubo'.\n");
        return 1;
    }

    const std::size_t quantidade_blocos =
        (ALTURA + static_cast<std::size_t>(tamanho_bloco) - 1) /
        static_cast<std::size_t>(tamanho_bloco);

    try {
        // O vetor é dimensionado antes da submissão para manter estáveis os
        // endereços guardados pelo escalonador em cada tarefa.
        std::vector<Bloco> blocos(quantidade_blocos);
        Escalonador escalonador(static_cast<unsigned>(num_trabalhadores),
                                politica,
                                static_cast<uint64_t>(semente));

        for (std::size_t i = 0; i < quantidade_blocos; ++i) {
            blocos[i].primeira_linha = static_cast<int>(i * tamanho_bloco);
            blocos[i].ultima_linha = static_cast<int>(
                std::min<std::size_t>(ALTURA, (i + 1) * tamanho_bloco));
            escalonador.submeter(calcula_bloco, &blocos[i]);
        }

        const uint64_t tarefas_executadas = escalonador.executar();
        uint64_t soma_iteracoes = 0;
        for (const Bloco &bloco : blocos)
            soma_iteracoes += bloco.soma_iteracoes;

        std::printf("Imagem: %dx%d\n", LARGURA, ALTURA);
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
