# x86
# CC = gcc
# CFLAGS_MATMUL = -g -O3 -std=c99 -ffinite-math-only -fno-signaling-nans -fcx-limited-range -fno-math-errno \
#          -fno-trapping-math -fassociative-math -fno-signed-zeros -funroll-loops -ftree-vectorize
# CFLAGS = -g -O3 -std=c99

# ARM
# CC = arm-linux-gnueabi-gcc
CC = aarch64-linux-gnu-gcc
CFLAGS_MATMUL = -g -O3 -std=c99 -ffinite-math-only -fno-signaling-nans -fcx-limited-range -fno-math-errno \
         -fno-trapping-math -fassociative-math -fno-signed-zeros -funroll-loops -ftree-vectorize -static
CFLAGS = -g -O3 -std=c99 -static

# riscv
# CC = riscv64-linux-gnu-gcc
# CFLAGS_MATMUL = -g -O3 -std=c99 -ffinite-math-only -fno-signaling-nans -fcx-limited-range -fno-math-errno \
#          -fno-trapping-math -fassociative-math -fno-signed-zeros -funroll-loops -ftree-vectorize -static
# CFLAGS = -g -O3 -std=c99 -static


BFS_DIR = breadthFirstSearch

SHA_DIR = sha-src

BINARIES = blocked-matmul queens sha BFS

SOURCES = gem5script.py blocked-matmul.c queens.c

INPUTS = $(wildcard inputs/*.graph inputs/*.txt)

DOCS = README.md

# default target
all: $(BINARIES)

blocked-matmul: CFLAGS = $(CFLAGS_MATMUL)
blocked-matmul: blocked-matmul.c

$(BFS_DIR)/deterministicBFS/BFS:
	cd $(BFS_DIR)/deterministicBFS && make BFS

BFS: $(BFS_DIR)/deterministicBFS/BFS
	cp $< $@

queens: queens.c
	$(CC) $(CFLAGS) $< -o $@

$(SHA_DIR)/sha:
	cd $(SHA_DIR) && make sha

sha: $(SHA_DIR)/sha
	cp $< $@

clean:
	rm -f $(BINARIES)
	cd $(BFS_DIR)/deterministicBFS && make clean
	cd $(BFS_DIR)/graphData && make clean
	cd $(SHA_DIR) && make clean

package: all
	rm -rf to-archive
	mkdir -p to-archive
	cp -r Makefile inputs $(DOCS) $(SOURCES) $(BINARIES) $(SHA_DIR) $(BFS_DIR) to-archive
	rm -f hw2-benchmarks.tar.gz
	tar -C to-archive --transform 's!^\.!./benchmarks!' -zcvf hw2-benchmarks.tar.gz .

.PHONY: all clean package $(MIN_SPANNING_FOREST_DIR)/serialMST/MST $(SHA_DIR)/sha $(BFS_DIR)/deterministicBFS/BFS
