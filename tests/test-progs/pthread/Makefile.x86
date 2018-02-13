# x86 - native

CPP      := g++
OBJDUMP  := objdump
CPPFLAGS := -g -O3 -static -std=c++11
LFLAGS := -pthread -Wl,--whole-archive -lpthread -Wl,--no-whole-archive

SRC_DIR := ./src
BIN_DIR := ./bin

TEST_SRC := test_pthread_create_seq.cpp \
            test_pthread_create_para.cpp \
            test_pthread_mutex.cpp \
            test_atomic.cpp \
            test_pthread_cond.cpp \
            test_std_thread.cpp \
            test_std_mutex.cpp \
            test_std_condition_variable.cpp \

TEST_BIN  := $(patsubst %.cpp,%,$(TEST_SRC))
TEST_DUMP := $(patsubst %.cpp,%.objdump,$(TEST_SRC))

# Rules

.PHONY: default clean

default: $(TEST_BIN)

$(TEST_BIN):
	mkdir -p $(BIN_DIR)
	$(CPP) $(CPPFLAGS) $(SRC_DIR)/$@.cpp $(LFLAGS) -o $(BIN_DIR)/$@

clean:
	$(RM) -r $(BIN_DIR)/*
