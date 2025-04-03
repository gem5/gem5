CC = riscv64-unknown-linux-gnu-gcc
CFLAGS = -static

TARGET = lax

SRC = lax_test.c

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $@ $<

clean:
	rm -f $(TARGET)
