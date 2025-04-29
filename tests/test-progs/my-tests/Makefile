CC = riscv64-unknown-linux-gnu-gcc
CFLAGS = -static

TARGET = lax

SRC = cache.c

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $@ $<

clean:
	rm -f $(TARGET)
