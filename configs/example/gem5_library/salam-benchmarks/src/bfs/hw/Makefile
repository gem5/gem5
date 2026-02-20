TARGET=bfs.ll top.ll

%.ll : %.c
	clang -O1 -S -target armv7-pc-none-eabi -emit-llvm -o $@ $<
build : $(TARGET)

clean:
	rm -f *.ll
