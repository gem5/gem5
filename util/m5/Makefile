all: m5

m5: m5.o m5op.o
	cc -o m5 m5.o m5op.o

m5op.o: m5op.s
	as -o m5op.o m5op.s

m5.o: m5.c
	cc -c -o m5.o m5.c

clean:
	@rm -f m5 *.o *~
