#include "lib.h"
#include "xxm.h"

unsigned char ppecRegRead(int, unsigned char);
void ppecRegWrite(int, unsigned char, unsigned char);
int ppecOpen(int);
void ppecClose(int);
ul ppecMemAddr(int);
ul ppecIOAddr(int, int, unsigned long, int);

typedef unsigned char	char2[2];
struct pcmcia {
        int   socket;
        char2 *cor;
        char2 *cis;
};

struct _pcmciaTuples {
  int count;
  unsigned char *tuples[32];
  unsigned char buf[256];
};

struct _ppecSlotInfo {
  int cardPresent;
  struct _pcmciaTuples tuples;
};

extern struct _ppecSlotInfo ppecSlotInfo[];




