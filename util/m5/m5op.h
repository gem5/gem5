#ifndef __M5OP_H__
#define __M5OP_H__

void arm(unsigned long address);
void quiesce();
void ivlb(unsigned long interval);
void ivle(unsigned long interval);
void m5exit();
unsigned long initparam();

#endif // __M5OP_H__
