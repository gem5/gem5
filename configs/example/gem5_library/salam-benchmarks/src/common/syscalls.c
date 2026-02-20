#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

int
_read_r(struct _reent *r, int file, char *ptr, int len)
{
    r = r;
    file = file;
    ptr = ptr;
    len = len;

    errno = EINVAL;
    return -1;
}

int
_lseek_r(struct _reent *r, int file, int ptr, int dir)
{
    r = r;
    file = file;
    ptr = ptr;
    dir = dir;

    return 0;
}

// The following two functions implement the basis for printf and the UART
// component in gem5.
volatile unsigned int *const UART0DR = (unsigned int *)0x1c090000;

void
gem5_print(const char *s)
{
    while (*s != '\0') {
        *UART0DR = (unsigned int)(*s);
        s++;
    }
}

int
_write_r(struct _reent *r, int file, char *ptr, int len)
{
    r = r;
    file = file;
    ptr = ptr;

    gem5_print(ptr);

    return len;
}

int
_close_r(struct _reent *r, int file)
{
    return 0;
}

register char *stack_ptr __asm("sp");

caddr_t
_sbrk_r(struct _reent *r, int incr)
{
    extern char end __asm("end");
    static char *heap_end;
    char *prev_heap_end;

    if (heap_end == NULL) {
        heap_end = &end;
    }

    prev_heap_end = heap_end;

    if (heap_end + incr > stack_ptr) {
        errno = ENOMEM;
        return (caddr_t)-1;
    }

    heap_end += incr;
    return (caddr_t)prev_heap_end;
}

int
_fstat_r(struct _reent *r, int file, struct stat *st)
{
    r = r;
    file = file;
    memset(st, 0, sizeof(*st));
    st->st_mode = S_IFCHR;
    return 0;
}

int
_isatty_r(struct _reent *r, int fd)
{
    r = r;
    fd = fd;
    return 1;
}

int
_getpid(int n)
{
    return 1;
    n = n;
}

int
_kill(int pid, int sig)
{
    asm("swi %a0" ::"i"(11));
    return -1; // Never gets here
}

void
_exit(int sig)
{
    _kill(sig, -1);
    while (1)
        ; // Infinite loop to halt execution
    __builtin_unreachable();
}
