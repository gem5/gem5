/* meas.c -- measure qt stuff. */

#include "copyright.h"

/* Need this to get assertions under Mach on the Sequent/i386: */
#ifdef __i386__
#define assert(ex) \
  do { \
    if (!(ex)) { \
      fprintf (stderr, "[%s:%d] Assertion " #ex " failed\n", __FILE__, __LINE__); \
      abort(); \
    } \
  } while (0)
#else
#include <assert.h>
#endif

/* This really ought to be defined in some ANSI include file (*I*
   think...), but it's defined here instead, which leads us to another
   machine dependency.

   The `iaddr_t' type is an integer representation of a pointer,
   suited for doing arithmetic on addresses, e.g. to round an address
   to an alignment boundary. */
typedef unsigned long iaddr_t;

#include <stdarg.h>	/* For varargs tryout. */
#include <stdio.h>
#include "b.h"
#include "qt.h"
#include "stp.h"

extern void exit (int status);
extern int atoi (char const *s);
extern int fprintf (FILE *out, char const *fmt, ...);
extern int fputs (char const *s, FILE *fp);
extern void free (void *sto);
extern void *malloc (unsigned nbytes);
extern void perror (char const *s);

void usage (void);
void tracer(void);

/* Round `v' to be `a'-aligned, assuming `a' is a power of two. */
#define ROUND(v, a)	(((v) + (a) - 1) & ~((a)-1))

typedef struct thread_t {
  qt_t *qt;		/* Pointer to thread of function... */
  void *stk;
  void *top;		/* Set top of stack if reuse. */
  struct thread_t *next;
} thread_t;


  static thread_t *
t_alloc (void)
{
  thread_t *t;
  int ssz = 0x1000;

  t = malloc (sizeof(thread_t));
  if (!t) {
    perror ("malloc");
    exit (1);
  }
  assert (ssz > QT_STKBASE);
  t->stk = malloc (ssz);
  t->stk = (void *)ROUND (((iaddr_t)t->stk), QT_STKALIGN);
  if (!t->stk) {
    perror ("malloc");
    exit (1);
  }
  assert ((((iaddr_t)t->stk) & (QT_STKALIGN-1)) == 0);
  t->top = QT_SP (t->stk, ssz - QT_STKBASE);

  return (t);
}


  static thread_t *
t_create (qt_only_t *starter, void *p0, qt_userf_t *f)
{
  thread_t *t;

  t = t_alloc();
  t->qt = QT_ARGS (t->top, p0, t, f, starter);
  return (t);
}


  static void
t_free (thread_t *t)
{
  free (t->stk);
  free (t);
}


  static void *
t_null (qt_t *old, void *p1, void *p2)
{
  /* return (garbage); */
}


  static void *
t_splat (qt_t *old, void *oldp, void *null)
{
  *(qt_t **)oldp = old;
  /* return (garbage); */
}


static char const test01_msg[] =
  "*QT_SP(sto,sz), QT_ARGS(top,p0,p1,userf,first)";

static char const *test01_descr[] = {
  "Performs 1 QT_SP and one QT_ARGS per iteration.",
  NULL
};

/* This test gives a guess on how long it takes to initalize
   a thread. */

  static void
test01 (int n)
{
  char stack[QT_STKBASE+QT_STKALIGN];
  char *stk;
  qt_t *top;

  stk = (char *)ROUND (((iaddr_t)stack), QT_STKALIGN);

  {
    int i;

    for (i=0; i<QT_STKBASE; ++i) {
      stk[i] = 0;
    }
  }

  while (n>0) {
    /* RETVALUSED */
    top = QT_SP (stk, QT_STKBASE);	QT_ARGS (top, 0, 0, 0, 0);
#ifdef NDEF
    top = QT_SP (stk, QT_STKBASE);	QT_ARGS (top, 0, 0, 0, 0);
    top = QT_SP (stk, QT_STKBASE);	QT_ARGS (top, 0, 0, 0, 0);
    top = QT_SP (stk, QT_STKBASE);	QT_ARGS (top, 0, 0, 0, 0);
    top = QT_SP (stk, QT_STKBASE);	QT_ARGS (top, 0, 0, 0, 0);

    top = QT_SP (stk, QT_STKBASE);	QT_ARGS (top, 0, 0, 0, 0);
    top = QT_SP (stk, QT_STKBASE);	QT_ARGS (top, 0, 0, 0, 0);
    top = QT_SP (stk, QT_STKBASE);	QT_ARGS (top, 0, 0, 0, 0);
    top = QT_SP (stk, QT_STKBASE);	QT_ARGS (top, 0, 0, 0, 0);
    top = QT_SP (stk, QT_STKBASE);	QT_ARGS (top, 0, 0, 0, 0);

    n -= 10;
#else
    n -= 1;
#endif
  }
}


static char const test02_msg[] = "QT_BLOCKI (0, 0, test02_aux, t->qt)";
static qt_t *rootthread;

  static void
test02_aux1 (void *pu, void *pt, qt_userf_t *f)
{
  QT_ABORT (t_null, 0, 0, rootthread);
}

  static void *
test02_aux2 (qt_t *old, void *farg1, void *farg2)
{
  rootthread = old;
  /* return (garbage); */
}

  static void
test02 (int n)
{
  thread_t *t;

  while (n>0) {
  t = t_create (test02_aux1, 0, 0);
    QT_BLOCKI (test02_aux2, 0, 0, t->qt);
  t_free (t);
  t = t_create (test02_aux1, 0, 0);
    QT_BLOCKI (test02_aux2, 0, 0, t->qt);
  t_free (t);
  t = t_create (test02_aux1, 0, 0);
    QT_BLOCKI (test02_aux2, 0, 0, t->qt);
  t_free (t);
  t = t_create (test02_aux1, 0, 0);
    QT_BLOCKI (test02_aux2, 0, 0, t->qt);
  t_free (t);
  t = t_create (test02_aux1, 0, 0);
    QT_BLOCKI (test02_aux2, 0, 0, t->qt);
  t_free (t);

    n -= 5;
  }
}


static char const test03_msg[] = "QT_BLOCKI (...) test vals are right.";


/* Called by the thread function when it wants to shut down.
   Return a value to the main thread. */

  static void *
test03_aux0 (qt_t *old_is_garbage, void *farg1, void *farg2)
{
  assert (farg1 == (void *)5);
  assert (farg2 == (void *)6);
  return ((void *)15);		/* Some unlikely value. */
}


/* Called during new thread startup by main thread.  Since the new
   thread has never run before, return value is ignored. */

  static void *
test03_aux1 (qt_t *old, void *farg1, void *farg2)
{
  assert (old != NULL);
  assert (farg1 == (void *)5);
  assert (farg2 == (void *)6);
  rootthread = old;
  return ((void *)16);		/* Different than `15'. */
}

  static void
test03_aux2 (void *pu, void *pt, qt_userf_t *f)
{
  assert (pu == (void *)1);
  assert (f == (qt_userf_t *)4);
  QT_ABORT (test03_aux0, (void *)5, (void *)6, rootthread);
}

  static void
test03 (int n)
{
  thread_t *t;
  void *rv;

  while (n>0) {
    t = t_create (test03_aux2, (void *)1, (qt_userf_t *)4);
    rv = QT_BLOCKI (test03_aux1, (void *)5, (void *)6, t->qt);
    assert (rv == (void *)15);
    t_free (t);

    --n;
  }
}


static char const test04_msg[] = "stp_start w/ no threads.";

  static void
test04 (int n)
{
  while (n>0) {
    stp_init();	stp_start();
    stp_init();	stp_start();
    stp_init();	stp_start();
    stp_init();	stp_start();
    stp_init();	stp_start();

    stp_init();	stp_start();
    stp_init();	stp_start();
    stp_init();	stp_start();
    stp_init();	stp_start();
    stp_init();	stp_start();

    n -= 10;
  }
}


static char const test05_msg[] = "stp w/ 2 yielding thread.";

  static void
test05_aux (void *null)
{
  stp_yield();
  stp_yield();
}

  static void
test05 (int n)
{
  while (n>0) {
    stp_init();
    stp_create (test05_aux, 0);
    stp_create (test05_aux, 0);
    stp_start();

    --n;
  }
}


static char const test06_msg[] = "*QT_ARGS(...), QT_BLOCKI one thread";

static char const *test06_descr[] = {
  "Does a QT_ARGS, QT_BLOCKI to a helper function that saves the",
  "stack pointer of the main thread, calls an `only' function that",
  "saves aborts the thread, calling a null helper function.",
  ":: start/stop = QT_ARGS + QT_BLOCKI + QT_ABORT + 3 procedure calls.",
  NULL
};

/* This test initializes a thread, runs it, then returns to the main
   program, which reinitializes the thread, runs it again, etc.  Each
   iteration corresponds to 1 init, 1 abort, 1 block. */

static qt_t *test06_sp;


  static void
test06_aux2 (void *null0a, void *null1b, void *null2b, qt_userf_t *null)
{
  QT_ABORT (t_null, 0, 0, test06_sp);
}


  static void *
test06_aux3 (qt_t *sp, void *null0c, void *null1c)
{
  test06_sp = sp;
  /* return (garbage); */
}


  static void
test06 (int n)
{
  thread_t *t;

  t = t_create (0, 0, 0);

  while (n>0) {
    /* RETVALUSED */
    QT_ARGS (t->top, 0, 0, 0, test06_aux2);
    QT_BLOCKI (test06_aux3, 0, 0, t->qt);
#ifdef NDEF
    /* RETVALUSED */
    QT_ARGS (t->top, 0, 0, 0, test06_aux2);
    QT_BLOCKI (test06_aux3, 0, 0, t->qt);

    /* RETVALUSED */
    QT_ARGS (t->top, 0, 0, 0, test06_aux2);
    QT_BLOCKI (test06_aux3, 0, 0, t->qt);

    /* RETVALUSED */
    QT_ARGS (t->top, 0, 0, 0, test06_aux2);
    QT_BLOCKI (test06_aux3, 0, 0, t->qt);

    /* RETVALUSED */
    QT_ARGS (t->top, 0, 0, 0, test06_aux2);
    QT_BLOCKI (test06_aux3, 0, 0, t->qt);

    n -= 5;
#else
    --n;
#endif
  }
}

static char test07_msg[] = "*cswap between threads";

static char const *test07_descr[] = {
  "Build a chain of threads where each thread has a fixed successor.",
  "There is no scheduling performed.  Each thread but one is a loop",
  "that simply blocks with QT_BLOCKI, calling a helper that saves the",
  "current stack pointer.  The last thread decrements a count, and,",
  "if zero, aborts back to the main thread.  Else it continues with",
  "the blocking chain.  The count is divided by the number of threads",
  "in the chain, so `n' is the number of integer block operations.",
  ":: integer cswap = QT_BLOCKI + a procedure call.",
  NULL
};

/* This test repeatedly blocks a bunch of threads.
   Each iteration corresponds to one block operation.

   The threads are arranged so that there are TEST07_N-1 of them that
   run `test07_aux2'.  Each one of those blocks saving it's sp to
   storage owned by the preceding thread; a pointer to that storage is
   passed in via `mep'.  Each thread has a handle on it's own storage
   for the next thread, referenced by `nxtp', and it blocks by passing
   control to `*nxtp', telling the helper function to save its state
   in `*mep'.  The last thread in the chain decrements a count and, if
   it's gone below zero, returns to `test07'; otherwise, it invokes
   the first thread in the chain. */

static qt_t *test07_heavy;

#define TEST07_N (4)


  static void
test07_aux2 (void *null0, void *mep, void *nxtp, qt_userf_t *null)
{
  qt_t *nxt;

  while (1) {
    nxt = *(qt_t **)nxtp;
#ifdef NDEF
    printf ("Helper 0x%p\n", nxtp);
#endif
    QT_BLOCKI (t_splat, mep, 0, nxt);
  }
}

  static void
test07_aux3 (void *np, void *mep, void *nxtp, qt_userf_t *null)
{
  int n;

  n = *(int *)np;
  while (1) {
    n -= TEST07_N;
    if (n<0) {
      QT_ABORT (t_splat, mep, 0, test07_heavy);
    }
    QT_BLOCKI (t_splat, mep, 0, *(qt_t **)nxtp);
  }
}


  static void
test07 (int n)
{
  int i;
  thread_t *t[TEST07_N];

  for (i=0; i<TEST07_N; ++i) {
    t[i] = t_create (0, 0, 0);
  }
  for (i=0; i<TEST07_N-1; ++i) {
    /* RETVALUSED */
    QT_ARGS (t[i]->top, 0, &t[i]->qt, &t[i+1]->qt, test07_aux2);
  }
  /* RETVALUSED */
  QT_ARGS (t[i]->top, &n, &t[TEST07_N-1]->qt, &t[0]->qt, test07_aux3);
  QT_BLOCKI (t_splat, &test07_heavy, 0, t[0]->qt);
}


static char test08_msg[] = "Floating-point cswap between threads";

static char const *test08_descr[] = {
  "Measure context switch times including floating-point, use QT_BLOCK.",
  NULL
};

static qt_t *test08_heavy;

#define TEST08_N (4)


  static void
test08_aux2 (void *null0, void *mep, void *nxtp, qt_userf_t *null)
{
  qt_t *nxt;

  while (1) {
    nxt = *(qt_t **)nxtp;
    QT_BLOCK (t_splat, mep, 0, nxt);
  }
}

  static void
test08_aux3 (void *np, void *mep, void *nxtp, qt_userf_t *null)
{
  int n;

  n = *(int *)np;
  while (1) {
    n -= TEST08_N;
    if (n<0) {
      QT_ABORT (t_splat, mep, 0, test08_heavy);
    }
    QT_BLOCK (t_splat, mep, 0, *(qt_t **)nxtp);
  }
}


  static void
test08 (int n)
{
  int i;
  thread_t *t[TEST08_N];

  for (i=0; i<TEST08_N; ++i) {
    t[i] = t_create (0, 0, 0);
  }
  for (i=0; i<TEST08_N-1; ++i) {
    /* RETVALUSED */
    QT_ARGS (t[i]->top, 0, &t[i]->qt, &t[i+1]->qt, test08_aux2);
  }
  /* RETVALUSED */
  QT_ARGS (t[i]->top, &n, &t[TEST08_N-1]->qt, &t[0]->qt, test08_aux3);
  QT_BLOCK (t_splat, &test08_heavy, 0, t[0]->qt);
}


/* Test the varargs procedure calling. */

char const test09_msg[] = { "Start and run threads using varargs." };

thread_t *test09_t0, *test09_t1, *test09_t2, *test09_main;

  thread_t *
test09_create (qt_startup_t *start, qt_vuserf_t *f,
	       qt_cleanup_t *cleanup, int nbytes, ...)
{
  va_list ap;
  thread_t *t;

  t = t_alloc();
  va_start (ap, nbytes);
  t->qt = QT_VARGS (t->top, nbytes, ap, t, start, f, cleanup);
  va_end (ap);
  return (t);
}


  static void
test09_cleanup (void *pt, void *vuserf_retval)
{
  assert (vuserf_retval == (void *)17);
  QT_ABORT (t_splat, &((thread_t *)pt)->qt, 0,
	    ((thread_t *)pt)->next->qt);
}


  static void
test09_start (void *pt)
{
}


  static void *
test09_user0 (void)
{
  QT_BLOCKI (t_splat, &test09_t0->qt, 0, test09_t1->qt);
  return ((void *)17);
}

  static void *
test09_user2 (int one, int two)
{
  assert (one == 1);
  assert (two == 2);
  QT_BLOCKI (t_splat, &test09_t1->qt, 0, test09_t2->qt);
  assert (one == 1);
  assert (two == 2);
  return ((void *)17);
}

  static void *
test09_user10 (int one, int two, int three, int four, int five,
	      int six, int seven, int eight, int nine, int ten)
{
  assert (one == 1);
  assert (two == 2);
  assert (three == 3);
  assert (four == 4);
  assert (five == 5);
  assert (six == 6);
  assert (seven == 7);
  assert (eight == 8);
  assert (nine == 9);
  assert (ten == 10);
  QT_BLOCKI (t_splat, &test09_t2->qt, 0, test09_main->qt);
  assert (one == 1);
  assert (two == 2);
  assert (three == 3);
  assert (four == 4);
  assert (five == 5);
  assert (six == 6);
  assert (seven == 7);
  assert (eight == 8);
  assert (nine == 9);
  assert (ten == 10);
  return ((void *)17);
}


  void
test09 (int n)
{
  thread_t main;

  test09_main = &main;

  while (--n >= 0) {
    test09_t0 = test09_create (test09_start, (qt_vuserf_t*)test09_user0,
			       test09_cleanup, 0);
    test09_t1 = test09_create (test09_start, (qt_vuserf_t*)test09_user2,
			       test09_cleanup, 2 * sizeof(qt_word_t), 1, 2);
    test09_t2 = test09_create (test09_start, (qt_vuserf_t*)test09_user10,
			       test09_cleanup, 10 * sizeof(qt_word_t),
			       1, 2, 3, 4, 5, 6, 7, 8, 9, 10);

    /* Chaining used by `test09_cleanup' to determine who is next. */
    test09_t0->next = test09_t1;
    test09_t1->next = test09_t2;
    test09_t2->next = test09_main;

    QT_BLOCKI (t_splat, &test09_main->qt, 0, test09_t0->qt);
    QT_BLOCKI (t_splat, &test09_main->qt, 0, test09_t0->qt);

    t_free (test09_t0);
    t_free (test09_t1);
    t_free (test09_t2);
  }
}


/* Test 10/11/12: time the cost of various number of args. */

char const test10_msg[] = { "*Test varargs init & startup w/ 0 args." };

char const *test10_descr[] = {
  "Start and stop threads that use variant argument lists (varargs).",
  "Each thread is initialized by calling a routine that calls",
  "QT_VARARGS.  Then runs the thread by calling QT_BLOCKI to hald the",
  "main thread, a helper that saves the main thread's stack pointer,",
  "a null startup function, a null user function, a cleanup function",
  "that calls QT_ABORT and restarts the main thread.  Copies no user",
  "parameters.",
  ":: varargs start/stop = QT_BLOCKI + QT_ABORT + 6 function calls.",
  NULL
};

/* Helper function to send control back to main.
   Don't save anything. */


/* Helper function for starting the varargs thread.  Save the stack
   pointer of the main thread so we can get back there eventually. */


/* Startup function for a varargs thread. */

  static void
test10_startup (void *pt)
{
}


/* User function for a varargs thread. */

  static void *
test10_run (int arg0, ...)
{
  /* return (garbage); */
}


/* Cleanup function for a varargs thread.  Send control
   back to the main thread.  Don't save any state from the thread that
   is halting. */

  void
test10_cleanup (void *pt, void *vuserf_retval)
{
  QT_ABORT (t_null, 0, 0, ((thread_t *)pt)->qt);
}


  void
test10_init (thread_t *new, thread_t *next, int nbytes, ...)
{
  va_list ap;

  va_start (ap, nbytes);
  new->qt = QT_VARGS (new->top, nbytes, ap, next, test10_startup,
		      test10_run, test10_cleanup);
  va_end (ap);
}


  void
test10 (int n)
{
  thread_t main;
  thread_t *t;

  t = t_alloc();
  t->next = &main;

  while (--n >= 0) {
    test10_init (t, &main, 0);
    QT_BLOCKI (t_splat, &main.qt, 0, t->qt);
  }
  t_free (t);
}


char const test11_msg[] = { "*Test varargs init & startup w/ 2 args." };

char const *test11_descr[] = {
  "Varargs initialization/run.  Copies 2 user arguments.",
  ":: varargs 2 start/stop = QT_VARGS(2 args), QT_BLOCKI, QT_ABORT, 6 f() calls.",
  NULL
};


  void
test11 (int n)
{
  thread_t main;
  thread_t *t;

  t = t_alloc();
  t->next = &main;

  while (--n >= 0) {
    test10_init (t, &main, 2 * sizeof(int), 2, 1);
    QT_BLOCKI (t_splat, &main.qt, 0, t->qt);
  }
  t_free (t);
}

char const test12_msg[] = { "*Test varargs init & startup w/ 4 args." };

char const *test12_descr[] = {
  "Varargs initialization/run.  Copies 4 user arguments.",
  ":: varargs 4 start/stop = QT_VARGS(4 args), QT_BLOCKI, QT_ABORT, 6 f() calls.",
  NULL
};


  void
test12 (int n)
{
  thread_t main;
  thread_t *t;

  t = t_alloc();
  t->next = &main;

  while (--n >= 0) {
    test10_init (t, &main, 4 * sizeof(int), 4, 3, 2, 1);
    QT_BLOCKI (t_splat, &main.qt, 0, t->qt);
  }
  t_free (t);
}


char const test13_msg[] = { "*Test varargs init & startup w/ 8 args." };

char const *test13_descr[] = {
  "Varargs initialization/run.  Copies 8 user arguments.",
  ":: varargs 8 start/stop = QT_VARGS(8 args), QT_BLOCKI, QT_ABORT, 6 f() calls.",
  NULL
};

  void
test13 (int n)
{
  thread_t main;
  thread_t *t;

  t = t_alloc();
  t->next = &main;

  while (--n >= 0) {
    test10_init (t, &main, 8 * sizeof(int), 8, 7, 6, 5, 4, 3, 2, 1);
    QT_BLOCKI (t_splat, &main.qt, 0, t->qt);
  }
  t_free (t);
}


char const test14_msg[] = { "*Test varargs initialization w/ 0 args." };

char const *test14_descr[] = {
  "Varargs initialization without running the thread.  Just calls",
  "QT_VARGS.",
  ":: varargs 0 init = QT_VARGS()",
  NULL
};

  void
test14 (int n)
{
  thread_t main;
  thread_t *t;

  t = t_alloc();
  t->next = &main;

  while (--n >= 0) {
    test10_init (t, &main, 0 * sizeof(int));
  }
  t_free (t);
}


char const test15_msg[] = { "*Test varargs initialization w/ 2 args." };

char const *test15_descr[] = {
  "Varargs initialization without running the thread.  Just calls",
  "QT_VARGS.",
  ":: varargs 2 init = QT_VARGS(2 args)",
  NULL
};

  void
test15 (int n)
{
  thread_t main;
  thread_t *t;

  t = t_alloc();
  t->next = &main;

  while (--n >= 0) {
    test10_init (t, &main, 2 * sizeof(int), 2, 1);
  }
  t_free (t);
}

char const test16_msg[] = { "*Test varargs initialization w/ 4 args." };

char const *test16_descr[] = {
  "Varargs initialization without running the thread.  Just calls",
  "QT_VARGS.",
  ":: varargs 4 init = QT_VARGS(4 args)",
  NULL
};


  void
test16 (int n)
{
  thread_t main;
  thread_t *t;

  t = t_alloc();
  t->next = &main;

  while (--n >= 0) {
    test10_init (t, &main, 4 * sizeof(int), 4, 3, 2, 1);
  }
  t_free (t);
}


char const test17_msg[] = { "*Test varargs initialization w/ 8 args." };

char const *test17_descr[] = {
  "Varargs initialization without running the thread.  Just calls",
  "QT_VARGS.",
  ":: varargs 8 init = QT_VARGS(8 args)",
  NULL
};


  void
test17 (int n)
{
  thread_t main;
  thread_t *t;

  t = t_alloc();
  t->next = &main;

  while (--n >= 0) {
    test10_init (t, &main, 8 * sizeof(int), 8, 7, 6, 5, 4, 3, 2, 1);
  }
  t_free (t);
}

/* Test times for basic machine operations. */

char const test18_msg[] = { "*Call register indirect." };
char const *test18_descr[] = { NULL };

  void
test18 (int n)
{
  b_call_reg (n);
}


char const test19_msg[] = { "*Call immediate." };
char const *test19_descr[] = { NULL };

  void
test19 (int n)
{
  b_call_imm (n);
}


char const test20_msg[] = { "*Add register-to-register." };
char const *test20_descr[] = { NULL };

  void
test20 (int n)
{
  b_add (n);
}


char const test21_msg[] = { "*Load memory to a register." };
char const *test21_descr[] = { NULL };

  void
test21 (int n)
{
  b_load (n);
}

/* Driver. */

typedef struct foo_t {
    char const *msg;	/* Message to print for generic help. */
    char const **descr;	/* A description of what is done by the test. */
    void (*f)(int n);
} foo_t;


static foo_t foo[] = {
  { "Usage:\n", NULL, (void(*)(int n))usage },
  { test01_msg, test01_descr, test01 },
  { test02_msg, NULL, test02 },
  { test03_msg, NULL, test03 },
  { test04_msg, NULL, test04 },
  { test05_msg, NULL, test05 },
  { test06_msg, test06_descr, test06 },
  { test07_msg, test07_descr, test07 },
  { test08_msg, test08_descr, test08 },
  { test09_msg, NULL, test09 },
  { test10_msg, test10_descr, test10 },
  { test11_msg, test11_descr, test11 },
  { test12_msg, test12_descr, test12 },
  { test13_msg, test13_descr, test13 },
  { test14_msg, test14_descr, test14 },
  { test15_msg, test15_descr, test15 },
  { test16_msg, test16_descr, test16 },
  { test17_msg, test17_descr, test17 },
  { test18_msg, test18_descr, test18 },
  { test19_msg, test19_descr, test19 },
  { test20_msg, test20_descr, test20 },
  { test21_msg, test21_descr, test21 },
  { 0, 0 }
};

static int tv = 0;

  void
tracer ()
{

  fprintf (stderr, "tracer\t%d\n", tv++);
  fflush (stderr);
}

  void
tracer2 (void *val)
{
  fprintf (stderr, "tracer2\t%d val=0x%p", tv++, val);
  fflush (stderr);
}


  void
describe()
{
  int i;
  FILE *out = stdout;

  for (i=0; foo[i].msg; ++i) {
    if (foo[i].descr) {
      int j;

      putc ('\n', out);
      fprintf (out, "[%d]\n", i);
      for (j=0; foo[i].descr[j]; ++j) {
	fputs (foo[i].descr[j], out);
	putc ('\n', out);
      }
    }
  }
  exit (0);
}


  void
usage()
{
  int i;

  fputs (foo[0].msg, stderr);
  for (i=1; foo[i].msg; ++i) {
    fprintf (stderr, "%2d\t%s\n", i, foo[i].msg);
  }
  exit (1);
}


  void
args (int *which, int *n, int argc, char **argv)
{
  static int nfuncs = 0;

  if (argc == 2 && argv[1][0] == '-' && argv[1][1] == 'h') {
    describe();
  }

  if (nfuncs == 0) {
    for (nfuncs=0; foo[nfuncs].msg; ++nfuncs)
      ;
  }

  if (argc != 2 && argc != 3) {
    usage();
  }

  *which = atoi (argv[1]);
  if (*which < 0 || *which >= nfuncs) {
    usage();
  }
  *n = (argc == 3)
    ? atoi (argv[2])
    : 1;
}


  int
main (int argc, char **argv)
{
  int which, n;
  args (&which, &n, argc, argv);
  (*(foo[which].f))(n);
  exit (0);
  return (0);
}
