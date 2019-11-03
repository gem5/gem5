#include "copyright.h"
#include "qt.h"
#include "stp.h"

#ifndef NULL
#define NULL	0
#endif

#define STP_STKSIZE (0x1000)

/* `alignment' must be a power of 2. */
#define STP_STKALIGN(sp, alignment) \
  ((void *)((((qt_word_t)(sp)) + (alignment) - 1) & ~((alignment)-1)))


/* The notion of a thread is merged with the notion of a queue.
   Thread stuff: thread status (sp) and stuff to use during
   (re)initialization.  Queue stuff: next thread in the queue
   (next). */

struct stp_t {
  qt_t *sp;              /* QuickThreads handle. */
  void *sto;             /* `malloc'-allocated stack. */
  struct stp_t *next;    /* Next thread in the queue. */
};


/* A queue is a circular list of threads.  The queue head is a
   designated list element.  If this is a uniprocessor-only
   implementation we can store the `main' thread in this, but in a
   multiprocessor there are several `heavy' threads but only one run
   queue.  A fancier implementation might have private run queues,
   which would lead to a simpler (trivial) implementation */

typedef struct stp_q_t {
  stp_t t;
  stp_t *tail;
} stp_q_t;


/* Helper functions. */

extern void *malloc (unsigned size);
extern void perror (char const *msg);
extern void free (void *sto);

  void *
xmalloc (unsigned size)
{
  void *sto;

  sto = malloc (size);
  if (!sto) {
    perror ("malloc");
    exit (1);
  }
  return (sto);
}

/* Queue access functions. */

  static void
stp_qinit (stp_q_t *q)
{
  q->t.next = q->tail = &q->t;
}


  static stp_t *
stp_qget (stp_q_t *q)
{
  stp_t *t;

  t = q->t.next;
  q->t.next = t->next;
  if (t->next == &q->t) {
    if (t == &q->t) {		/* If it was already empty .. */
      return (NULL);		/* .. say so. */
    }
    q->tail = &q->t;		/* Else now it is empty. */
  }
  return (t);
}


  static void
stp_qput (stp_q_t *q, stp_t *t)
{
  q->tail->next = t;
  t->next = &q->t;
  q->tail = t;
}


/* Thread routines. */

static stp_q_t stp_global_runq;	/* A queue of runable threads. */
static stp_t stp_global_main;   /* Thread for the process. */
static stp_t *stp_global_curr;	/* Currently-executing thread. */

static void *stp_starthelp (qt_t *old, void *ignore0, void *ignore1);
static void stp_only (void *pu, void *pt, qt_userf_t *f);
static void *stp_aborthelp (qt_t *sp, void *old, void *null);
static void *stp_yieldhelp (qt_t *sp, void *old, void *blockq);


  void
stp_init()
{
  stp_qinit (&stp_global_runq);
}


  void
stp_start()
{
  stp_t *next;

  while ((next = stp_qget (&stp_global_runq)) != NULL) {
    stp_global_curr = next;
    QT_BLOCK (stp_starthelp, 0, 0, next->sp);
  }
}


  static void *
stp_starthelp (qt_t *old, void *ignore0, void *ignore1)
{
  stp_global_main.sp = old;
  stp_qput (&stp_global_runq, &stp_global_main);
  /* return (garbage); */
}


  void
stp_create (stp_userf_t *f, void *pu)
{
  stp_t *t;
  void *sto;

  t = xmalloc (sizeof(stp_t));
  t->sto = xmalloc (STP_STKSIZE);
  sto = STP_STKALIGN (t->sto, QT_STKALIGN);
  t->sp = QT_SP (sto, STP_STKSIZE - QT_STKALIGN);
  t->sp = QT_ARGS (t->sp, pu, t, (qt_userf_t *)f, stp_only);
  stp_qput (&stp_global_runq, t);
}


  static void
stp_only (void *pu, void *pt, qt_userf_t *f)
{
  stp_global_curr = (stp_t *)pt;
  (*(stp_userf_t *)f)(pu);
  stp_abort();
  /* NOTREACHED */
}


  void
stp_abort (void)
{
  stp_t *old, *newthread;

  newthread = stp_qget (&stp_global_runq);
  old = stp_global_curr;
  stp_global_curr = newthread;
  QT_ABORT (stp_aborthelp, old, (void *)NULL, newthread->sp);
}


  static void *
stp_aborthelp (qt_t *sp, void *old, void *null)
{
  free (((stp_t *)old)->sto);
  free (old);
  /* return (garbage); */
}


  void
stp_yield()
{
  stp_t *old, *newthread;

  newthread = stp_qget (&stp_global_runq);
  old = stp_global_curr;
  stp_global_curr = newthread;
  QT_BLOCK (stp_yieldhelp, old, &stp_global_runq, newthread->sp);
}


  static void *
stp_yieldhelp (qt_t *sp, void *old, void *blockq)
{
  ((stp_t *)old)->sp = sp;
  stp_qput ((stp_q_t *)blockq, (stp_t *)old);
  /* return (garbage); */
}
