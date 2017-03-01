#include "copyright.h"
#include "qt.h"

// static void *qt_sp_bottom_save;

#ifdef QT_VARGS_DEFAULT

/* If the stack grows down, `vargs' is a pointer to the lowest
   address in the block of arguments.  If the stack grows up, it is a
   pointer to the highest address in the block. */

  qt_t *
qt_vargs (qt_t *sp, int nbytes, void *vargs,
	  void *pt, qt_startup_t *startup,
	  qt_vuserf_t *vuserf, qt_cleanup_t *cleanup)
{
  int i;

  sp = QT_VARGS_MD0 (sp, nbytes);
#ifdef QT_GROW_UP
    for (i=nbytes/sizeof(qt_word_t); i>0; --i) {
      QT_SPUT (QT_VARGS_ADJUST(sp), i, ((qt_word_t *)vargs)[-i]);
    }
#else
    for (i=nbytes/sizeof(qt_word_t); i>0; --i) {
      QT_SPUT (QT_VARGS_ADJUST(sp), i-1, ((qt_word_t *)vargs)[i-1]);
    }
#endif

  QT_VARGS_MD1 (QT_VADJ(sp));
  QT_SPUT (QT_VADJ(sp), QT_VARGT_INDEX, pt);
  QT_SPUT (QT_VADJ(sp), QT_VSTARTUP_INDEX, startup);
  QT_SPUT (QT_VADJ(sp), QT_VUSERF_INDEX, vuserf);
  QT_SPUT (QT_VADJ(sp), QT_VCLEANUP_INDEX, cleanup);
  return ((qt_t *)QT_VADJ(sp));
}
#endif /* def QT_VARGS_DEFAULT */

#ifdef __cplusplus
extern "C"
#endif
  void
qt_null (void)
{
}

#ifdef __cplusplus
extern "C"
#endif
  void
qt_error (void)
{
  extern void abort(void);

  abort();
}
