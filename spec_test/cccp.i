typedef unsigned char U_CHAR;
extern int target_flags;
enum reg_class { NO_REGS, GENERAL_REGS, FLOAT_REGS, ALL_REGS,
		 LIM_REG_CLASSES };
extern struct rtx_def *alpha_builtin_saveregs ();
extern struct rtx_def *alpha_compare_op0, *alpha_compare_op1;
extern int alpha_compare_fp_p;
extern char *alpha_function_name;
extern char *current_function_name;
typedef struct stringdef STRINGDEF;
struct stringdef
{
  U_CHAR *contents;		
  int len;			
  int writeflag;		
  int lineno;			
  U_CHAR *filename;		
  STRINGDEF *chain;		
  int output_mark;		
};
typedef struct keydef KEYDEF;
struct keydef
{
  STRINGDEF *str;
  KEYDEF *chain;
};
typedef signed long     ptrdiff_t;
    typedef unsigned int  wchar_t;
typedef unsigned int wctype_t;
typedef long            fpos_t;
typedef int            time_t;
typedef int             clock_t;
typedef unsigned long   size_t;
typedef long                    ssize_t; 
typedef	unsigned char	uchar_t;
typedef	unsigned short	ushort_t;
typedef	unsigned int	uint_t;
typedef unsigned long	ulong_t;
typedef	volatile unsigned char	vuchar_t;
typedef	volatile unsigned short	vushort_t;
typedef	volatile unsigned int	vuint_t;
typedef volatile unsigned long	vulong_t;
typedef	struct	{ long r[1]; } *physadr_t;
typedef	struct	label_t	{
	long	val[10];
} label_t;
typedef int		level_t;
typedef	int		daddr_t;	
typedef	char *		caddr_t;	
typedef long *		qaddr_t;        
typedef char *          addr_t;
typedef	uint_t		ino_t;		
typedef short		cnt_t;
typedef int		dev_t;		
typedef	int		chan_t;		
typedef long    off_t;			
typedef unsigned long	rlim_t;		
typedef	int		paddr_t;
typedef	ushort_t	nlink_t;
typedef int    		key_t;		
typedef	uint_t		mode_t;		
typedef uint_t		uid_t;		
typedef uint_t		gid_t;		
typedef	void *		mid_t;		
typedef	int		pid_t;		
typedef char		slab_t[12];	
typedef ulong_t		shmatt_t;	
typedef ulong_t		msgqnum_t;	
typedef ulong_t		msglen_t;	
        typedef unsigned int wint_t;         
typedef unsigned long	sigset_t;
typedef long            timer_t;        
typedef void (*sig_t) ();
typedef pid_t		id_t;		
typedef uint_t	major_t;      
typedef uint_t	minor_t;      
typedef uint_t	devs_t;       
typedef uint_t	unit_t;       
typedef	unsigned long	vm_offset_t;
typedef	unsigned long	vm_size_t;
typedef	uchar_t		uchar;
typedef	ushort_t	ushort;
typedef	uint_t		uint;
typedef ulong_t		ulong;
typedef	physadr_t	physadr;
typedef	uchar_t		u_char;
typedef	ushort_t 	u_short;
typedef	uint_t		u_int;
typedef	ulong_t		u_long;
typedef	vuchar_t	vu_char;
typedef	vushort_t 	vu_short;
typedef	vuint_t		vu_int;
typedef	vulong_t	vu_long;
typedef struct  _quad { int val[2]; } quad;
typedef	long	swblk_t;
typedef u_long	fixpt_t;
typedef int	fd_mask;
typedef	struct fd_set {
	fd_mask	fds_bits[(((4096)+(( (sizeof(fd_mask) * 8		)	)-1))/( (sizeof(fd_mask) * 8		)	))];
} fd_set;
extern void  bzero (); 
struct timeval;
int select ();
struct  stat
{
	dev_t	st_dev;			
	ino_t	st_ino;			
	mode_t	st_mode;		
	nlink_t	st_nlink;		
	uid_t	st_uid;			
	gid_t	st_gid;			
	dev_t	st_rdev;		
	off_t	st_size;		
	time_t	st_atime;		
	int	st_spare1;
	time_t	st_mtime;		
	int	st_spare2;
	time_t	st_ctime;		
	int	st_spare3;
	uint_t	st_blksize;		
        int    st_blocks;              
        uint_t  st_flags;               
        uint_t  st_gen;                 
};
	extern int	mkdir(); 
	extern mode_t	umask(); 
	extern int	stat();
	extern int	fstat();
	extern int	chmod();
	extern int	mkfifo();
	extern int	lstat();
extern int	isalpha ();
extern int	isalnum ();
extern int	iscntrl ();
extern int	isdigit ();
extern int	isgraph ();
extern int	islower ();
extern int	isprint ();
extern int	ispunct ();
extern int	isspace ();
extern int	isupper ();
extern int	isxdigit ();
extern int	toupper ();
extern int	tolower ();
extern int	isascii ();
extern int	toascii ();
extern int	(_toupper) ();
extern int	(_tolower) ();
typedef struct _LC_charmap_t 	_LC_charmap_t;
typedef struct _LC_monetary_t 	_LC_monetary_t;
typedef struct _LC_numeric_t	_LC_numeric_t;
typedef struct _LC_resp_t	_LC_resp_t;
typedef struct _LC_time_t	_LC_time_t;
typedef struct _LC_collate_t	_LC_collate_t;
typedef struct _LC_ctype_t	_LC_ctype_t;
typedef struct _LC_locale_t	_LC_locale_t;
typedef enum __lc_type_id_t {
    _LC_CAR=1,
    _LC_LOCALE=2,
    _LC_CHARMAP=3,
    _LC_CTYPE=4,
    _LC_COLLATE=5,
    _LC_NUMERIC=6,
    _LC_MONETARY=7,
    _LC_TIME=8,
    _LC_RESP=9 } __lc_type_id_t;
typedef struct {
    __lc_type_id_t
	type_id;
    unsigned short
	magic;
    unsigned long
	version;
    unsigned long  size;
} _LC_object_t;
typedef struct {
    _LC_object_t  hdr;
    char     *(*nl_langinfo) ();
    unsigned long   (*mbtowc) ();
    unsigned long   (*mbstowcs) ();
    int      (*wctomb) ();
    unsigned long   (*wcstombs) ();
    int      (*mblen) ();
    unsigned long   (*wcswidth) ();
    unsigned long   (*wcwidth) ();
    int      (*__mbtopc) ();
    int      (*__mbstopcs) ();
    int      (*__pctomb) ();
    int      (*__pcstombs) ();
    _LC_charmap_t *(*init) ();
    void     *data;
} _LC_core_charmap_t;
typedef struct {
    _LC_object_t  hdr; 
    unsigned int  (*towupper) ();
    unsigned int  (*towlower) ();
    unsigned int    (*wctype) ();
    int      (*iswctype) ();
    _LC_ctype_t   *(*init) ();
    void     *data;
} _LC_core_ctype_t;
typedef struct {
    _LC_object_t  hdr;
    int      (*strcoll) ();
    unsigned long   (*strxfrm) ();
    int      (*wcscoll) ();
    unsigned long   (*wcsxfrm) ();
    int      (*fnmatch) ();
    int      (*regcomp) ();
    unsigned long   (*regerror) ();
    int      (*regexec) ();
    void     (*regfree) ();
    _LC_collate_t *(*init) ();
    void     *data;
} _LC_core_collate_t;
struct tm;
typedef struct {
    _LC_object_t  hdr;
    char     *(*nl_langinfo) ();
    unsigned long   (*strftime) ();
    char     *(*strptime) ();
    unsigned long   (*wcsftime) ();
    _LC_time_t    *(*init) ();
    void     *data;
} _LC_core_time_t;
typedef struct {
    _LC_object_t  hdr;
    char     *(*nl_langinfo) ();
    unsigned long   (*strfmon) ();
    _LC_monetary_t        *(*init) ();
    void     *data;
} _LC_core_monetary_t;
typedef struct {
    _LC_object_t  hdr;
    char     *(*nl_langinfo) ();
    _LC_numeric_t *(*init) ();
    void     *data;
} _LC_core_numeric_t;
typedef struct {
    _LC_object_t  hdr;
    char        *(*nl_langinfo) ();
    int		(*rpmatch) ();
    _LC_resp_t    *(*init) ();
    void        *data;
} _LC_core_resp_t;
typedef struct {
    _LC_object_t hdr;
    char         *(*nl_langinfo) ();
    struct lconv * (*localeconv) ();
    _LC_locale_t  *(*init) ();
    void         *data;
} _LC_core_locale_t;
struct _LC_charmap_t {
    _LC_core_charmap_t core;
    char     *cm_csname;	
    unsigned long   cm_mb_cur_max; 
    unsigned long   cm_mb_cur_min; 
    unsigned char
	     cm_max_disp_width; 
};
struct _LC_monetary_t {
    _LC_core_monetary_t   core;
    char *int_curr_symbol;	   
    char *currency_symbol;	   
    char *mon_decimal_point;	   
    char *mon_thousands_sep;	   
    char *mon_grouping;		   
    char *positive_sign;	   
    char *negative_sign;	   
     char int_frac_digits;   
     char frac_digits;	   
     char p_cs_precedes;	   
     char p_sep_by_space;	   
     char n_cs_precedes;	   
     char n_sep_by_space;	   
     char p_sign_posn;	   
     char n_sign_posn;	   
    char *debit_sign;		   
    char *credit_sign;		   
    char *left_parenthesis;	   
    char *right_parenthesis;	   
};
struct _LC_numeric_t {
    _LC_core_numeric_t core;
    char     *decimal_point;
    char     *thousands_sep;
    unsigned
	char *grouping;
};
struct _LC_resp_t  {
    _LC_core_resp_t core;
    char    *yesexpr;	     
    char    *noexpr;	     
    char    *yesstr;	     
    char    *nostr;	     
};
struct _LC_time_t {
    _LC_core_time_t core;
    char *d_fmt;  
    char *t_fmt;  
    char *d_t_fmt;
    char *t_fmt_ampm;
    char *abday[7];  
    char *day[7];    
    char *abmon[12];  
    char *mon[12];    
    char *am_pm[2];
    char **era;			
    char *era_year;
    char *era_d_fmt;
    char *alt_digits;
    char *m_d_recent;		
    char *m_d_old;		
    char *era_d_t_fmt;
    char *era_t_fmt;
};
typedef union {
    unsigned int n[(sizeof(unsigned int *)/sizeof(unsigned int))];
     unsigned int      
	  *p;	       
} _LC_weight_t;    
typedef struct {
     char         *ce_sym; 
    _LC_weight_t ce_wgt;	
} _LC_collel_t;
typedef struct {
    _LC_weight_t   ct_wgt;    
     _LC_collel_t   *ct_collel;
} _LC_coltbl_t;
typedef struct {
    _LC_weight_t ss_act;	
     char *ss_src;	
     char *ss_tgt;	
} _LC_subs_t;
struct _LC_collate_t {
    _LC_core_collate_t core;
    unsigned			            
	char    co_nord;	            
    _LC_weight_t co_sort;	            
    unsigned int     co_wc_min;		    
    unsigned int     co_wc_max;		    
    unsigned int     co_hbound;		    
    unsigned int     co_col_min;	    
    unsigned int     co_col_max;	    
     _LC_coltbl_t *co_coltbl; 	    
    unsigned
	char    co_nsubs;		    
     _LC_subs_t  *co_subs; 	    
};
typedef struct {
    char    *name;
    unsigned
	int mask;
} _LC_classnm_t;
struct _LC_ctype_t {
  _LC_core_ctype_t core;
  unsigned int      min_wc;
  unsigned int      max_wc;
  unsigned int      max_upper;	
  unsigned int      max_lower;	
   unsigned int      *_upper;	
   unsigned int      *_lower;	
   unsigned
      int      *_mask;       
   unsigned
      int      *qmask;	     
   unsigned
      char     *qidx;	     
  unsigned int	qidx_hbound;	
  unsigned
      char     nclasses;
  _LC_classnm_t *classnms;
};
struct _LC_locale_t {
    _LC_core_locale_t core;
    char           *nl_info[55];    
    struct lconv   *nl_lconv;
    _LC_charmap_t  *lc_charmap;
    _LC_collate_t  *lc_collate;
    _LC_ctype_t    *lc_ctype;
    _LC_monetary_t *lc_monetary;
    _LC_numeric_t  *lc_numeric;
    _LC_resp_t     *lc_resp;
    _LC_time_t     *lc_time;
    char	   *nl_info2[61 - 55];
};
extern _LC_charmap_t  *__lc_charmap;
extern _LC_collate_t  *__lc_collate;
extern _LC_ctype_t    *__lc_ctype;
extern _LC_monetary_t *__lc_monetary;
extern _LC_numeric_t  *__lc_numeric;
extern _LC_resp_t     *__lc_resp;
extern _LC_time_t     *__lc_time;
extern _LC_locale_t   *__lc_locale;
typedef struct {
	int	_cnt;
	unsigned char	*_ptr;
	unsigned char	*_base;
	int	_bufsiz;
	short	_flag;
	short	_file;
	char	*__newbase;
	void	*_lock;			
	unsigned char	*_bufendp;
} FILE;
extern FILE	_iob[];
extern int     fread();
extern int     fwrite();
extern int	_flsbuf ();
extern int	_filbuf ();
extern int 	ferror ();
extern int 	feof ();
extern void 	clearerr ();
extern int 	putchar ();
extern int 	getchar ();
extern int 	putc ();
extern int 	getc ();
extern int	remove ();
extern int	rename ();
extern FILE 	*tmpfile ();
extern char 	*tmpnam ();
extern int 	fclose ();
extern int 	fflush ();
extern FILE	*fopen ();
extern FILE 	*freopen ();
extern void 	setbuf ();
extern int 	setvbuf ();
extern int	fprintf ();
extern int	fscanf ();
extern int	printf ();
extern int	scanf ();
extern int	sprintf ();
extern int	sscanf ();
typedef struct {
	char	**_a0;		
	int	_offset;		
} va_list;
extern int  vfprintf ();
extern int  vprintf ();
extern int  vsprintf ();
extern int 	fgetc ();
extern char 	*fgets ();
extern int 	fputc ();
extern int 	fputs ();
extern char 	*gets ();
extern int 	puts ();
extern int	ungetc ();
extern int	fgetpos ();
extern int 	fseek ();
extern int	fsetpos ();
extern long	ftell ();
extern void	rewind ();
extern void 	perror ();
extern int 	fileno ();
extern FILE 	*fdopen ();
extern char *cuserid ();
extern int getopt ();
extern char *optarg;
extern int optind;
extern int optopt;
extern int opterr;
extern char	*ctermid ();
extern int 	getw ();
extern int 	pclose ();
extern int 	putw ();
extern FILE 	*popen ();
extern char 	*tempnam ();
extern void 	setbuffer ();
extern void 	setlinebuf ();
extern void (*signal())();
extern int raise();
struct sigaction {
	void	(*sa_handler) (); 
	sigset_t sa_mask;		
	int	sa_flags;		
};
typedef union sigval {
	int 	sival_int;
	void	*sival_ptr;
} sigval_t;
typedef struct sigevent {
	union sigval	sigev_value;	
	int		sigev_signo;	
	int		sigev_notify;	
} sigevent_t;
extern int kill (); 
extern int sigaction (); 
extern int sigprocmask ();
extern int sigsuspend ();  
extern int sigemptyset ();
extern int sigfillset ();
extern int sigaddset ();
extern int sigdelset ();
extern int sigismember ();
extern int sigpending ();
typedef int sig_atomic_t;
struct  sigcontext {
	long    sc_onstack;		
	long    sc_mask;		
	long	sc_pc;			
	long	sc_ps;			
	long	sc_regs[32];		
	long	sc_ownedfp;		
	long	sc_fpregs[32];		
	unsigned long sc_fpcr;		
	unsigned long sc_fp_control;	
	long sc_reserved1;		
	long sc_reserved2;		
	size_t	sc_ssize;		
	caddr_t	sc_sbase;		
	unsigned long sc_traparg_a0;	
	unsigned long sc_traparg_a1;	
	unsigned long sc_traparg_a2;	
	unsigned long sc_fp_trap_pc;	
	unsigned long sc_fp_trigger_sum; 
	unsigned long sc_fp_trigger_inst; 
};
extern sigset_t cantmasksigset;
struct	sigvec {
	void	(*sv_handler)();	
	int     sv_mask;        
	int     sv_flags;    
};                           
extern int sigvec();
extern int killpg();
struct  sigstack {
        char    *ss_sp;                 
        int     ss_onstack;             
};
typedef struct  sigaltstack {
        caddr_t	ss_sp;			
        int     ss_flags;		
        size_t	ss_size;		
} stack_t;
extern int sigblock();
extern int sigpause();
extern int sigreturn();
extern int sigsetmask();
extern int sigstack(); 
extern int siginterrupt();
extern int sigaltstack();
extern void (*sigset())();
extern int sighold();
extern int sigrelse();
extern int sigignore();
extern int sigsendset();
extern int sigsend();
  void (*ssignal ()) ();
  int gsignal ();
typedef int             clockid_t;
struct	tm {			
        int     tm_sec;         
        int     tm_min;         
        int     tm_hour;        
        int     tm_mday;        
        int     tm_mon;         
        int     tm_year;        
        int     tm_wday;        
        int     tm_yday;        
        int     tm_isdst;       
	long    __tm_gmtoff;
        char    *__tm_zone;
};
extern clock_t 	clock();
extern double 	difftime();
extern time_t 	mktime();
extern time_t 	time();
extern char 	*asctime();
extern char 	*ctime();
extern struct tm *gmtime();
extern struct tm *localtime();
extern size_t 	strftime();
typedef struct timespec {
        time_t  tv_sec;         
        long    tv_nsec;        
} timespec_t;
struct	itimerspec {
	struct		timespec it_interval; 
	struct		timespec it_value; 
};
int clock_gettime();
int clock_settime();
int clock_getdrift();
int clock_setdrift();
int  timer_create();
int timer_delete();
int timer_gettime();
int timer_settime();
int timer_getoverrun();
int nanosleep();
int clock_getres();
extern char *tzname[];
extern void tzset();
extern long timezone;
extern int daylight;
extern char *strptime ();
extern unsigned char *NLctime(), *NLasctime();
extern char *NLstrtime();
struct timeval {
	int	tv_sec;		
	int	tv_usec;	
};
struct	itimerval {
	struct		timeval it_interval; 
	struct		timeval it_value; 
};
struct timezone {
	int	tz_minuteswest;	
	int	tz_dsttime;	
};
extern int adjtime ();
extern int getitimer ();
extern int setitimer ();
extern int gettimeofday ();
extern int settimeofday ();
extern int utimes ();
typedef struct psx4_timer_struct {
        long		psx4t_idx;      
	long		psx4t_tid;	
	struct itimerval psx4t_timeval;	
	volatile unsigned int	psx4t_flags;	
	unsigned int	psx4t_type:1;   
	unsigned int	psx4t_overrun; 	
	sigval_t     	psx4t_value;	
	int		psx4t_signo;	
	void     	*psx4t_p_proc;	
} psx4_timer_t;
typedef struct psx4_tblock_struct {
	long		psx4tb_free;	
	psx4_timer_t 	psx4_timers[32];
} psx4_tblock_t;
struct	rusage {
	struct timeval ru_utime;	
	struct timeval ru_stime;	
	long	ru_maxrss;
	long	ru_ixrss;		
	long	ru_idrss;		
	long	ru_isrss;		
	long	ru_minflt;		
	long	ru_majflt;		
	long	ru_nswap;		
	long	ru_inblock;		
	long	ru_oublock;		
	long	ru_msgsnd;		
	long	ru_msgrcv;		
	long	ru_nsignals;		
	long	ru_nvcsw;		
	long	ru_nivcsw;		
};
struct rlimit {
	rlim_t	rlim_cur;		
	rlim_t	rlim_max;		
};
struct rusage_dev {
	struct rusage ru_rusage;
	dev_t	      ru_dev;
};
extern int getpriority ();
extern int setpriority ();
extern int getrlimit ();
extern int setrlimit ();
extern int getrusage ();
extern char *index ();
extern char *rindex ();
char *xmalloc ();
void error ();
void warning ();
extern char *getenv ();
extern FILE *fdopen ();
extern char *version_string;
extern struct tm *localtime ();
extern int sys_nerr;
extern char *sys_errlist[];
extern int errno;
struct directive;
struct file_buf;
struct arglist;
struct argdata;
static int do_define ();
static int do_line ();
static int do_include ();
static int do_undef ();
static int do_error ();
static int do_pragma ();
static int do_ident ();
static int do_if ();
static int do_xifdef ();
static int do_else ();
static int do_elif ();
static int do_endif ();
static int do_sccs ();
static int do_once ();
static int do_assert ();
static int do_unassert ();
static int do_warning ();
static void add_import ();
static void append_include_chain ();
static void deps_output ();
static void make_undef ();
static void make_definition ();
static void make_assertion ();
static void path_include ();
static void initialize_builtins ();
static void initialize_char_syntax ();
static void dump_arg_n ();
static void dump_defn_1 ();
static void delete_macro ();
static void trigraph_pcp ();
static void rescan ();
static void finclude ();
static void validate_else ();
static int comp_def_part ();
static void error_from_errno ();
static void error_with_line ();
void pedwarn ();
static void pedwarn_with_file_and_line ();
static void fatal ();
void fancy_abort ();
static void pfatal_with_name ();
static void perror_with_name ();
static void print_containing_files ();
static int lookup_import ();
static int redundant_include_p ();
static is_system_include ();
static int check_preconditions ();
static void pcfinclude ();
static void pcstring_used ();
static void write_output ();
static int check_macro_name ();
static int compare_defs ();
static int compare_token_lists ();
static int eval_if_expression ();
static int discard_comments ();
static int delete_newlines ();
static int line_for_error ();
static int hashf ();
static int file_size_and_mode ();
static struct arglist *read_token_list ();
static void free_token_list ();
static struct hashnode *install ();
struct hashnode *lookup ();
static struct assertion_hashnode *assertion_install ();
static struct assertion_hashnode *assertion_lookup ();
static char *xrealloc ();
static char *xcalloc ();
static char *savestring ();
static void delete_assertion ();
static void macroexpand ();
static void dump_all_macros ();
static void conditional_skip ();
static void skip_if_group ();
static void output_line_command ();
enum file_change_code {same_file, enter_file, leave_file};
static int grow_outbuf ();
static int handle_directive ();
static void memory_full ();
static U_CHAR *macarg1 ();
static char *macarg ();
static U_CHAR *skip_to_end_of_comment ();
static U_CHAR *skip_quoted_string ();
static U_CHAR *skip_paren_group ();
static char *check_precompiled ();
static void dump_single_macro ();
static char *progname;
static int cplusplus;
static int cplusplus_comments;
static int objc;
static int lang_asm;
static int max_include_len;
static int lint = 0;
static int put_out_comments = 0;
static int no_trigraphs = 0;
static int print_deps = 0;
static int print_include_names = 0;
static int no_line_commands;
static enum {dump_none, dump_only, dump_names, dump_definitions}
     dump_macros = dump_none;
static int debug_output = 0;
static FILE *pcp_outfile;
static int pcp_inside_if;
static int no_precomp;
int pedantic;
static int pedantic_errors;
static int inhibit_warnings = 0;
static int warn_comments;
static int warn_stringify;
static int warn_trigraphs;
static int warn_import = 1;
static int warnings_are_errors;
int traditional;
static int no_output;
static int done_initializing = 0;
static struct file_buf {
  char *fname;
  char *nominal_fname;
  struct file_name_list *dir;
  int lineno;
  int length;
  U_CHAR *buf;
  U_CHAR *bufp;
  struct hashnode *macro;
  struct if_stack *if_stack;
  U_CHAR *free_ptr;
  char system_header_p;
} instack[200];
static int last_error_tick;	   
static int input_file_stack_tick;  
static int indepth = -1;
static int system_include_depth = 0;
typedef struct file_buf FILE_BUF;
static FILE_BUF outbuf;
struct file_name_list
  {
    struct file_name_list *next;
    char *fname;
    U_CHAR *control_macro;
  };
static struct default_include { char *fname; int cplusplus; } include_defaults_array[]
  = {
    { "/usr", 1},
    { "/usr/local/bin", 0},
    { "/usr/local/include", 0},
    { "/usr/include", 0},
    { 0, 0}
    };
static struct default_include *include_defaults = include_defaults_array;
static struct file_name_list *include = 0;	
static struct file_name_list *first_bracket_include = 0;
static struct file_name_list *first_system_include = 0;
static struct file_name_list *last_include = 0;	
static struct file_name_list *after_include = 0;
static struct file_name_list *last_after_include = 0;	
static struct file_name_list *dont_repeat_files = 0;
static struct file_name_list *all_include_files = 0;
static char *include_prefix;
static STRINGDEF *stringlist;
static STRINGDEF **stringlist_tailp = &stringlist;
typedef struct macrodef MACRODEF;
struct macrodef
{
  struct definition *defn;
  U_CHAR *symnam;
  int symlen;
};
static struct macrodef create_definition ();
typedef struct definition DEFINITION;
struct definition {
  int nargs;
  int length;			
  int predefined;		
  U_CHAR *expansion;
  int line;			
  char *file;			
  char rest_args;		
  struct reflist {
    struct reflist *next;
    char stringify;		
    char raw_before;		
    char raw_after;		
    char rest_args;		
    int nchars;			
    int argno;			
  } *pattern;
  union {
    U_CHAR *argnames;
  } args;
};
union hashval {
  int ival;
  char *cpval;
  DEFINITION *defn;
  KEYDEF *keydef;
};
static char rest_extension[] = "...";
enum node_type {
 T_DEFINE = 1,	
 T_INCLUDE,	
 T_INCLUDE_NEXT, 
 T_IMPORT,      
 T_IFDEF,	
 T_IFNDEF,	
 T_IF,		
 T_ELSE,	
 T_PRAGMA,	
 T_ELIF,	
 T_UNDEF,	
 T_LINE,	
 T_ERROR,	
 T_WARNING,	
 T_ENDIF,	
 T_SCCS,	
 T_IDENT,	
 T_ASSERT,	
 T_UNASSERT,	
 T_SPECLINE,	
 T_DATE,	
 T_FILE,	
 T_BASE_FILE,	
 T_INCLUDE_LEVEL, 
 T_VERSION,	
 T_SIZE_TYPE,   
 T_PTRDIFF_TYPE,   
 T_WCHAR_TYPE,   
 T_TIME,	
 T_CONST,	
 T_MACRO,	
 T_DISABLED,	
 T_SPEC_DEFINED, 
 T_PCSTRING,	
 T_UNUSED	
 };
struct hashnode {
  struct hashnode *next;	
  struct hashnode *prev;
  struct hashnode **bucket_hdr;	
  enum node_type type;		
  int length;			
  U_CHAR *name;			
  union hashval value;		
};
typedef struct hashnode HASHNODE;
static HASHNODE *hashtab[1403];
static char *predefs = "-Dunix -D__osf__ -D__alpha -D__alpha__ -D_LONGLONG -DSYSTYPE_BSD  -D_SYSTYPE_BSD";
struct tokenlist_list {
  struct tokenlist_list *next;
  struct arglist *tokens;
};
struct assertion_hashnode {
  struct assertion_hashnode *next;	
  struct assertion_hashnode *prev;
  struct assertion_hashnode **bucket_hdr;
  int length;			
  U_CHAR *name;			
  struct tokenlist_list *value;
};
typedef struct assertion_hashnode ASSERTION_HASHNODE;
static ASSERTION_HASHNODE *assertion_hashtab[37];
static int assertions_flag;
struct directive {
  int length;			
  int (*func)();		
  char *name;			
  enum node_type type;		
  char angle_brackets;		
  char traditional_comments;	
  char pass_thru;		
};
static struct directive directive_table[] = {
  {  6, do_define, "define", T_DEFINE, 0, 1},
  {  2, do_if, "if", T_IF},
  {  5, do_xifdef, "ifdef", T_IFDEF},
  {  6, do_xifdef, "ifndef", T_IFNDEF},
  {  5, do_endif, "endif", T_ENDIF},
  {  4, do_else, "else", T_ELSE},
  {  4, do_elif, "elif", T_ELIF},
  {  4, do_line, "line", T_LINE},
  {  7, do_include, "include", T_INCLUDE, 1},
  { 12, do_include, "include_next", T_INCLUDE_NEXT, 1},
  {  6, do_include, "import", T_IMPORT, 1},
  {  5, do_undef, "undef", T_UNDEF},
  {  5, do_error, "error", T_ERROR},
  {  7, do_warning, "warning", T_WARNING},
  {  6, do_pragma, "pragma", T_PRAGMA, 0, 0, 1},
  {  5, do_ident, "ident", T_IDENT, 0, 0, 1},
  {  6, do_assert, "assert", T_ASSERT},
  {  8, do_unassert, "unassert", T_UNASSERT},
  {  -1, 0, "", T_UNUSED},
};
U_CHAR *directive_start;
U_CHAR is_idchar[256];
U_CHAR is_idstart[256];
U_CHAR is_hor_space[256];
static U_CHAR is_space[256];
static int errors = 0;			
static int dollars_in_ident;
static FILE_BUF expand_to_temp_buffer ();
static DEFINITION *collect_expansion ();
struct if_stack {
  struct if_stack *next;	
  char *fname;		
  int lineno;			
  int if_succeeded;		
  U_CHAR *control_macro;	
  enum node_type type;		
};
typedef struct if_stack IF_STACK_FRAME;
static IF_STACK_FRAME *if_stack = 0L;
static char *deps_buffer;
static int deps_allocated_size;
static int deps_size;
static int deps_column;
static int ignore_srcdir;
int
main (argc, argv)
     int argc;
     char **argv;
{
  int st_mode;
  long st_size;
  char *in_fname, *out_fname;
  char *p;
  int f, i;
  FILE_BUF *fp;
  char **pend_files = (char **) xmalloc (argc * sizeof (char *));
  char **pend_defs = (char **) xmalloc (argc * sizeof (char *));
  char **pend_undefs = (char **) xmalloc (argc * sizeof (char *));
  char **pend_assertions = (char **) xmalloc (argc * sizeof (char *));
  char **pend_includes = (char **) xmalloc (argc * sizeof (char *));
  char **pend_assertion_options = (char **) xmalloc (argc * sizeof (char *));
  int inhibit_predefs = 0;
  int no_standard_includes = 0;
  int no_standard_cplusplus_includes = 0;
  int missing_newline = 0;
  int inhibit_output = 0;
  char *deps_file = 0;
  FILE *deps_stream = 0;
  char *deps_target = 0;
  {
    struct rlimit rlim;
    getrlimit (3		, &rlim);
    rlim.rlim_cur = rlim.rlim_max;
    setrlimit (3		, &rlim);
  }
  progname = argv[0];
  in_fname = 0L;
  out_fname = 0L;
  dollars_in_ident = 1;
  initialize_char_syntax ();
  dollars_in_ident = 1 > 0;
  no_line_commands = 0;
  no_trigraphs = 1;
  dump_macros = dump_none;
  no_output = 0;
  cplusplus = 0;
  cplusplus_comments = 0;
  bzero (pend_files, argc * sizeof (char *));
  bzero (pend_defs, argc * sizeof (char *));
  bzero (pend_undefs, argc * sizeof (char *));
  bzero (pend_assertions, argc * sizeof (char *));
  bzero (pend_includes, argc * sizeof (char *));
  for (i = 1; i < argc; i++) {
    if (argv[i][0] != '-') {
      if (out_fname != 0L)
	fatal ("Usage: %s [switches] input output", argv[0]);
      else if (in_fname != 0L)
	out_fname = argv[i];
      else
	in_fname = argv[i];
    } else {
      switch (argv[i][1]) {
      case 'i':
	if (!strcmp (argv[i], "-include")) {
	  if (i + 1 == argc)
	    fatal ("Filename missing after -include option");
	  else
	    pend_includes[i] = argv[i+1], i++;
	}
	if (!strcmp (argv[i], "-imacros")) {
	  if (i + 1 == argc)
	    fatal ("Filename missing after -imacros option");
	  else
	    pend_files[i] = argv[i+1], i++;
	}
	if (!strcmp (argv[i], "-iprefix")) {
	  if (i + 1 == argc)
	    fatal ("Filename missing after -iprefix option");
	  else
	    include_prefix = argv[++i];
	}
	if (!strcmp (argv[i], "-idirafter")) {
	  struct file_name_list *dirtmp;
	  dirtmp = (struct file_name_list *)
	    xmalloc (sizeof (struct file_name_list));
	  dirtmp->next = 0;	
	  dirtmp->control_macro = 0;
	  if (i + 1 == argc)
	    fatal ("Directory name missing after -idirafter option");
	  else
	    dirtmp->fname = argv[++i];
	  if (after_include == 0)
	    after_include = dirtmp;
	  else
	    last_after_include->next = dirtmp;
	  last_after_include = dirtmp; 
	}
	break;
      case 'o':
	if (out_fname != 0L)
	  fatal ("Output filename specified twice");
	if (i + 1 == argc)
	  fatal ("Filename missing after -o option");
	out_fname = argv[++i];
	if (!strcmp (out_fname, "-"))
	  out_fname = "";
	break;
      case 'p':
	if (!strcmp (argv[i], "-pedantic"))
	  pedantic = 1;
	else if (!strcmp (argv[i], "-pedantic-errors")) {
	  pedantic = 1;
	  pedantic_errors = 1;
	} else if (!strcmp (argv[i], "-pcp")) {
	  char *pcp_fname = argv[++i];
	  pcp_outfile = 
	    ((pcp_fname[0] != '-' || pcp_fname[1] != '\0')
	     ? fopen (pcp_fname, "w")
	     : fdopen (dup ((((&_iob[1]))->_file)), "w"));
	  if (pcp_outfile == 0)
	    pfatal_with_name (pcp_fname);
	  no_precomp = 1;
	}
	break;
      case 't':
	if (!strcmp (argv[i], "-traditional")) {
	  traditional = 1;
	  if (dollars_in_ident > 0)
	    dollars_in_ident = 1;
	} else if (!strcmp (argv[i], "-trigraphs")) {
	  no_trigraphs = 0;
	}
	break;
      case 'l':
	if (! strcmp (argv[i], "-lang-c"))
	  cplusplus = 0, cplusplus_comments = 0, objc = 0;
	if (! strcmp (argv[i], "-lang-c++"))
	  cplusplus = 1, cplusplus_comments = 1, objc = 0;
	if (! strcmp (argv[i], "-lang-objc"))
	  objc = 1, cplusplus = 0, cplusplus_comments = 1;
	if (! strcmp (argv[i], "-lang-objc++"))
	  objc = 1, cplusplus = 1, cplusplus_comments = 1;
 	if (! strcmp (argv[i], "-lang-asm"))
 	  lang_asm = 1;
 	if (! strcmp (argv[i], "-lint"))
 	  lint = 1;
	break;
      case '+':
	cplusplus = 1, cplusplus_comments = 1;
	break;
      case 'w':
	inhibit_warnings = 1;
	break;
      case 'W':
	if (!strcmp (argv[i], "-Wtrigraphs"))
	  warn_trigraphs = 1;
	else if (!strcmp (argv[i], "-Wno-trigraphs"))
	  warn_trigraphs = 0;
	else if (!strcmp (argv[i], "-Wcomment"))
	  warn_comments = 1;
	else if (!strcmp (argv[i], "-Wno-comment"))
	  warn_comments = 0;
	else if (!strcmp (argv[i], "-Wcomments"))
	  warn_comments = 1;
	else if (!strcmp (argv[i], "-Wno-comments"))
	  warn_comments = 0;
	else if (!strcmp (argv[i], "-Wtraditional"))
	  warn_stringify = 1;
	else if (!strcmp (argv[i], "-Wno-traditional"))
	  warn_stringify = 0;
	else if (!strcmp (argv[i], "-Wimport"))
	  warn_import = 1;
	else if (!strcmp (argv[i], "-Wno-import"))
	  warn_import = 0;
	else if (!strcmp (argv[i], "-Werror"))
	  warnings_are_errors = 1;
	else if (!strcmp (argv[i], "-Wno-error"))
	  warnings_are_errors = 0;
	else if (!strcmp (argv[i], "-Wall"))
	  {
	    warn_trigraphs = 1;
	    warn_comments = 1;
	  }
	break;
      case 'M':
	if (!strcmp (argv[i], "-M"))
	  print_deps = 2;
	else if (!strcmp (argv[i], "-MM"))
	  print_deps = 1;
	else if (!strcmp (argv[i], "-MD"))
	  print_deps = 2;
	else if (!strcmp (argv[i], "-MMD"))
	  print_deps = 1;
	if (!strcmp (argv[i], "-MD")
	    || !strcmp (argv[i], "-MMD")) {
	  i++;
	  deps_file = argv[i];
	} else {
	  deps_stream = (&_iob[1]);
	  inhibit_output = 1;
	}	  
	break;
      case 'd':
	{
	  char *p = argv[i] + 2;
	  char c;
	  while (c = *p++) {
	    switch (c) {
	    case 'M':
	      dump_macros = dump_only;
	      no_output = 1;
	      break;
	    case 'N':
	      dump_macros = dump_names;
	      break;
	    case 'D':
	      dump_macros = dump_definitions;
	      break;
	    }
	  }
	}
	break;
      case 'g':
	if (argv[i][2] == '3')
	  debug_output = 1;
	break;
      case 'v':
	fprintf ((&_iob[2]), "GNU CPP version %s", version_string);
	;
	fprintf ((&_iob[2]), "\n");
	break;
      case 'H':
	print_include_names = 1;
	break;
      case 'D':
	{
	  char *p, *p1;
	  if (argv[i][2] != 0)
	    p = argv[i] + 2;
	  else if (i + 1 == argc)
	    fatal ("Macro name missing after -D option");
	  else
	    p = argv[++i];
	  pend_defs[i] = p;
	}
	break;
      case 'A':
	{
	  char *p, *p1;
	  if (argv[i][2] != 0)
	    p = argv[i] + 2;
	  else if (i + 1 == argc)
	    fatal ("Assertion missing after -A option");
	  else
	    p = argv[++i];
	  if (!strcmp (p, "-")) {
	    int j;
	    inhibit_predefs = 1;
	    for (j = 0; j < i; j++)
	      pend_defs[j] = pend_assertions[j] = 0;
	  } else {
	    pend_assertions[i] = p;
	    pend_assertion_options[i] = "-A";
	  }
	}
	break;
      case 'U':		
	if (argv[i][2] != 0)
	  pend_undefs[i] = argv[i] + 2;
	else if (i + 1 == argc)
	  fatal ("Macro name missing after -U option");
	else
	  pend_undefs[i] = argv[i+1], i++;
	break;
      case 'C':
	put_out_comments = 1;
	break;
      case 'E':			
	break;
      case 'P':
	no_line_commands = 1;
	break;
      case '$':			
	dollars_in_ident = 0;
	break;
      case 'I':			
	{
	  struct file_name_list *dirtmp;
	  if (! ignore_srcdir && !strcmp (argv[i] + 2, "-")) {
	    ignore_srcdir = 1;
	    first_bracket_include = 0;
	  }
	  else {
	    dirtmp = (struct file_name_list *)
	      xmalloc (sizeof (struct file_name_list));
	    dirtmp->next = 0;		
	    dirtmp->control_macro = 0;
	    if (argv[i][2] != 0)
	      dirtmp->fname = argv[i] + 2;
	    else if (i + 1 == argc)
	      fatal ("Directory name missing after -I option");
	    else
	      dirtmp->fname = argv[++i];
	    append_include_chain (dirtmp, dirtmp);
	  }
	}
	break;
      case 'n':
	if (!strcmp (argv[i], "-nostdinc"))
	  no_standard_includes = 1;
	else if (!strcmp (argv[i], "-nostdinc++"))
	  no_standard_cplusplus_includes = 1;
	else if (!strcmp (argv[i], "-noprecomp"))
	  no_precomp = 1;
	break;
      case 'u':
	inhibit_predefs = 1;
	break;
      case '\0': 
	if (in_fname == 0L) {
	  in_fname = "";
	  break;
	} else if (out_fname == 0L) {
	  out_fname = "";
	  break;
	}	
      default:
	fatal ("Invalid option `%s'", argv[i]);
      }
    }
  }
  p = (char *) getenv ("CPATH");
  if (p != 0 && ! no_standard_includes)
    path_include (p);
  initialize_char_syntax ();
  outbuf.buf = (U_CHAR *) xmalloc (10	);
  outbuf.bufp = outbuf.buf;
  outbuf.length = 10	;
  fp = &instack[++indepth];
  if (in_fname == 0L)
    in_fname = "";
  fp->nominal_fname = fp->fname = in_fname;
  fp->lineno = 0;
  initialize_builtins (fp, &outbuf);
  if (!inhibit_predefs) {
    char *p = (char *) alloca (strlen (predefs) + 1);
    strcpy (p, predefs);
    while (*p) {
      char *q;
      while (*p == ' ' || *p == '\t')
	p++;
      if (p[0] == '-' && p[1] == 'D') {
	q = &p[2];
	while (*p && *p != ' ' && *p != '\t')
	  p++;
	if (*p != 0)
	  *p++= 0;
	if (debug_output)
	  output_line_command (fp, &outbuf, 0, same_file);
	make_definition (q, &outbuf);
	while (*p == ' ' || *p == '\t')
	  p++;
      } else if (p[0] == '-' && p[1] == 'A') {
	char *assertion;
	char *past_name;
	char *value;
	char *past_value;
	char *termination;
	int save_char;
	assertion = &p[2];
	past_name = assertion;
	while (*past_name && *past_name != ' '
	       && *past_name != '\t' && *past_name != '(')
	  past_name++;
	value = past_name;
	while (*value && (*value == ' ' || *value == '\t'))
	  value++;
	if (*value++ != '(')
	  abort ();
	while (*value && (*value == ' ' || *value == '\t'))
	  value++;
	past_value = value;
	while (*past_value && *past_value != ' '
	       && *past_value != '\t' && *past_value != ')')
	  past_value++;
	termination = past_value;
	while (*termination && (*termination == ' ' || *termination == '\t'))
	  termination++;
	if (*termination++ != ')')
	  abort ();
	if (*termination && *termination != ' ' && *termination != '\t')
	  abort ();
	save_char = *termination;
	*termination = '\0';
	make_assertion ("-A", assertion);
	*termination = (char) save_char;
	p = termination;
	while (*p == ' ' || *p == '\t')
	  p++;
      } else {
	abort ();
      }
    }
  }
  for (i = 1; i < argc; i++) {
    if (pend_undefs[i]) {
      if (debug_output)
        output_line_command (fp, &outbuf, 0, same_file);
      make_undef (pend_undefs[i], &outbuf);
    }
    if (pend_defs[i]) {
      if (debug_output)
        output_line_command (fp, &outbuf, 0, same_file);
      make_definition (pend_defs[i], &outbuf);
    }
    if (pend_assertions[i])
      make_assertion (pend_assertion_options[i], pend_assertions[i]);
  }
  done_initializing = 1;
  { 
    char *epath = 0;
    switch ((objc << 1) + cplusplus)
      {
      case 0:
	epath = getenv ("C_INCLUDE_PATH");
	break;
      case 1:
	epath = getenv ("CPLUS_INCLUDE_PATH");
	break;
      case 2:
	epath = getenv ("OBJC_INCLUDE_PATH");
	break;
      case 3:
	epath = getenv ("OBJCPLUS_INCLUDE_PATH");
	break;
      }
    if (epath) {
      char *nstore = (char *) alloca (strlen (epath) + 2);
      int num_dirs;
      char *startp, *endp;
      for (num_dirs = 1, startp = epath; *startp; startp++)
	if (*startp == ':')
	  num_dirs++;
      include_defaults
	= (struct default_include *) xmalloc ((num_dirs
					       * sizeof (struct default_include))
					      + sizeof (include_defaults_array));
      startp = endp = epath;
      num_dirs = 0;
      while (1) {
        if ((*endp == ':'
	     )
            || *endp == 0) {
	  strncpy (nstore, startp, endp-startp);
	  if (endp == startp)
	    strcpy (nstore, ".");
	  else
	    nstore[endp-startp] = '\0';
	  include_defaults[num_dirs].fname = savestring (nstore);
	  include_defaults[num_dirs].cplusplus = cplusplus;
	  num_dirs++;
	  if (*endp == '\0')
	    break;
	  endp = startp = endp + 1;
	} else
	  endp++;
      }
      bcopy (include_defaults_array, &include_defaults[num_dirs],
	     sizeof (include_defaults_array));
    }
  }
  first_system_include = 0;
  if (!no_standard_includes) {
    struct default_include *p = include_defaults;
    char *specd_prefix = include_prefix;
    char *default_prefix = savestring ("/usr/local/bin");
    int default_len = 0;
    if (!strcmp (default_prefix + strlen (default_prefix) - 8, "/include")) {
      default_len = strlen (default_prefix) - 7;
      default_prefix[default_len] = 0;
    }
    if (specd_prefix != 0 && default_len != 0)
      for (p = include_defaults; p->fname; p++) {
	if (!p->cplusplus || (cplusplus && !no_standard_cplusplus_includes)) {
	  if (!strncmp (p->fname, default_prefix, default_len)) {
	    struct file_name_list *new
	      = (struct file_name_list *) xmalloc (sizeof (struct file_name_list));
	    int this_len = strlen (specd_prefix) + strlen (p->fname) - default_len;
	    char *str = (char *) xmalloc (this_len + 1);
	    strcpy (str, specd_prefix);
	    strcat (str, p->fname + default_len);
	    new->fname = str;
	    new->control_macro = 0;
	    append_include_chain (new, new);
	    if (first_system_include == 0)
	      first_system_include = new;
	  }
	}
      }
    for (p = include_defaults; p->fname; p++) {
      if (!p->cplusplus || (cplusplus && !no_standard_cplusplus_includes)) {
	struct file_name_list *new
	  = (struct file_name_list *) xmalloc (sizeof (struct file_name_list));
	new->control_macro = 0;
	new->fname = p->fname;
	append_include_chain (new, new);
	if (first_system_include == 0)
	  first_system_include = new;
      }
    }
  }
  append_include_chain (after_include, last_after_include);
  if (first_system_include == 0)
    first_system_include = after_include;
  no_output++;
  for (i = 1; i < argc; i++)
    if (pend_files[i]) {
      int fd = open (pend_files[i], 0, 0666);
      if (fd < 0) {
	perror_with_name (pend_files[i]);
	return 33	;
      }
      finclude (fd, pend_files[i], &outbuf, 0, ((char *)0));
    }
  no_output--;
  if (in_fname == 0L || *in_fname == 0) {
    in_fname = "";
    f = 0;
  } else if ((f = open (in_fname, 0, 0666)) < 0)
    goto perror;
  if (print_deps == 0
      && (getenv ("SUNPRO_DEPENDENCIES") != 0
	  || getenv ("DEPENDENCIES_OUTPUT") != 0)) {
    char *spec = getenv ("DEPENDENCIES_OUTPUT");
    char *s;
    char *output_file;
    if (spec == 0) {
      spec = getenv ("SUNPRO_DEPENDENCIES");
      print_deps = 2;
    }
    else
      print_deps = 1;
    s = spec;
    while (*s != 0 && *s != ' ') s++;
    if (*s != 0) {
      deps_target = s + 1;
      output_file = (char *) xmalloc (s - spec + 1);
      bcopy (spec, output_file, s - spec);
      output_file[s - spec] = 0;
    }
    else {
      deps_target = 0;
      output_file = spec;
    }
    deps_file = output_file;
  }
  if (print_deps) {
    deps_allocated_size = 200;
    deps_buffer = (char *) xmalloc (deps_allocated_size);
    deps_buffer[0] = 0;
    deps_size = 0;
    deps_column = 0;
    if (deps_target) {
      deps_output (deps_target, 0);
      deps_output (":", 0);
    } else if (*in_fname == 0)
      deps_output ("-: ", 0);
    else {
      int len;
      char *p = in_fname;
      char *p1 = p;
      while (*p1) {
	if (*p1 == '/')
	  p = p1 + 1;
	p1++;
      }
      len = strlen (p);
      if (p[len - 2] == '.' && p[len - 1] == 'c')
	deps_output (p, len - 2);
      else if (p[len - 2] == '.' && p[len - 1] == 'C')
	deps_output (p, len - 2);
      else if (p[len - 3] == '.'
	       && p[len - 2] == 'c'
	       && p[len - 1] == 'c')
	deps_output (p, len - 3);
      else if (p[len - 2] == '.' && p[len - 1] == 's')
	deps_output (p, len - 2);
      else if (p[len - 2] == '.' && p[len - 1] == 'S')
	deps_output (p, len - 2);
      else if (p[len - 2] == '.' && p[len - 1] == 'm')
	deps_output (p, len - 2);
      else
	deps_output (p, 0);
      deps_output (".o : ", 0);
      deps_output (in_fname, 0);
      deps_output (" ", 0);
    }
  }
  file_size_and_mode (f, &st_mode, &st_size);
  fp->nominal_fname = fp->fname = in_fname;
  fp->lineno = 1;
  fp->system_header_p = 0;
  if (! (((st_mode)&(0170000)) == (0100000))) {
    int size;
    int bsize;
    int cnt;
    U_CHAR *bufp;
    bsize = 2000;
    size = 0;
    fp->buf = (U_CHAR *) xmalloc (bsize + 2);
    bufp = fp->buf;
    for (;;) {
      cnt = read (f, bufp, bsize - size);
      if (cnt < 0) goto perror;	
      if (cnt == 0) break;	
      size += cnt;
      bufp += cnt;
      if (bsize == size) {	
        bsize *= 2;
        fp->buf = (U_CHAR *) xrealloc (fp->buf, bsize + 2);
	bufp = fp->buf + size;	
      }
    }
    fp->length = size;
  } else {
    long i;
    fp->length = 0;
    fp->buf = (U_CHAR *) xmalloc (st_size + 2);
    while (st_size > 0) {
      i = read (f, fp->buf + fp->length, st_size);
      if (i <= 0) {
        if (i == 0) break;
	goto perror;
      }
      fp->length += i;
      st_size -= i;
    }
  }
  fp->bufp = fp->buf;
  fp->if_stack = if_stack;
  if ((fp->length > 0 && fp->buf[fp->length - 1] != '\n')
      || (fp->length > 1 && fp->buf[fp->length - 2] == '\\')) {
    fp->buf[fp->length++] = '\n';
    missing_newline = 1;
  }
  fp->buf[fp->length] = '\0';
  if (!no_trigraphs)
    trigraph_pcp (fp);
  if (!out_fname || !strcmp (out_fname, ""))
    out_fname = "stdout";
  else if (! freopen (out_fname, "w", (&_iob[1])))
    pfatal_with_name (out_fname);
  output_line_command (fp, &outbuf, 0, same_file);
  for (i = 1; i < argc; i++)
    if (pend_includes[i]) {
      int fd = open (pend_includes[i], 0, 0666);
      if (fd < 0) {
	perror_with_name (pend_includes[i]);
	return 33	;
      }
      finclude (fd, pend_includes[i], &outbuf, 0, ((char *)0));
    }
  rescan (&outbuf, 0);
  if (pedantic && missing_newline)
    pedwarn ("file does not end in newline");
  if (dump_macros == dump_only)
    dump_all_macros ();
  else if (! inhibit_output) {
    write_output ();
  }
  if (print_deps) {
    if (errors == 0) {
      if (deps_file && ! (deps_stream = fopen (deps_file, "a")))
	pfatal_with_name (deps_file);
      fputs (deps_buffer, deps_stream);
      		(--(  deps_stream)->_cnt < 0 ? 			_flsbuf((int) ('\n'), (  deps_stream)) : 			(int) (*(  deps_stream)->_ptr++ = (unsigned char) ('\n')));
      if (deps_file) {
	if (((deps_stream)->_flag & 0040) || fclose (deps_stream) != 0)
	  fatal ("I/O error on output");
      }
    }
  }
  if ((((&_iob[1]))->_flag & 0040) || fclose ((&_iob[1])) != 0)
    fatal ("I/O error on output");
  if (errors)
    exit (33	);
  exit (0	);
 perror:
  pfatal_with_name (in_fname);
  return 0;
}
static void
path_include (path)
     char *path;
{
  char *p;
  p = path;
  if (*p)
    while (1) {
      char *q = p;
      char *name;
      struct file_name_list *dirtmp;
      while (*q != 0 && *q != ':') q++;
      if (p == q) {
	name = (char *) xmalloc (2);
	name[0] = '.';
	name[1] = 0;
      } else {
	name = (char *) xmalloc (q - p + 1);
	bcopy (p, name, q - p);
	name[q - p] = 0;
      }
      dirtmp = (struct file_name_list *)
	xmalloc (sizeof (struct file_name_list));
      dirtmp->next = 0;		
      dirtmp->control_macro = 0;
      dirtmp->fname = name;
      append_include_chain (dirtmp, dirtmp);
      p = q;
      if (*p == 0)
	break;
      p++;
    }
}
static void
trigraph_pcp (buf)
     FILE_BUF *buf;
{
  register U_CHAR c, *fptr, *bptr, *sptr;
  int len;
  fptr = bptr = sptr = buf->buf;
  while ((sptr = (U_CHAR *) index (sptr, '?')) != 0L) {
    if (*++sptr != '?')
      continue;
    switch (*++sptr) {
      case '=':
      c = '#';
      break;
    case '(':
      c = '[';
      break;
    case '/':
      c = '\\';
      break;
    case ')':
      c = ']';
      break;
    case '\'':
      c = '^';
      break;
    case '<':
      c = '{';
      break;
    case '!':
      c = '|';
      break;
    case '>':
      c = '}';
      break;
    case '-':
      c  = '~';
      break;
    case '?':
      sptr--;
      continue;
    default:
      continue;
    }
    len = sptr - fptr - 2;
    if (bptr != fptr && len > 0)
      bcopy (fptr, bptr, len);	
    bptr += len;
    *bptr++ = c;
    fptr = ++sptr;
  }
  len = buf->length - (fptr - buf->buf);
  if (bptr != fptr && len > 0)
    bcopy (fptr, bptr, len);
  buf->length -= fptr - bptr;
  buf->buf[buf->length] = '\0';
  if (warn_trigraphs && fptr != bptr)
    warning ("%d trigraph(s) encountered", (fptr - bptr) / 2);
}
static void
newline_fix (bp)
     U_CHAR *bp;
{
  register U_CHAR *p = bp;
  register int count = 0;
  while (1) {
    if (p[0] == '\\') {
      if (p[1] == '\n')
	p += 2, count++;
      else if (p[1] == '\r' && p[2] == '\n')
	p += 3, count++;
      else
	break;
    } else
      break;
  }
  if (count == 0 || (*p != '/' && *p != '*'))
    return;
  while (*p == '*' || *p == '/')
    *bp++ = *p++;
  while (count-- > 0) {
    *bp++ = '\\';
    *bp++ = '\n';
  }
}
static void
name_newline_fix (bp)
     U_CHAR *bp;
{
  register U_CHAR *p = bp;
  register int count = 0;
  while (1) {
    if (p[0] == '\\') {
      if (p[1] == '\n')
	p += 2, count++;
      else if (p[1] == '\r' && p[2] == '\n')
	p += 3, count++;
      else
	break;
    } else
      break;
  }
  if (count == 0 || !is_idchar[*p])
    return;
  while (is_idchar[*p])
    *bp++ = *p++;
  while (count-- > 0) {
    *bp++ = '\\';
    *bp++ = '\n';
  }
}
static char *
get_lintcmd (ibp, limit, argstart, arglen, cmdlen)
     register U_CHAR *ibp;
     register U_CHAR *limit;
     U_CHAR **argstart;		
     int *arglen, *cmdlen;	
{
  long linsize;
  register U_CHAR *numptr;	
  *arglen = 0;
  do { while (is_hor_space[*ibp]) ibp++; } while (0);
  if (ibp >= limit) return 0L;
  linsize = limit - ibp;
  if ((linsize >= 10) && !strncmp (ibp, "NOTREACHED", 10)) {
    *cmdlen = 10;
    return "NOTREACHED";
  }
  if ((linsize >= 8) && !strncmp (ibp, "ARGSUSED", 8)) {
    *cmdlen = 8;
    return "ARGSUSED";
  }
  if ((linsize >= 11) && !strncmp (ibp, "LINTLIBRARY", 11)) {
    *cmdlen = 11;
    return "LINTLIBRARY";
  }
  if ((linsize >= 7) && !strncmp (ibp, "VARARGS", 7)) {
    *cmdlen = 7;
    ibp += 7; linsize -= 7;
    if ((linsize == 0) || ! (((*(__lc_ctype->core.iswctype)) == 0L) 		? (int) (__lc_ctype->_mask[*ibp] & (0x010)) 		: (*(__lc_ctype->core.iswctype)) (*ibp,0x010,__lc_ctype))) return "VARARGS";
    for (numptr = *argstart = ibp; (numptr < limit) && (((*(__lc_ctype->core.iswctype)) == 0L) 		? (int) (__lc_ctype->_mask[*numptr] & (0x010)) 		: (*(__lc_ctype->core.iswctype)) (*numptr,0x010,__lc_ctype));
	 numptr++);
    *arglen = numptr - *argstart;
    return "VARARGS";
  }
  return 0L;
}
static void
rescan (op, output_marks)
     FILE_BUF *op;
     int output_marks;
{
  register U_CHAR c;
  register int ident_length = 0;
  register int hash = 0;
  FILE_BUF *ip;
  register U_CHAR *ibp;
  register U_CHAR *limit;
  register U_CHAR *obp;
  int redo_char = 0;
  int concatenated = 0;
  int start_line;
  int multiline_string_line = 0;
  U_CHAR *beg_of_line;
  if (no_output && instack[indepth].fname != 0)
    skip_if_group (&instack[indepth], 1);
  obp = op->bufp;
  do { ip = &instack[indepth];		     ibp = ip->bufp;			     limit = ip->buf + ip->length;	     op->bufp = obp;			       (((op)->length - ((op)->bufp - (op)->buf) <= ( limit - ibp))      ? grow_outbuf ((op), ( limit - ibp)) : 0);	     beg_of_line = 0;			     obp = op->bufp; } while (0);
  beg_of_line = ibp;
  if (*limit != 0)
    abort ();
  while (1) {
    c = *ibp++;
    *obp++ = c;
    switch (c) {
    case '\\':
      if (ibp >= limit)
	break;
      if (*ibp == '\n') {
	++ibp;
	++ip->lineno;
	--obp;		
	break;
      }
      if (ident_length > 0)
	goto specialchar;
      *obp++ = *ibp++;
      break;
    case '#':
      if (assertions_flag) {
	do { while (is_hor_space[*ibp]) ibp++; } while (0);
	while (is_idchar[*ibp])
	  *obp++ = *ibp++;
	do { while (is_hor_space[*ibp]) ibp++; } while (0);
	if (*ibp == '(') {
	  ip->bufp = ibp;
	  skip_paren_group (ip);
	  bcopy (ibp, obp, ip->bufp - ibp);
	  obp += ip->bufp - ibp;
	  ibp = ip->bufp;
	}
      }
      if (ip->macro != 0)
	goto randomchar;
      if (ip->fname == 0 && beg_of_line == ip->buf)
	goto randomchar;
      if (ident_length)
	goto specialchar;
      if (beg_of_line == 0)
	goto randomchar;
      {
	U_CHAR *bp;
	bp = beg_of_line;
	if (!traditional)
	  while (1) {
	    if (is_hor_space[*bp])
	      bp++;
	    else if (*bp == '\\' && bp[1] == '\n')
	      bp += 2;
	    else if (*bp == '/' && bp[1] == '*') {
	      bp += 2;
	      while (!(*bp == '*' && bp[1] == '/'))
		bp++;
	      bp += 2;
	    }
	    else if (cplusplus_comments && *bp == '/' && bp[1] == '/') {
	      bp += 2;
	      while (*bp++ != '\n') ;
	    }
	    else break;
	  }
	if (bp + 1 != ibp)
	  goto randomchar;
      }
      --obp;		
      ip->bufp = ibp;
      op->bufp = obp;
      if (! handle_directive (ip, op)) {
	if (no_output && instack[indepth].fname) {
	  skip_if_group (&instack[indepth], 1);
	  do { ip = &instack[indepth];		     ibp = ip->bufp;			     limit = ip->buf + ip->length;	     op->bufp = obp;			       (((op)->length - ((op)->bufp - (op)->buf) <= ( limit - ibp))      ? grow_outbuf ((op), ( limit - ibp)) : 0);	     beg_of_line = 0;			     obp = op->bufp; } while (0);
	  beg_of_line = ibp;
	  break;
	}
	++obp;		
	goto randomchar;
      }
      if (no_output && instack[indepth].fname)
	skip_if_group (&instack[indepth], 1);
      obp = op->bufp;
      do { ip = &instack[indepth];		     ibp = ip->bufp;			     limit = ip->buf + ip->length;	     op->bufp = obp;			       (((op)->length - ((op)->bufp - (op)->buf) <= ( limit - ibp))      ? grow_outbuf ((op), ( limit - ibp)) : 0);	     beg_of_line = 0;			     obp = op->bufp; } while (0);
      beg_of_line = ibp;
      break;
    case '\"':			
    case '\'':
      if (ident_length)
	goto specialchar;
      start_line = ip->lineno;
      while (1) {
	if (ibp >= limit) {
	  if (ip->macro != 0) {
	    do { ip->macro->type = T_MACRO;		     if (ip->free_ptr) free (ip->free_ptr);	     --indepth; } while (0);
	    do { ip = &instack[indepth];		     ibp = ip->bufp;			     limit = ip->buf + ip->length;	     op->bufp = obp;			       (((op)->length - ((op)->bufp - (op)->buf) <= ( limit - ibp))      ? grow_outbuf ((op), ( limit - ibp)) : 0);	     beg_of_line = 0;			     obp = op->bufp; } while (0);
	    continue;
	  }
	  if (!traditional) {
	    error_with_line (line_for_error (start_line),
			     "unterminated string or character constant");
	    error_with_line (multiline_string_line,
			     "possible real start of unterminated constant");
	    multiline_string_line = 0;
	  }
	  break;
	}
	*obp++ = *ibp;
	switch (*ibp++) {
	case '\n':
	  ++ip->lineno;
	  ++op->lineno;
	  if (traditional) {
	    beg_of_line = ibp;
	    goto while2end;
	  }
	  if (pedantic || c == '\'') {
	    error_with_line (line_for_error (start_line),
			     "unterminated string or character constant");
	    goto while2end;
	  }
	  if (multiline_string_line == 0)
	    multiline_string_line = ip->lineno - 1;
	  break;
	case '\\':
	  if (ibp >= limit)
	    break;
	  if (*ibp == '\n') {
	    --obp;
	    ++ibp;
	    ++ip->lineno;
	  } else {
	    while (*ibp == '\\' && ibp[1] == '\n') {
	      ibp += 2;
	      ++ip->lineno;
	    }
	    *obp++ = *ibp++;
	  }
	  break;
	case '\"':
	case '\'':
	  if (ibp[-1] == c)
	    goto while2end;
	  break;
	}
      }
    while2end:
      break;
    case '/':
      if (*ibp == '\\' && ibp[1] == '\n')
	newline_fix (ibp);
      if (*ibp != '*'
	  && !(cplusplus_comments && *ibp == '/'))
	goto randomchar;
      if (ip->macro != 0)
	goto randomchar;
      if (ident_length)
	goto specialchar;
      if (*ibp == '/') {
	start_line = ip->lineno;
	--ibp;			
	--obp;
	if (! put_out_comments)
	  *obp++ = ' ';
	else {
	  *obp++ = '/';
	  *obp++ = '/';
	}
	{
	  U_CHAR *before_bp = ibp+2;
	  while (ibp < limit) {
	    if (*ibp++ == '\n') {
	      ibp--;
	      if (put_out_comments) {
		bcopy (before_bp, obp, ibp - before_bp);
		obp += ibp - before_bp;
	      }
	      break;
	    }
	  }
	  break;
	}
      }
      start_line = ip->lineno;
      ++ibp;			
      if (lint) {
	U_CHAR *argbp;
	int cmdlen, arglen;
	char *lintcmd = get_lintcmd (ibp, limit, &argbp, &arglen, &cmdlen);
	if (lintcmd != 0L) {
	  obp[-1] = '\n';
	  bcopy ("#pragma lint ", obp, 13);
	  obp += 13;
	  bcopy (lintcmd, obp, cmdlen);
	  obp += cmdlen;
	  if (arglen != 0) {
	    *(obp++) = ' ';
	    bcopy (argbp, obp, arglen);
	    obp += arglen;
	  }
	  output_line_command (ip, op, 0, same_file);
	  *(obp++) = ' ';	
	  *(obp++) = '/';
	}
      }
      if (! put_out_comments) {
	if (traditional)
	  obp--;
	else
	  obp[-1] = ' ';
      }
      else
	*obp++ = '*';
      {
	U_CHAR *before_bp = ibp;
	while (ibp < limit) {
	  switch (*ibp++) {
	  case '/':
	    if (warn_comments && ibp < limit && *ibp == '*')
	      warning("`/*' within comment");
	    break;
	  case '*':
	    if (*ibp == '\\' && ibp[1] == '\n')
	      newline_fix (ibp);
	    if (ibp >= limit || *ibp == '/')
	      goto comment_end;
	    break;
	  case '\n':
	    ++ip->lineno;
	    if (!put_out_comments)
	      *obp++ = '\n';
	    ++op->lineno;
	  }
	}
      comment_end:
	if (ibp >= limit)
	  error_with_line (line_for_error (start_line),
			   "unterminated comment");
	else {
	  ibp++;
	  if (put_out_comments) {
	    bcopy (before_bp, obp, ibp - before_bp);
	    obp += ibp - before_bp;
	  }
	}
      }
      break;
    case '$':
      if (!dollars_in_ident)
	goto randomchar;
      goto letter;
    case '0': case '1': case '2': case '3': case '4':
    case '5': case '6': case '7': case '8': case '9':
      if (ident_length == 0) {
	while (ibp < limit) {
	  while (ibp < limit && ibp[0] == '\\' && ibp[1] == '\n') {
	    ++ip->lineno;
	    ibp += 2;
	  }
	  c = *ibp++;
	  if (!(((*(__lc_ctype->core.iswctype)) == 0L) 		? (int) (__lc_ctype->_mask[c] & (0x002)) 		: (*(__lc_ctype->core.iswctype)) (c,0x002,__lc_ctype)) && (c != '.' || *ibp == '.') && c != '_') {
	    --ibp;
	    break;
	  }
	  *obp++ = c;
	  if (c == 'e' || c == 'E') {
	    while (ibp < limit && ibp[0] == '\\' && ibp[1] == '\n') {
	      ++ip->lineno;
	      ibp += 2;
	    }
	    if (ibp < limit && (*ibp == '+' || *ibp == '-')) {
	      *obp++ = *ibp++;
	      if (traditional)
		break;
	    }
	  }
	}
	break;
      }
    case '_':
    case 'a': case 'b': case 'c': case 'd': case 'e': case 'f':
    case 'g': case 'h': case 'i': case 'j': case 'k': case 'l':
    case 'm': case 'n': case 'o': case 'p': case 'q': case 'r':
    case 's': case 't': case 'u': case 'v': case 'w': case 'x':
    case 'y': case 'z':
    case 'A': case 'B': case 'C': case 'D': case 'E': case 'F':
    case 'G': case 'H': case 'I': case 'J': case 'K': case 'L':
    case 'M': case 'N': case 'O': case 'P': case 'Q': case 'R':
    case 'S': case 'T': case 'U': case 'V': case 'W': case 'X':
    case 'Y': case 'Z':
    letter:
      ident_length++;
      hash = ((hash << 2) +  c);
      break;
    case '\n':
      if (ip->macro != 0) {
	if (*ibp == '-') {
	  if (! concatenated) {
	    ident_length = 0;
	    hash = 0;
	  }
	  ibp++;
	  if (!output_marks) {
	    obp--;
	  } else {
	    *obp++ = '-';
	  }
	} else if (is_space[*ibp]) {
	  if (ident_length > 0)
	    goto specialchar;
	  if (!output_marks) {
	    obp[-1] = *ibp++;
	    if (obp[-1] == '\n')
	      op->lineno++;
	  } else {
	    *obp++ = *ibp++;
	  }
	} else abort ();	
	break;
      }
      if (ident_length > 0)
	goto specialchar;
      beg_of_line = ibp;
      ++ip->lineno;
      ++op->lineno;
      if (ip->lineno != op->lineno) {
	op->bufp = obp;
	output_line_command (ip, op, 1, same_file);
	  (((op)->length - ((op)->bufp - (op)->buf) <= ( ip->length - (ip->bufp - ip->buf)))      ? grow_outbuf ((op), ( ip->length - (ip->bufp - ip->buf))) : 0);
	obp = op->bufp;
      }
      break;
    case 0:
      if (ibp <= limit)
	goto randomchar;
      if (ip->macro != 0) {
	obp--;
	ibp--;
	if (traditional && ident_length
	    && ! is_idchar[*instack[indepth - 1].bufp]) {
	  redo_char = 1;
	  goto randomchar;
	}
	do { ip->macro->type = T_MACRO;		     if (ip->free_ptr) free (ip->free_ptr);	     --indepth; } while (0);
	do { ip = &instack[indepth];		     ibp = ip->bufp;			     limit = ip->buf + ip->length;	     op->bufp = obp;			       (((op)->length - ((op)->bufp - (op)->buf) <= ( limit - ibp))      ? grow_outbuf ((op), ( limit - ibp)) : 0);	     beg_of_line = 0;			     obp = op->bufp; } while (0);
	break;
      }
      if (ident_length == 0) {
	obp--;
	ibp--;
	op->bufp = obp;
	ip->bufp = ibp;
	goto ending;
      }
specialchar:
      ibp--;
      obp--;
      redo_char = 1;
    default:
randomchar:
      if (ident_length > 0) {
	register HASHNODE *hp;
	if (!pcp_outfile || pcp_inside_if) {
startagain:
	  for (hp = hashtab[(hash & 0x7fffffff)  % 1403]; hp != 0L;
	       hp = hp->next) {
	    if (hp->length == ident_length) {
	      int obufp_before_macroname;
	      int op_lineno_before_macroname;
	      register int i = ident_length;
	      register U_CHAR *p = hp->name;
	      register U_CHAR *q = obp - i;
	      int disabled;
	      if (! redo_char)
		q--;
	      do {		
		if (*p++ != *q++)
		  goto hashcollision;
	      } while (--i);
	      if (! redo_char) {
		ibp--;
		obp--;
	      }
	      obufp_before_macroname = (obp - op->buf) - ident_length;
	      op_lineno_before_macroname = op->lineno;
	      if (hp->type == T_PCSTRING) {
		pcstring_used (hp); 
		break;		
	      }
	      disabled = hp->type == T_DISABLED;
	      if (disabled) {
		if (output_marks) {
		    (((op)->length - ((op)->bufp - (op)->buf) <= ( limit - ibp + 2))      ? grow_outbuf ((op), ( limit - ibp + 2)) : 0);
		  *obp++ = '\n';
		  *obp++ = '-';
		}
		break;
	      }
	      if ((hp->type == T_MACRO || hp->type == T_DISABLED)
		  && hp->value.defn->nargs >= 0)
		{
		  U_CHAR *old_ibp = ibp;
		  U_CHAR *old_obp = obp;
		  int old_iln = ip->lineno;
		  int old_oln = op->lineno;
		  while (1) {
		    if (ibp == limit && ip->macro != 0) {
		      do { ip->macro->type = T_MACRO;		     if (ip->free_ptr) free (ip->free_ptr);	     --indepth; } while (0);
		      do { ip = &instack[indepth];		     ibp = ip->bufp;			     limit = ip->buf + ip->length;	     op->bufp = obp;			       (((op)->length - ((op)->bufp - (op)->buf) <= ( limit - ibp))      ? grow_outbuf ((op), ( limit - ibp)) : 0);	     beg_of_line = 0;			     obp = op->bufp; } while (0);
		      old_ibp = ibp;
		      old_obp = obp;
		      old_iln = ip->lineno;
		      old_oln = op->lineno;
		    }
		    else if (*ibp == '/' && ibp+1 != limit && ibp[1] == '*') {
		      if (put_out_comments) {
			*obp++ = '/';
			*obp++ = '*';
		      } else if (! traditional) {
			*obp++ = ' ';
		      }
		      ibp += 2;
		      while (ibp + 1 != limit
			     && !(ibp[0] == '*' && ibp[1] == '/')) {
			if (*ibp == '\n') {
			  ++ip->lineno;
			  ++op->lineno;
			}
			if (put_out_comments)
			  *obp++ = *ibp++;
			else
			  ibp++;
		      }
		      ibp += 2;
		      if (put_out_comments) {
			*obp++ = '*';
			*obp++ = '/';
		      }
		    }
		    else if (is_space[*ibp]) {
		      *obp++ = *ibp++;
		      if (ibp[-1] == '\n') {
			if (ip->macro == 0) {
			  ++ip->lineno;
			  ++op->lineno;
			} else if (!output_marks) {
			  obp--;
			  if (*ibp == '-')
			    ibp++;
			  else {
			    if (*ibp == '\n')
			      ++op->lineno;
			    *obp++ = *ibp++;
			  }
			} else {
			  *obp++ = *ibp++;
			}
		      }
		    }
		    else break;
		  }
		  if (*ibp != '(') {
		    ibp = old_ibp;
		    obp = old_obp;
		    ip->lineno = old_iln;
		    op->lineno = old_oln;
		    break;
		  }
		}
	      obp = op->buf + obufp_before_macroname;
	      op->lineno = op_lineno_before_macroname;
	      ip->bufp = ibp;
	      op->bufp = obp;
	      macroexpand (hp, op);
	      obp = op->bufp;
	      do { ip = &instack[indepth];		     ibp = ip->bufp;			     limit = ip->buf + ip->length;	     op->bufp = obp;			       (((op)->length - ((op)->bufp - (op)->buf) <= ( limit - ibp))      ? grow_outbuf ((op), ( limit - ibp)) : 0);	     beg_of_line = 0;			     obp = op->bufp; } while (0);
	      break;
	    }
hashcollision:
	    ;
	  }			
	}
	ident_length = hash = 0; 
	redo_char = 0;
	concatenated = 0;
      }				
    }				
  }				
 ending:
  if (if_stack != ip->if_stack) {
    char *str;
    switch (if_stack->type) {
    case T_IF:
      str = "if";
      break;
    case T_IFDEF:
      str = "ifdef";
      break;
    case T_IFNDEF:
      str = "ifndef";
      break;
    case T_ELSE:
      str = "else";
      break;
    case T_ELIF:
      str = "elif";
      break;
    }
    error_with_line (line_for_error (if_stack->lineno),
		     "unterminated `#%s' conditional", str);
  }
  if_stack = ip->if_stack;
}
static FILE_BUF
expand_to_temp_buffer (buf, limit, output_marks, assertions)
     U_CHAR *buf, *limit;
     int output_marks, assertions;
{
  register FILE_BUF *ip;
  FILE_BUF obuf;
  int length = limit - buf;
  U_CHAR *buf1;
  int odepth = indepth;
  int save_assertions_flag = assertions_flag;
  assertions_flag = assertions;
  if (length < 0)
    abort ();
  buf1 = (U_CHAR *) alloca (length + 1);
  {
    register U_CHAR *p1 = buf;
    register U_CHAR *p2 = buf1;
    while (p1 != limit)
      *p2++ = *p1++;
  }
  buf1[length] = 0;
  obuf.length = length * 2 + 100; 
  obuf.bufp = obuf.buf = (U_CHAR *) xmalloc (obuf.length);
  obuf.fname = 0;
  obuf.macro = 0;
  obuf.free_ptr = 0;
    if (indepth >= (200 - 1))					    {									      error_with_line (line_for_error (instack[indepth].lineno),			       "macro or `#include' recursion too deep");	      {return obuf;};								    };
  ++indepth;
  ip = &instack[indepth];
  ip->fname = 0;
  ip->nominal_fname = 0;
  ip->system_header_p = 0;
  ip->macro = 0;
  ip->free_ptr = 0;
  ip->length = length;
  ip->buf = ip->bufp = buf1;
  ip->if_stack = if_stack;
  ip->lineno = obuf.lineno = 1;
  rescan (&obuf, output_marks);
  --indepth;
  if (indepth != odepth)
    abort ();
  obuf.length = obuf.bufp - obuf.buf;
  assertions_flag = save_assertions_flag;
  return obuf;
}
static int
handle_directive (ip, op)
     FILE_BUF *ip, *op;
{
  register U_CHAR *bp, *cp;
  register struct directive *kt;
  register int ident_length;
  U_CHAR *resume_p;
  int copy_command = 0;
  U_CHAR *ident, *after_ident;
  bp = ip->bufp;
  directive_start = bp - 1;
  while (1) {
    if (is_hor_space[*bp]) {
      if ((*bp == '\f' || *bp == '\v') && pedantic)
	pedwarn ("%s in preprocessing directive",
		 *bp == '\f' ? "formfeed" : "vertical tab");
      bp++;
    } else if (*bp == '/' && bp[1] == '*') {
      ip->bufp = bp;
      skip_to_end_of_comment (ip, &ip->lineno, 0);
      bp = ip->bufp;
    } else if (*bp == '\\' && bp[1] == '\n') {
      bp += 2; ip->lineno++;
    } else break;
  }
  cp = bp;
  while (1) {
    if (is_idchar[*cp])
      cp++;
    else {
      if (*cp == '\\' && cp[1] == '\n')
	name_newline_fix (cp);
      if (is_idchar[*cp])
	cp++;
      else break;
    }
  }
  ident_length = cp - bp;
  ident = bp;
  after_ident = cp;
  if (ident_length == 0 && *after_ident == '\n') {
    ip->bufp = after_ident;
    return 1;
  }
  if (ident_length == 0 || !is_idstart[*ident]) {
    U_CHAR *p = ident;
    while (is_idchar[*p]) {
      if (*p < '0' || *p > '9')
	break;
      p++;
    }
    if (p != ident && !is_idchar[*p]) {
      static struct directive line_directive_table[] = {
	{  4, do_line, "line", T_LINE},
      };
      if (pedantic)
	pedwarn ("`#' followed by integer");
      after_ident = ident;
      kt = line_directive_table;
      goto old_linenum;
    }
    if (p == ident) {
      while (*p == '#' || is_hor_space[*p]) p++;
      if (*p == '\n') {
	if (pedantic && !lang_asm)
	  warning ("invalid preprocessor directive");
	return 0;
      }
    }
    if (!lang_asm)
      error ("invalid preprocessor directive name");
    return 0;
  }
  for (kt = directive_table; kt->length > 0; kt++) {
    if (kt->length == ident_length && !strncmp (kt->name, ident, ident_length)) {
      register U_CHAR *buf;
      register U_CHAR *limit;
      int unterminated;
      int junk;
      int *already_output = 0;
      int keep_comments;
    old_linenum:
      limit = ip->buf + ip->length;
      unterminated = 0;
      keep_comments = traditional && kt->traditional_comments;
      if (kt->type == T_IMPORT && !(objc || lookup ("__NeXT__", -1, -1)))
	break;
      buf = bp = after_ident;
      while (bp < limit) {
	register U_CHAR c = *bp++;
	switch (c) {
	case '\\':
	  if (bp < limit) {
	    if (*bp == '\n') {
	      ip->lineno++;
	      copy_command = 1;
	    }
	    bp++;
	  }
	  break;
	case '\'':
	case '\"':
	  bp = skip_quoted_string (bp - 1, limit, ip->lineno, &ip->lineno, &copy_command, &unterminated);
	  if (unterminated) {
	    if (traditional) {
	      ip->bufp = bp;
	      goto endloop1;
	    }
	    ip->bufp = bp;
	    return 1;
	  }
	  break;
	case '<':
	  if (!kt->angle_brackets)
	    break;
	  while (*bp && *bp != '>') bp++;
	  break;
	case '/':
	  if (*bp == '\\' && bp[1] == '\n')
	    newline_fix (bp);
	  if (*bp == '*'
	      || (cplusplus_comments && *bp == '/')) {
	    U_CHAR *obp = bp - 1;
	    ip->bufp = bp + 1;
	    skip_to_end_of_comment (ip, &ip->lineno, 0);
	    bp = ip->bufp;
	    if (bp == limit || *bp == '\n') {
	      bp = obp;
	      goto endloop1;
	    }
	    if (! keep_comments)
	      copy_command++;
	  }
	  break;
	case '\f':
	case '\v':
	  if (pedantic)
	    pedwarn ("%s in preprocessing directive",
		     c == '\f' ? "formfeed" : "vertical tab");
	  break;
	case '\n':
	  --bp;		
	  ip->bufp = bp;
	  goto endloop1;
	}
      }
      ip->bufp = bp;
    endloop1:
      resume_p = ip->bufp;
      if (!no_output && kt->pass_thru && put_out_comments) {
        int len;
          (((op)->length - ((op)->bufp - (op)->buf) <= ( kt->length + 2))      ? grow_outbuf ((op), ( kt->length + 2)) : 0);
	if (op->bufp > op->buf && op->bufp[-1] != '\n') {
	  op->lineno++;
	  *op->bufp++ = '\n';
	}
        *op->bufp++ = '#';
        bcopy (kt->name, op->bufp, kt->length);
        op->bufp += kt->length;
	len = (bp - buf);
	  (((op)->length - ((op)->bufp - (op)->buf) <= ( len))      ? grow_outbuf ((op), ( len)) : 0);
	bcopy (buf, op->bufp, len);
	op->bufp += len;
	while (--len >= 0)
	  if (buf[len] == '\n')
	    op->lineno++;
	already_output = &junk;
      }				
      if (copy_command) {
	register U_CHAR *xp = buf;
	cp = (U_CHAR *) alloca (bp - buf + 5); 
	buf = cp;
	while (xp < bp) {
	  register U_CHAR c = *xp++;
	  *cp++ = c;
	  switch (c) {
	  case '\n':
	    abort ();  
	    break;
	  case '<':
	    if (!kt->angle_brackets)
	      break;
	    while (xp < bp && c != '>') {
	      c = *xp++;
	      if (c == '\\' && xp < bp && *xp == '\n')
		xp++;
	      else
		*cp++ = c;
	    }
	    break;
	  case '\\':
	    if (*xp == '\n') {
	      xp++;
	      cp--;
	      if (cp != buf && is_space[cp[-1]]) {
		while (cp != buf && is_space[cp[-1]]) cp--;
		cp++;
		do { while (is_hor_space[*xp]) xp++; } while (0);
	      } else if (is_space[*xp]) {
		*cp++ = *xp++;
		do { while (is_hor_space[*xp]) xp++; } while (0);
	      }
	    } else {
	      *cp++ = *xp++;
	    }
	    break;
	  case '\'':
	  case '\"':
	    {
	      register U_CHAR *bp1
		= skip_quoted_string (xp - 1, bp, ip->lineno,
				      ((char *)0), ((char *)0), ((char *)0));
	      while (xp != bp1)
		if (*xp == '\\') {
		  if (*++xp != '\n')
		    *cp++ = '\\';
		  else
		    xp++;
		} else
		  *cp++ = *xp++;
	    }
	    break;
	  case '/':
	    if (*xp == '*'
		|| (cplusplus_comments && *xp == '/')) {
	      ip->bufp = xp + 1;
	      skip_to_end_of_comment (ip, already_output, 0);
	      if (keep_comments)
		while (xp != ip->bufp)
		  *cp++ = *xp++;
	      else if (traditional)
		cp--;
	      else
		cp[-1] = ' ';
	      xp = ip->bufp;
	    }
	  }
	}
	*cp = 0;
      } else
	cp = bp;
      ip->bufp = resume_p;
      if (!no_output && already_output == 0
	  && (kt->pass_thru
	      || (kt->type == T_DEFINE
		  && (dump_macros == dump_names
		      || dump_macros == dump_definitions)))) {
        int len;
          (((op)->length - ((op)->bufp - (op)->buf) <= ( kt->length + 1))      ? grow_outbuf ((op), ( kt->length + 1)) : 0);
        *op->bufp++ = '#';
        bcopy (kt->name, op->bufp, kt->length);
        op->bufp += kt->length;
	if (kt->pass_thru || dump_macros == dump_definitions) {
	  len = (cp - buf);
	    (((op)->length - ((op)->bufp - (op)->buf) <= ( len))      ? grow_outbuf ((op), ( len)) : 0);
	  bcopy (buf, op->bufp, len);
	  op->bufp += len;
	} else if (kt->type == T_DEFINE && dump_macros == dump_names) {
	  U_CHAR *xp = buf;
	  U_CHAR *yp;
	  do { while (is_hor_space[*xp]) xp++; } while (0);
	  yp = xp;
	  while (is_idchar[*xp]) xp++;
	  len = (xp - yp);
	    (((op)->length - ((op)->bufp - (op)->buf) <= ( len + 1))      ? grow_outbuf ((op), ( len + 1)) : 0);
	  *op->bufp++ = ' ';
	  bcopy (yp, op->bufp, len);
	  op->bufp += len;
	}
      }				
      (*kt->func) (buf, cp, op, kt);
        (((op)->length - ((op)->bufp - (op)->buf) <= ( ip->length - (ip->bufp - ip->buf)))      ? grow_outbuf ((op), ( ip->length - (ip->bufp - ip->buf))) : 0);
      return 1;
    }
  }
  return 0;
}
static struct tm *
timestamp ()
{
  static struct tm *timebuf;
  if (!timebuf) {
    time_t t = time (0);
    timebuf = localtime (&t);
  }
  return timebuf;
}
static char *monthnames[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
			     "Jul", "Aug", "Sep", "Oct", "Nov", "Dec",
			    };
static void
special_symbol (hp, op)
     HASHNODE *hp;
     FILE_BUF *op;
{
  char *buf;
  int i, len;
  int true_indepth;
  FILE_BUF *ip = 0L;
  struct tm *timebuf;
  int paren = 0;		
  if (pcp_outfile && pcp_inside_if
      && hp->type != T_SPEC_DEFINED && hp->type != T_CONST)
    error ("Predefined macro `%s' used inside `#if' during precompilation",
	   hp->name);
  for (i = indepth; i >= 0; i--)
    if (instack[i].fname != 0L) {
      ip = &instack[i];
      break;
    }
  if (ip == 0L) {
    error ("cccp error: not in any file?!");
    return;			
  }
  switch (hp->type) {
  case T_FILE:
  case T_BASE_FILE:
    {
      char *string;
      if (hp->type == T_FILE)
	string = ip->nominal_fname;
      else
	string = instack[0].nominal_fname;
      if (string)
	{
	  buf = (char *) alloca (3 + strlen (string));
	  sprintf (buf, "\"%s\"", string);
	}
      else
	buf = "\"\"";
      break;
    }
  case T_INCLUDE_LEVEL:
    true_indepth = 0;
    for (i = indepth; i >= 0; i--)
      if (instack[i].fname != 0L)
        true_indepth++;
    buf = (char *) alloca (8);	
    sprintf (buf, "%d", true_indepth - 1);
    break;
  case T_VERSION:
    buf = (char *) alloca (3 + strlen (version_string));
    sprintf (buf, "\"%s\"", version_string);
    break;
  case T_SIZE_TYPE:
    buf = (char *) alloca (3 + strlen ("long unsigned int"));
    sprintf (buf, "%s", "long unsigned int");
    break;
  case T_PTRDIFF_TYPE:
    buf = (char *) alloca (3 + strlen ("long int"));
    sprintf (buf, "%s", "long int");
    break;
  case T_WCHAR_TYPE:
    buf = (char *) alloca (3 + strlen ("short unsigned int"));
    sprintf (buf, "%s", "short unsigned int");
    break;
  case T_CONST:
    buf = (char *) alloca (4 * sizeof (int));
    sprintf (buf, "%d", hp->value.ival);
    if (pcp_inside_if && pcp_outfile)
      fprintf (pcp_outfile, "#define %s %d\n", hp->name, hp->value.ival);
    break;
  case T_SPECLINE:
    buf = (char *) alloca (10);
    sprintf (buf, "%d", ip->lineno);
    break;
  case T_DATE:
  case T_TIME:
    buf = (char *) alloca (20);
    timebuf = timestamp ();
    if (hp->type == T_DATE)
      sprintf (buf, "\"%s %2d %4d\"", monthnames[timebuf->tm_mon],
	      timebuf->tm_mday, timebuf->tm_year + 1900);
    else
      sprintf (buf, "\"%02d:%02d:%02d\"", timebuf->tm_hour, timebuf->tm_min,
	      timebuf->tm_sec);
    break;
  case T_SPEC_DEFINED:
    buf = " 0 ";		
    ip = &instack[indepth];
    do { while (is_hor_space[*ip->bufp]) ip->bufp++; } while (0);
    if (*ip->bufp == '(') {
      paren++;
      ip->bufp++;			
      do { while (is_hor_space[*ip->bufp]) ip->bufp++; } while (0);
    }
    if (!is_idstart[*ip->bufp])
      goto oops;
    if (hp = lookup (ip->bufp, -1, -1)) {
      if (pcp_outfile && pcp_inside_if
	  && hp->value.defn->predefined)
	fprintf (pcp_outfile, "#define %s\n", hp->name);
      buf = " 1 ";
    }
    else
      if (pcp_outfile && pcp_inside_if)	{
	U_CHAR *cp = ip->bufp;
	fprintf (pcp_outfile, "#undef ");
	while (is_idchar[*cp]) 
	  fputc (*cp++, pcp_outfile);
			(--(  pcp_outfile)->_cnt < 0 ? 			_flsbuf((int) ('\n'), (  pcp_outfile)) : 			(int) (*(  pcp_outfile)->_ptr++ = (unsigned char) ('\n')));
      }
    while (is_idchar[*ip->bufp])
      ++ip->bufp;
    do { while (is_hor_space[*ip->bufp]) ip->bufp++; } while (0);
    if (paren) {
      if (*ip->bufp != ')')
	goto oops;
      ++ip->bufp;
    }
    break;
oops:
    error ("`defined' without an identifier");
    break;
  default:
    error ("cccp error: invalid special hash type"); 
    abort ();
  }
  len = strlen (buf);
    (((op)->length - ((op)->bufp - (op)->buf) <= ( len))      ? grow_outbuf ((op), ( len)) : 0);
  bcopy (buf, op->bufp, len);
  op->bufp += len;
  return;
}
static int
do_include (buf, limit, op, keyword)
     U_CHAR *buf, *limit;
     FILE_BUF *op;
     struct directive *keyword;
{
  int importing = (keyword->type == T_IMPORT);
  int skip_dirs = (keyword->type == T_INCLUDE_NEXT);
  static int import_warning = 0;
  char *fname;		
  char *pcftry;
  char *pcfname;
  U_CHAR *fbeg, *fend;		
  struct file_name_list *search_start = include; 
  struct file_name_list dsp[1];	
  struct file_name_list *searchptr = 0;
  int flen;
  int f;			
  int retried = 0;		
  FILE_BUF trybuf;		
  int angle_brackets = 0;	
  int pcf = -1;
  char *pcfbuf;
  int pcfbuflimit;
  int pcfnum;
  f= -1;			
  if (importing && warn_import && !inhibit_warnings
      && !instack[indepth].system_header_p && !import_warning) {
    import_warning = 1;
    warning ("using `#import' is not recommended");
    fprintf ((&_iob[2]), "The fact that a certain header file need not be processed more than once\n");
    fprintf ((&_iob[2]), "should be indicated in the header file, not where it is used.\n");
    fprintf ((&_iob[2]), "The best way to do this is with a conditional of this form:\n\n");
    fprintf ((&_iob[2]), "  #ifndef _FOO_H_INCLUDED\n");
    fprintf ((&_iob[2]), "  #define _FOO_H_INCLUDED\n");
    fprintf ((&_iob[2]), "  ... <real contents of file> ...\n");
    fprintf ((&_iob[2]), "  #endif /* Not _FOO_H_INCLUDED */\n\n");
    fprintf ((&_iob[2]), "Then users can use `#include' any number of times.\n");
    fprintf ((&_iob[2]), "GNU C automatically avoids processing the file more than once\n");
    fprintf ((&_iob[2]), "when it is equipped with such a conditional.\n");
  }
get_filename:
  fbeg = buf;
  do { while (is_hor_space[*fbeg]) fbeg++; } while (0);
  while (limit != fbeg && is_hor_space[limit[-1]]) limit--;
  switch (*fbeg++) {
  case '\"':
    {
      FILE_BUF *fp;
      {
	U_CHAR *fin = fbeg;
	fbeg = (U_CHAR *) alloca (limit - fbeg + 1);
	fend = fbeg;
	while (fin != limit) {
	  while (fin != limit && *fin != '\"')
	    *fend++ = *fin++;
	  fin++;
	  if (fin == limit)
	    break;
	  while (fin != limit && is_hor_space[*fin]) fin++;
	  if (fin != limit && *fin == '\"')
	    fin++;
	  else
	    goto fail;
	}
      }
      *fend++ = 0;
      if (ignore_srcdir) break;
      for (fp = &instack[indepth]; fp >= instack; fp--)
	{
	  int n;
	  char *ep,*nam;
	  if ((nam = fp->nominal_fname) != 0L) {
	    dsp[0].next = search_start;
	    search_start = dsp;
	    ep = rindex (nam, '/');
	    if (ep != 0L) {
	      n = ep - nam;
	      dsp[0].fname = (char *) alloca (n + 1);
	      strncpy (dsp[0].fname, nam, n);
	      dsp[0].fname[n] = '\0';
	      if (n + 0 > max_include_len)
		max_include_len = n + 0;
	    } else {
	      dsp[0].fname = 0; 
	    }
	    break;
	  }
	}
      break;
    }
  case '<':
    fend = fbeg;
    while (fend != limit && *fend != '>') fend++;
    if (*fend == '>' && fend + 1 == limit) {
      angle_brackets = 1;
      if (first_bracket_include)
	search_start = first_bracket_include;
      break;
    }
    goto fail;
  default:
  fail:
    if (retried) {
      if (importing)
        error ("`#import' expects \"fname\" or <fname>");
      else
        error ("`#include' expects \"fname\" or <fname>");
      return 0;
    } else {
      trybuf = expand_to_temp_buffer (buf, limit, 0, 0);
      buf = (U_CHAR *) alloca (trybuf.bufp - trybuf.buf + 1);
      bcopy (trybuf.buf, buf, trybuf.bufp - trybuf.buf);
      limit = buf + (trybuf.bufp - trybuf.buf);
      free (trybuf.buf);
      retried++;
      goto get_filename;
    }
  }
  if (skip_dirs) {
    FILE_BUF *fp;
    for (fp = &instack[indepth]; fp >= instack; fp--)
      if (fp->fname != 0L) {
	if (fp->dir)
	  search_start = fp->dir->next;
	break;
      }
  }
  flen = fend - fbeg;
  fname = (char *) xmalloc (max_include_len + flen + 2);
  if (*fbeg == '/') {
    strncpy (fname, fbeg, flen);
    fname[flen] = 0;
    if (redundant_include_p (fname))
      return 0;
    if (importing)
      f = lookup_import (fname);
    else
      f = open (fname, 0, 0666);
    if (f == -2)
      return 0;		
  } else {
    for (searchptr = search_start; searchptr; searchptr = searchptr->next) {
      if (searchptr->fname) {
	if (searchptr->fname[0] == 0)
	  continue;
	strcpy (fname, searchptr->fname);
	strcat (fname, "/");
	fname[strlen (fname) + flen] = 0;
      } else {
	fname[0] = 0;
      }
      strncat (fname, fbeg, flen);
      if (importing)
	f = lookup_import (fname);
      else
	f = open (fname, 0, 0666);
      if (f == -2)
	return 0;			
      if (redundant_include_p (fname)) {
	close (f);
	return 0;
      }
      if (f >= 0)
	break;
    }
  }
  if (f < 0) {
    strncpy (fname, fbeg, flen);
    fname[flen] = 0;
    if (search_start)
      error_from_errno (fname);
    else
      error ("No include path in which to find %s", fname);
    if (print_deps > (angle_brackets || (system_include_depth > 0))) {
      deps_output ("", 0);
      if (angle_brackets) {
	for (searchptr = search_start; searchptr; searchptr = searchptr->next) {
	  if (searchptr->fname) {
	    if (searchptr->fname[0] == 0)
	      continue;
	    deps_output (searchptr->fname, 0);
	    deps_output ("/", 0);
	    break;
	  }
	}
      }
      deps_output (fbeg, flen);
      deps_output (" ", 0);
    }
  } else {
    struct stat stat_f;
    struct file_name_list* ptr;
    for (ptr = dont_repeat_files; ptr; ptr = ptr->next) {
      if (!strcmp (ptr->fname, fname)) {
	close (f);
        return 0;				
      }
    }
    for (ptr = all_include_files; ptr; ptr = ptr->next) {
      if (!strcmp (ptr->fname, fname))
        break;				
    }
    if (ptr == 0) {
      ptr = (struct file_name_list *) xmalloc (sizeof (struct file_name_list));
      ptr->control_macro = 0;
      ptr->next = all_include_files;
      all_include_files = ptr;
      ptr->fname = savestring (fname);
      if (print_deps > (angle_brackets || (system_include_depth > 0))) {
	deps_output ("", 0);
	deps_output (fname, 0);
	deps_output (" ", 0);
      }
    }   
    if (print_include_names)
      fprintf ((&_iob[2]), "%s\n", fname);
    if (angle_brackets)
      system_include_depth++;
    add_import (f, fname);	
    pcftry = (char *) alloca (strlen (fname) + 30);
    pcfbuf = 0;
    pcfnum = 0;
    fstat (f, &stat_f);
    if (!no_precomp)
      do {
	sprintf (pcftry, "%s%d", fname, pcfnum++);
	pcf = open (pcftry, 0, 0666);
	if (pcf != -1)
	  {
	    struct stat s;
	    fstat (pcf, &s);
	    if (bcmp (&stat_f.st_ino, &s.st_ino, sizeof (s.st_ino))
		|| stat_f.st_dev != s.st_dev)
	      {
		pcfbuf = check_precompiled (pcf, fname, &pcfbuflimit);
		close (pcf);
	      }
	    else
	      {
		close (pcf);
		break;
	      }
	  }
      } while (pcf != -1 && !pcfbuf);
    if (pcfbuf) {
      pcfname = xmalloc (strlen (pcftry) + 1);
      strcpy (pcfname, pcftry);
      pcfinclude (pcfbuf, pcfbuflimit, fname, op);
    }
    else
      finclude (f, fname, op, is_system_include (fname), searchptr);
    if (angle_brackets)
      system_include_depth--;
  }
  return 0;
}
static int
redundant_include_p (name)
     char *name;
{
  struct file_name_list *l = all_include_files;
  for (; l; l = l->next)
    if (! strcmp (name, l->fname)
	&& l->control_macro
	&& lookup (l->control_macro, -1, -1))
      return 1;
  return 0;
}
static int
is_system_include (filename)
    register char *filename;
{
  struct file_name_list *searchptr;
  for (searchptr = first_system_include; searchptr;
       searchptr = searchptr->next)
    if (searchptr->fname) {
      register char *sys_dir = searchptr->fname;
      register unsigned length = strlen (sys_dir);
      if (! strncmp (sys_dir, filename, length) && filename[length] == '/')
	return 1;
    }
  return 0;
}
static void
finclude (f, fname, op, system_header_p, dirptr)
     int f;
     char *fname;
     FILE_BUF *op;
     int system_header_p;
     struct file_name_list *dirptr;
{
  int st_mode;
  long st_size;
  long i;
  FILE_BUF *fp;			
  int missing_newline = 0;
    if (indepth >= (200 - 1))					    {									      error_with_line (line_for_error (instack[indepth].lineno),			       "macro or `#include' recursion too deep");	      return;;								    };
  if (file_size_and_mode (f, &st_mode, &st_size) < 0)
    {
      perror_with_name (fname);
      close (f);
      return;
    }
  fp = &instack[indepth + 1];
  bzero (fp, sizeof (FILE_BUF));
  fp->nominal_fname = fp->fname = fname;
  fp->length = 0;
  fp->lineno = 1;
  fp->if_stack = if_stack;
  fp->system_header_p = system_header_p;
  fp->dir = dirptr;
  if ((((st_mode)&(0170000)) == (0100000))) {
    fp->buf = (U_CHAR *) xmalloc (st_size + 2);
    fp->bufp = fp->buf;
    while (st_size > 0) {
      i = read (f, fp->buf + fp->length, st_size);
      if (i <= 0) {
	if (i == 0) break;
	goto nope;
      }
      fp->length += i;
      st_size -= i;
    }
  }
  else {
    U_CHAR *bufp;
    U_CHAR *basep;
    int bsize = 2000;
    st_size = 0;
    basep = (U_CHAR *) xmalloc (bsize + 2);
    fp->buf = basep; 
    bufp = basep;
    for (;;) {
      i = read (f, bufp, bsize - st_size);
      if (i < 0)
	goto nope;      
      if (i == 0)
	break;	
      st_size += i;
      bufp += i;
      if (bsize == st_size) {	
	  bsize *= 2;
	  basep = (U_CHAR *) xrealloc (basep, bsize + 2);
	  fp->buf = basep;
	  bufp = basep + st_size;	
	}
    }
    fp->bufp = fp->buf;
    fp->length = st_size;
  }
  close (f);
  indepth++;
  input_file_stack_tick++;
  if (!no_trigraphs)
    trigraph_pcp (fp);
  if ((fp->length > 0 && fp->buf[fp->length - 1] != '\n')
      || (fp->length > 1 && fp->buf[fp->length - 2] == '\\')) {
    fp->buf[fp->length++] = '\n';
    missing_newline = 1;
  }
  fp->buf[fp->length] = '\0';
  output_line_command (fp, op, 0, enter_file);
  rescan (op, 0);
  if (pedantic && missing_newline)
    pedwarn ("file does not end in newline");
  indepth--;
  input_file_stack_tick++;
  output_line_command (&instack[indepth], op, 0, leave_file);
  free (fp->buf);
  return;
 nope:
  perror_with_name (fname);
  close (f);
  free (fp->buf);
}
static void
record_control_macro (file, macro_name)
     char *file;
     U_CHAR *macro_name;
{
  struct file_name_list *new;
  for (new = all_include_files; new; new = new->next) {
    if (!strcmp (new->fname, file)) {
      new->control_macro = macro_name;
      return;
    }
  }
  abort ();
}
struct import_file {
  char *name;
  ino_t inode;
  dev_t dev;
  struct import_file *next;
};
static struct import_file *import_hash_table[31];
static int 
import_hash (f)
     char *f;
{
  int val = 0;
  while (*f) val += *f++;
  return (val%31);
}
static int
lookup_import (filename)
     char *filename;
{
  struct import_file *i;
  int h;
  int hashval;
  struct stat sb;
  int fd;
  hashval = import_hash (filename);
  i = import_hash_table[hashval];
  while (i) {
    if (!strcmp (filename, i->name))
      return -2;		
    i = i->next;
  }
  fd = open (filename, 0, 0666);
  if (fd < 0)
    return fd;
  fstat (fd, &sb);
  for (h = 0; h < 31; h++) {
    i = import_hash_table[h];
    while (i) {
      if (!bcmp (&i->inode, &sb.st_ino, sizeof (sb.st_ino))
	  && i->dev == sb.st_dev) {
        close (fd);
        return -2;		
      }
      i = i->next;
    }
  }
  return fd;			
}
static void
add_import (fd, fname)
     int fd;
     char *fname;
{
  struct import_file *i;
  int hashval;
  struct stat sb;
  hashval = import_hash (fname);
  fstat (fd, &sb);
  i = (struct import_file *)xmalloc (sizeof (struct import_file));
  i->name = (char *)xmalloc (strlen (fname)+1);
  strcpy (i->name, fname);
  bcopy (&sb.st_ino, &i->inode, sizeof (sb.st_ino));
  i->dev = sb.st_dev;
  i->next = import_hash_table[hashval];
  import_hash_table[hashval] = i;
}
static char *
check_precompiled (pcf, fname, limit)
     int pcf;
     char *fname;
     char **limit;
{
  int st_mode;
  long st_size;
  int length = 0;
  char *buf;
  char *dollar_loc;
  int i;
  char *cp;
  if (pcp_outfile)
    return 0;
  if (file_size_and_mode (pcf, &st_mode, &st_size) < 0)
    return 0;
  if ((((st_mode)&(0170000)) == (0100000)))
    {
      buf = xmalloc (st_size + 2);
      while (st_size > 0)
	{
	  i = read (pcf, buf + length, st_size);
	  if (i < 0)
	    goto nope;
	  if (i == 0)
	    break;
	  length += i;
	  st_size -= i;
	}	  
    }
  else
    abort ();
  if (length > 0 && buf[length-1] != '\n')
    buf[length++] = '\n';
  buf[length] = '\0';
  *limit = buf + length;
  if (!check_preconditions (buf))
    goto nope;
  for (cp = buf; *cp; cp++)
    ;
  return cp + 1;
 nope:
  free (buf);
  return 0;
}
static int 
check_preconditions (prec)
     char *prec;
{
  MACRODEF mdef;
  char *lineend;
  while (*prec) {
    lineend = (char *) index (prec, '\n');
    if (*prec++ != '#') {
      error ("Bad format encountered while reading precompiled file");
      return 0;
    }
    if (!strncmp (prec, "define", 6)) {
      HASHNODE *hp;
      prec += 6;
      mdef = create_definition (prec, lineend, ((char *)0));
      if (mdef.defn == 0)
	abort();
      if ((hp = lookup (mdef.symnam, mdef.symlen, -1)) == 0L
	  || (hp->type != T_MACRO && hp->type != T_CONST)
	  || (hp->type == T_MACRO
	      && !compare_defs (mdef.defn, hp->value.defn)
	      && (mdef.defn->length != 2
		  || mdef.defn->expansion[0] != '\n'
		  || mdef.defn->expansion[1] != ' ')))
	return 0;
    } else if (!strncmp (prec, "undef", 5)) {
      char *name;
      int len;
      prec += 5;
      while (is_hor_space[*prec])
	prec++;
      name = prec;
      while (is_idchar[*prec])
	prec++;
      len = prec - name;
      if (lookup (name, len, -1))
	return 0;
    } else {
      error ("Bad format encountered while reading precompiled file");
      return 0;
    }
    prec = lineend + 1;
  }
  return 1;
}
static void
pcfinclude (buf, limit, name, op)
     U_CHAR *buf, *limit, *name;
     FILE_BUF *op;
{
  FILE_BUF tmpbuf;
  int nstrings;
  U_CHAR *cp = buf;
  nstrings = *cp++;
  nstrings = (nstrings << 8) | *cp++;
  nstrings = (nstrings << 8) | *cp++;
  nstrings = (nstrings << 8) | *cp++;
  while (nstrings--) {
    U_CHAR *string_start;
    U_CHAR *endofthiskey;
    STRINGDEF *str;
    int nkeys;
    if ((int) cp & 3)
      cp += 4 - ((int) cp & 3);
    str = (STRINGDEF *) cp;
    string_start = cp += sizeof (STRINGDEF);
    for (; *cp; cp++)		
      ;
    tmpbuf = expand_to_temp_buffer (string_start, cp++, 0, 0);
    str->contents = tmpbuf.buf;
    str->len = tmpbuf.length;
    str->writeflag = 0;
    str->filename = name;
    str->output_mark = outbuf.bufp - outbuf.buf;
    str->chain = 0;
    *stringlist_tailp = str;
    stringlist_tailp = &str->chain;
    nkeys = *cp++;
    nkeys = (nkeys << 8) | *cp++;
    nkeys = (nkeys << 8) | *cp++;
    nkeys = (nkeys << 8) | *cp++;
    if (nkeys == -1)
      str->writeflag = 1;
    else
      for (; nkeys--; free (tmpbuf.buf), cp = endofthiskey + 1) {
	KEYDEF *kp = (KEYDEF *) cp;
	HASHNODE *hp;
	cp += sizeof (KEYDEF);
	endofthiskey = cp + strlen (cp);
	kp->str = str;
	tmpbuf = expand_to_temp_buffer (cp, endofthiskey, 0, 0);
	tmpbuf.bufp = tmpbuf.buf;
	while (is_hor_space[*tmpbuf.bufp])
	  tmpbuf.bufp++;
	if (!is_idstart[*tmpbuf.bufp]
	    || tmpbuf.bufp == tmpbuf.buf + tmpbuf.length) {
	  str->writeflag = 1;
	  continue;
	}
	hp = lookup (tmpbuf.bufp, -1, -1);
	if (hp == 0L) {
	  kp->chain = 0;
	  install (tmpbuf.bufp, -1, T_PCSTRING, 0, (char *) kp, -1);
	}
	else if (hp->type == T_PCSTRING) {
	  kp->chain = hp->value.keydef;
	  hp->value.keydef = kp;
	}
	else
	  str->writeflag = 1;
      }
  }
  output_line_command (&instack[indepth], op, 0, enter_file);
}
static void
pcstring_used (hp)
     HASHNODE *hp;
{
  KEYDEF *kp, *tmp;
  for (kp = hp->value.keydef; kp; kp = kp->chain)
    kp->str->writeflag = 1;
  delete_macro (hp);
}
static void
write_output ()
{
  STRINGDEF *next_string;
  U_CHAR *cur_buf_loc;
  int line_command_len = 80;
  char *line_command = xmalloc (line_command_len);
  int len;
  cur_buf_loc = outbuf.buf; 
  next_string = stringlist;
  while (cur_buf_loc < outbuf.bufp || next_string) {
    if (next_string
	&& cur_buf_loc - outbuf.buf == next_string->output_mark) {
      if (next_string->writeflag) {
	len = strlen (next_string->filename);
	if (len > line_command_len)
	  line_command = xrealloc (line_command, 
				   line_command_len *= 2);
	sprintf (line_command, "\n# %d \"%s\"\n",
		 next_string->lineno, next_string->filename);
	write ((((&_iob[1]))->_file), line_command, 
	       strlen (line_command));
	write ((((&_iob[1]))->_file),
	       next_string->contents, next_string->len);
      }	      
      next_string = next_string->chain;
    }
    else {
      len = (next_string
	     ? (next_string->output_mark 
		- (cur_buf_loc - outbuf.buf))
	     : outbuf.bufp - cur_buf_loc);
      write ((((&_iob[1]))->_file), cur_buf_loc, len);
      cur_buf_loc += len;
    }
  }
}
static void
pass_thru_directive (buf, limit, op, keyword)
     U_CHAR *buf, *limit;
     FILE_BUF *op;
     struct directive *keyword;
{
  register unsigned keyword_length = keyword->length;
    (((op)->length - ((op)->bufp - (op)->buf) <= ( 1 + keyword_length + (limit - buf)))      ? grow_outbuf ((op), ( 1 + keyword_length + (limit - buf))) : 0);
  *op->bufp++ = '#';
  bcopy (keyword->name, op->bufp, keyword_length);
  op->bufp += keyword_length;
  if (limit != buf && buf[0] != ' ')
    *op->bufp++ = ' ';
  bcopy (buf, op->bufp, limit - buf);
  op->bufp += (limit - buf);
}
struct arglist {
  struct arglist *next;
  U_CHAR *name;
  int length;
  int argno;
  char rest_args;
};
static MACRODEF
create_definition (buf, limit, op)
     U_CHAR *buf, *limit;
     FILE_BUF *op;
{
  U_CHAR *bp;			
  U_CHAR *symname;		
  int sym_length;		
  int line = instack[indepth].lineno;
  char *file = instack[indepth].nominal_fname;
  int rest_args = 0;
  DEFINITION *defn;
  int arglengths = 0;		
  MACRODEF mdef;
  bp = buf;
  while (is_hor_space[*bp])
    bp++;
  symname = bp;			
  sym_length = check_macro_name (bp, "macro");
  bp += sym_length;
  if (*bp == '(') {
    struct arglist *arg_ptrs = 0L;
    int argno = 0;
    bp++;			
    do { while (is_hor_space[*bp]) bp++; } while (0);
    while (*bp != ')') {
      struct arglist *temp;
      temp = (struct arglist *) alloca (sizeof (struct arglist));
      temp->name = bp;
      temp->next = arg_ptrs;
      temp->argno = argno++;
      temp->rest_args = 0;
      arg_ptrs = temp;
      if (rest_args)
	pedwarn ("another parameter follows `%s'",
		 rest_extension);
      if (!is_idstart[*bp])
	pedwarn ("invalid character in macro parameter name");
      while (is_idchar[*bp]) {
	bp++;
	if (limit - bp > (sizeof (rest_extension) - 1) &&
	    strncmp (rest_extension, bp, (sizeof (rest_extension) - 1)) == 0) {
	  rest_args = 1;
	  temp->rest_args = 1;
	  break;
	}
      }
      temp->length = bp - temp->name;
      if (rest_args == 1)
	bp += (sizeof (rest_extension) - 1);
      arglengths += temp->length + 2;
      do { while (is_hor_space[*bp]) bp++; } while (0);
      if (temp->length == 0 || (*bp != ',' && *bp != ')')) {
	error ("badly punctuated parameter list in `#define'");
	goto nope;
      }
      if (*bp == ',') {
	bp++;
	do { while (is_hor_space[*bp]) bp++; } while (0);
      }
      if (bp >= limit) {
	error ("unterminated parameter list in `#define'");
	goto nope;
      }
      {
	struct arglist *otemp;
	for (otemp = temp->next; otemp != 0L; otemp = otemp->next)
	  if (temp->length == otemp->length &&
	    strncmp(temp->name, otemp->name, temp->length) == 0) {
	      U_CHAR *name;
	      name = (U_CHAR *) alloca(temp->length + 1);
	      (void) strncpy(name, temp->name, temp->length);
	      name[temp->length] = '\0';
	      error ("duplicate argument name `%s' in `#define'", name);
	      goto nope;
	  }
      }
    }
    ++bp;			
    if (bp < limit && (*bp == ' ' || *bp == '\t')) ++bp;
    defn = collect_expansion (bp, limit, argno, arg_ptrs);
    defn->rest_args = rest_args;
    defn->args.argnames = (U_CHAR *) xmalloc (arglengths + 1);
    {
      struct arglist *temp;
      int i = 0;
      for (temp = arg_ptrs; temp; temp = temp->next) {
	bcopy (temp->name, &defn->args.argnames[i], temp->length);
	i += temp->length;
	if (temp->next != 0) {
	  defn->args.argnames[i++] = ',';
	  defn->args.argnames[i++] = ' ';
	}
      }
      defn->args.argnames[i] = 0;
    }
  } else {
    if (is_hor_space[*bp])
      ++bp;		
    defn = collect_expansion (bp, limit, -1, ((char *)0));
    defn->args.argnames = (U_CHAR *) "";
  }
  defn->line = line;
  defn->file = file;
  defn->predefined = !op;
  mdef.defn = defn;
  mdef.symnam = symname;
  mdef.symlen = sym_length;
  return mdef;
 nope:
  mdef.defn = 0;
  return mdef;
}
static int
do_define (buf, limit, op, keyword)
     U_CHAR *buf, *limit;
     FILE_BUF *op;
     struct directive *keyword;
{
  int hashcode;
  MACRODEF mdef;
  if (pcp_outfile && op)
    pass_thru_directive (buf, limit, op, keyword);
  mdef = create_definition (buf, limit, op);
  if (mdef.defn == 0)
    goto nope;
  hashcode = hashf (mdef.symnam, mdef.symlen, 1403);
  {
    HASHNODE *hp;
    if ((hp = lookup (mdef.symnam, mdef.symlen, hashcode)) != 0L) {
      int ok = 0;
      if (hp->type == T_PCSTRING)
	ok = 1;
      else if (hp->type == T_MACRO)
	ok = ! compare_defs (mdef.defn, hp->value.defn);
      else if (hp->type == T_CONST)
        ok = ! done_initializing;
      if (!ok) {
	U_CHAR *msg;		
        if (debug_output && op)
	  pass_thru_directive (buf, limit, op, keyword);
	msg = (U_CHAR *) alloca (mdef.symlen + 22);
	*msg = '`';
	bcopy (mdef.symnam, msg + 1, mdef.symlen);
	strcpy ((char *) (msg + mdef.symlen + 1), "' redefined");
	pedwarn (msg);
	if (hp->type == T_MACRO)
	  pedwarn_with_file_and_line (hp->value.defn->file, hp->value.defn->line,
				      "this is the location of the previous definition");
      }
      hp->type = T_MACRO;
      hp->value.defn = mdef.defn;
    } else {
      if (debug_output && op)
	pass_thru_directive (buf, limit, op, keyword);
      install (mdef.symnam, mdef.symlen, T_MACRO, 0,
	       (char *) mdef.defn, hashcode);
    }
  }
  return 0;
nope:
  return 1;
}
static int
check_macro_name (symname, usage)
     U_CHAR *symname;
     char *usage;
{
  U_CHAR *p;
  int sym_length;
  for (p = symname; is_idchar[*p]; p++)
    ;
  sym_length = p - symname;
  if (sym_length == 0)
    error ("invalid %s name", usage);
  else if (!is_idstart[*symname]) {
    U_CHAR *msg;			
    msg = (U_CHAR *) alloca (sym_length + 1);
    bcopy (symname, msg, sym_length);
    msg[sym_length] = 0;
    error ("invalid %s name `%s'", usage, msg);
  } else {
    if (! strncmp (symname, "defined", 7) && sym_length == 7)
      error ("invalid %s name `defined'", usage);
  }
  return sym_length;
}
static int
compare_defs (d1, d2)
     DEFINITION *d1, *d2;
{
  register struct reflist *a1, *a2;
  register U_CHAR *p1 = d1->expansion;
  register U_CHAR *p2 = d2->expansion;
  int first = 1;
  if (d1->nargs != d2->nargs)
    return 1;
  if (strcmp ((char *)d1->args.argnames, (char *)d2->args.argnames))
    return 1;
  for (a1 = d1->pattern, a2 = d2->pattern; a1 && a2;
       a1 = a1->next, a2 = a2->next) {
    if (!((a1->nchars == a2->nchars && ! strncmp (p1, p2, a1->nchars))
	  || ! comp_def_part (first, p1, a1->nchars, p2, a2->nchars, 0))
	|| a1->argno != a2->argno
	|| a1->stringify != a2->stringify
	|| a1->raw_before != a2->raw_before
	|| a1->raw_after != a2->raw_after)
      return 1;
    first = 0;
    p1 += a1->nchars;
    p2 += a2->nchars;
  }
  if (a1 != a2)
    return 1;
  if (comp_def_part (first, p1, d1->length - (p1 - d1->expansion),
		     p2, d2->length - (p2 - d2->expansion), 1))
    return 1;
  return 0;
}
static int
comp_def_part (first, beg1, len1, beg2, len2, last)
     int first;
     U_CHAR *beg1, *beg2;
     int len1, len2;
     int last;
{
  register U_CHAR *end1 = beg1 + len1;
  register U_CHAR *end2 = beg2 + len2;
  if (first) {
    while (beg1 != end1 && is_space[*beg1]) beg1++;
    while (beg2 != end2 && is_space[*beg2]) beg2++;
  }
  if (last) {
    while (beg1 != end1 && is_space[end1[-1]]) end1--;
    while (beg2 != end2 && is_space[end2[-1]]) end2--;
  }
  while (beg1 != end1 && beg2 != end2) {
    if (is_space[*beg1] && is_space[*beg2]) {
      while (beg1 != end1 && is_space[*beg1]) beg1++;
      while (beg2 != end2 && is_space[*beg2]) beg2++;
    } else if (*beg1 == *beg2) {
      beg1++; beg2++;
    } else break;
  }
  return (beg1 != end1) || (beg2 != end2);
}
static DEFINITION *
collect_expansion (buf, end, nargs, arglist)
     U_CHAR *buf, *end;
     int nargs;
     struct arglist *arglist;
{
  DEFINITION *defn;
  register U_CHAR *p, *limit, *lastp, *exp_p;
  struct reflist *endpat = 0L;
  U_CHAR *concat = 0;
  U_CHAR *stringify = 0;
  int maxsize;
  int expected_delimiter = '\0';
  if (end < buf)
    abort ();
  limit = end;
  p = buf;
  while (p < limit && is_space[limit[-1]]) limit--;
  while (p < limit && is_space[*p]) p++;
  maxsize = (sizeof (DEFINITION)
	     + 2 * (end - limit) + 2 * (p - buf)
	     + (limit - p) + 3);
  defn = (DEFINITION *) xcalloc (1, maxsize);
  defn->nargs = nargs;
  exp_p = defn->expansion = (U_CHAR *) defn + sizeof (DEFINITION);
  lastp = exp_p;
  p = buf;
  while (p < limit && is_space[*p]) {
    *exp_p++ = '\n';
    *exp_p++ = *p++;
  }
  if (limit - p >= 2 && p[0] == '#' && p[1] == '#') {
    error ("`##' at start of macro definition");
    p += 2;
  }
  while (p < limit) {
    int skipped_arg = 0;
    register U_CHAR c = *p++;
    *exp_p++ = c;
    if (!traditional) {
      switch (c) {
      case '\'':
      case '\"':
        if (expected_delimiter != '\0') {
          if (c == expected_delimiter)
            expected_delimiter = '\0';
        } else
          expected_delimiter = c;
	break;
      case '\\':
	if (p < limit && *p == '#' && !expected_delimiter) {
	  exp_p--;
	  *exp_p++ = *p++;
	} else if (p < limit && expected_delimiter) {
	  *exp_p++ = *p++;
	}
	break;
      case '#':
	if (expected_delimiter)
	  break;
	if (p < limit && *p == '#') {
	  exp_p--;
	  while (exp_p > lastp && is_hor_space[exp_p[-1]])
	    --exp_p;
	  p++;
	  do { while (is_hor_space[*p]) p++; } while (0);
	  concat = p;
	  if (p == limit)
	    error ("`##' at end of macro definition");
	} else {
	  exp_p--;
	  do { while (is_hor_space[*p]) p++; } while (0);
	  if (p == limit || ! is_idstart[*p] || nargs <= 0)
	    error ("`#' operator is not followed by a macro argument name");
	  else
	    stringify = p;
	}
	break;
      }
    } else {
      switch (c) {
      case '\'':
      case '\"':
	if (expected_delimiter != '\0') {
	  if (c == expected_delimiter)
	    expected_delimiter = '\0';
	} else
	  expected_delimiter = c;
	break;
      case '\\':
	if (expected_delimiter != 0 && p < limit
	    && (*p == expected_delimiter || *p == '\\')) {
	  *exp_p++ = *p++;
	  continue;
	}
	break;
      case '/':
	if (expected_delimiter != '\0') 
	  break;
	if (*p == '*') {
	  exp_p--;
	  p += 1;
	  while (p < limit && !(p[-2] == '*' && p[-1] == '/'))
	    p++;
	}
	break;
      }
    }
    if (is_idchar[c] && nargs > 0) {
      U_CHAR *id_beg = p - 1;
      int id_len;
      --exp_p;
      while (p != limit && is_idchar[*p]) p++;
      id_len = p - id_beg;
      if (is_idstart[c]) {
	register struct arglist *arg;
	for (arg = arglist; arg != 0L; arg = arg->next) {
	  struct reflist *tpat;
	  if (arg->name[0] == c
	      && arg->length == id_len
	      && strncmp (arg->name, id_beg, id_len) == 0) {
	    if (expected_delimiter && warn_stringify) {
	      if (traditional) {
		warning ("macro argument `%.*s' is stringified.",
			 id_len, arg->name);
	      } else {
		warning ("macro arg `%.*s' would be stringified with -traditional.",
			 id_len, arg->name);
	      }
	    }
	    if (!traditional && expected_delimiter)
	      break;
	    tpat = (struct reflist *) xmalloc (sizeof (struct reflist));
	    tpat->next = 0L;
	    tpat->raw_before = concat == id_beg;
	    tpat->raw_after = 0;
	    tpat->rest_args = arg->rest_args;
	    tpat->stringify = (traditional ? expected_delimiter != '\0'
			       : stringify == id_beg);
	    if (endpat == 0L)
	      defn->pattern = tpat;
	    else
	      endpat->next = tpat;
	    endpat = tpat;
	    tpat->argno = arg->argno;
	    tpat->nchars = exp_p - lastp;
	    {
	      register U_CHAR *p1 = p;
	      do { while (is_hor_space[*p1]) p1++; } while (0);
	      if (p1 + 2 <= limit && p1[0] == '#' && p1[1] == '#')
		tpat->raw_after = 1;
	    }
	    lastp = exp_p;	
	    skipped_arg = 1;
	    break;
	  }
	}
      }
      if (! skipped_arg) {
	register U_CHAR *lim1 = p;
	p = id_beg;
	while (p != lim1)
	  *exp_p++ = *p++;
	if (stringify == id_beg)
	  error ("`#' operator should be followed by a macro argument name");
      }
    }
  }
  if (limit < end) {
    while (limit < end && is_space[*limit]) {
      *exp_p++ = '\n';
      *exp_p++ = *limit++;
    }
  } else if (!traditional && expected_delimiter == 0) {
    *exp_p++ = '\n';
    *exp_p++ = ' ';
  }
  *exp_p = '\0';
  defn->length = exp_p - defn->expansion;
  if (defn->length + 1 > maxsize)
    abort ();
  return defn;
}
static int
do_assert (buf, limit, op, keyword)
     U_CHAR *buf, *limit;
     FILE_BUF *op;
     struct directive *keyword;
{
  U_CHAR *bp;			
  U_CHAR *symname;		
  int sym_length;		
  struct arglist *tokens = 0L;
  if (pedantic && done_initializing && !instack[indepth].system_header_p)
    pedwarn ("ANSI C does not allow `#assert'");
  bp = buf;
  while (is_hor_space[*bp])
    bp++;
  symname = bp;			
  sym_length = check_macro_name (bp, "assertion");
  bp += sym_length;
  do { while (is_hor_space[*bp]) bp++; } while (0);
  if (*bp != '(') {
    error ("missing token-sequence in `#assert'");
    return 1;
  }
  {
    int error_flag = 0;
    bp++;			
    do { while (is_hor_space[*bp]) bp++; } while (0);
    tokens = read_token_list (&bp, limit, &error_flag);
    if (error_flag)
      return 1;
    if (tokens == 0) {
      error ("empty token-sequence in `#assert'");
      return 1;
    }
    ++bp;			
    do { while (is_hor_space[*bp]) bp++; } while (0);
  }
  {
    ASSERTION_HASHNODE *hp;
    int hashcode = hashf (symname, sym_length, 37);
    struct tokenlist_list *value
      = (struct tokenlist_list *) xmalloc (sizeof (struct tokenlist_list));
    hp = assertion_lookup (symname, sym_length, hashcode);
    if (hp == 0L) {
      if (sym_length == 7 && ! strncmp (symname, "defined", sym_length))
	error ("`defined' redefined as assertion");
      hp = assertion_install (symname, sym_length, hashcode);
    }
    value->tokens = tokens;
    value->next = hp->value;
    hp->value = value;
  }
  return 0;
}
static int
do_unassert (buf, limit, op, keyword)
     U_CHAR *buf, *limit;
     FILE_BUF *op;
     struct directive *keyword;
{
  U_CHAR *bp;			
  U_CHAR *symname;		
  int sym_length;		
  struct arglist *tokens = 0L;
  int tokens_specified = 0;
  if (pedantic && done_initializing && !instack[indepth].system_header_p)
    pedwarn ("ANSI C does not allow `#unassert'");
  bp = buf;
  while (is_hor_space[*bp])
    bp++;
  symname = bp;			
  sym_length = check_macro_name (bp, "assertion");
  bp += sym_length;
  do { while (is_hor_space[*bp]) bp++; } while (0);
  if (*bp == '(') {
    int error_flag = 0;
    bp++;			
    do { while (is_hor_space[*bp]) bp++; } while (0);
    tokens = read_token_list (&bp, limit, &error_flag);
    if (error_flag)
      return 1;
    if (tokens == 0) {
      error ("empty token list in `#unassert'");
      return 1;
    }
    tokens_specified = 1;
    ++bp;			
    do { while (is_hor_space[*bp]) bp++; } while (0);
  }
  {
    ASSERTION_HASHNODE *hp;
    int hashcode = hashf (symname, sym_length, 37);
    struct tokenlist_list *tail, *prev;
    hp = assertion_lookup (symname, sym_length, hashcode);
    if (hp == 0L)
      return 1;
    if (! tokens_specified) {
      struct tokenlist_list *next;
      for (tail = hp->value; tail; tail = next) {
	next = tail->next;
	free_token_list (tail->tokens);
	free (tail);
      }
      delete_assertion (hp);
    } else {
      tail = hp->value;
      prev = 0;
      while (tail) {
	struct tokenlist_list *next = tail->next;
	if (compare_token_lists (tail->tokens, tokens)) {
	  if (prev)
	    prev->next = next;
	  else
	    hp->value = tail->next;
	  free_token_list (tail->tokens);
	  free (tail);
	} else {
	  prev = tail;
	}
	tail = next;
      }
    }
  }
  return 0;
}
int
check_assertion (name, sym_length, tokens_specified, tokens)
     U_CHAR *name;
     int sym_length;
     int tokens_specified;
     struct arglist *tokens;
{
  ASSERTION_HASHNODE *hp;
  int hashcode = hashf (name, sym_length, 37);
  if (pedantic && !instack[indepth].system_header_p)
    pedwarn ("ANSI C does not allow testing assertions");
  hp = assertion_lookup (name, sym_length, hashcode);
  if (hp == 0L)
    return 0;
  if (! tokens_specified)
    return 1;
  {
    struct tokenlist_list *tail;
    tail = hp->value;
    while (tail) {
      if (compare_token_lists (tail->tokens, tokens))
	return 1;
      tail = tail->next;
    }
    return 0;
  }
}
static int
compare_token_lists (l1, l2)
     struct arglist *l1, *l2;
{
  while (l1 && l2) {
    if (l1->length != l2->length)
      return 0;
    if (strncmp (l1->name, l2->name, l1->length))
      return 0;
    l1 = l1->next;
    l2 = l2->next;
  }
  return l1 == l2;
}
static struct arglist *
read_token_list (bpp, limit, error_flag)
     U_CHAR **bpp;
     U_CHAR *limit;
     int *error_flag;
{
  struct arglist *token_ptrs = 0;
  U_CHAR *bp = *bpp;
  int depth = 1;
  *error_flag = 0;
  while (depth > 0) {
    struct arglist *temp;
    int eofp = 0;
    U_CHAR *beg = bp;
    if (*bp == '(') {
      bp++;
      depth++;
    } else if (*bp == ')') {
      depth--;
      if (depth == 0)
	break;
      bp++;
    } else if (*bp == '"' || *bp == '\'')
      bp = skip_quoted_string (bp, limit, 0, ((char *)0), ((char *)0), &eofp);
    else
      while (! is_hor_space[*bp] && *bp != '(' && *bp != ')'
	     && *bp != '"' && *bp != '\'' && bp != limit)
	bp++;
    temp = (struct arglist *) xmalloc (sizeof (struct arglist));
    temp->name = (U_CHAR *) xmalloc (bp - beg + 1);
    bcopy (beg, temp->name, bp - beg);
    temp->name[bp - beg] = 0;
    temp->next = token_ptrs;
    token_ptrs = temp;
    temp->length = bp - beg;
    do { while (is_hor_space[*bp]) bp++; } while (0);
    if (bp >= limit) {
      error ("unterminated token sequence in `#assert' or `#unassert'");
      *error_flag = -1;
      return 0;
    }
  }
  *bpp = bp;
  {
    register struct arglist *prev = 0, *this, *next;
    for (this = token_ptrs; this; this = next) {
      next = this->next;
      this->next = prev;
      prev = this;
    }
    return prev;
  }
}
static void
free_token_list (tokens)
     struct arglist *tokens;
{
  while (tokens) {
    struct arglist *next = tokens->next;
    free (tokens->name);
    free (tokens);
    tokens = next;
  }
}
static ASSERTION_HASHNODE *
assertion_install (name, len, hash)
     U_CHAR *name;
     int len;
     int hash;
{
  register ASSERTION_HASHNODE *hp;
  register int i, bucket;
  register U_CHAR *p, *q;
  i = sizeof (ASSERTION_HASHNODE) + len + 1;
  hp = (ASSERTION_HASHNODE *) xmalloc (i);
  bucket = hash;
  hp->bucket_hdr = &assertion_hashtab[bucket];
  hp->next = assertion_hashtab[bucket];
  assertion_hashtab[bucket] = hp;
  hp->prev = 0L;
  if (hp->next != 0L)
    hp->next->prev = hp;
  hp->length = len;
  hp->value = 0;
  hp->name = ((U_CHAR *) hp) + sizeof (ASSERTION_HASHNODE);
  p = hp->name;
  q = name;
  for (i = 0; i < len; i++)
    *p++ = *q++;
  hp->name[len] = 0;
  return hp;
}
static ASSERTION_HASHNODE *
assertion_lookup (name, len, hash)
     U_CHAR *name;
     int len;
     int hash;
{
  register U_CHAR *bp;
  register ASSERTION_HASHNODE *bucket;
  bucket = assertion_hashtab[hash];
  while (bucket) {
    if (bucket->length == len && strncmp (bucket->name, name, len) == 0)
      return bucket;
    bucket = bucket->next;
  }
  return 0L;
}
static void
delete_assertion (hp)
     ASSERTION_HASHNODE *hp;
{
  if (hp->prev != 0L)
    hp->prev->next = hp->next;
  if (hp->next != 0L)
    hp->next->prev = hp->prev;
  if (hp == *hp->bucket_hdr)
    *hp->bucket_hdr = hp->next;
  free (hp);
}
static int
do_line (buf, limit, op, keyword)
     U_CHAR *buf, *limit;
     FILE_BUF *op;
     struct directive *keyword;
{
  register U_CHAR *bp;
  FILE_BUF *ip = &instack[indepth];
  FILE_BUF tem;
  int new_lineno;
  enum file_change_code file_change = same_file;
  tem = expand_to_temp_buffer (buf, limit, 0, 0);
  bp = tem.buf;
  do { while (is_hor_space[*bp]) bp++; } while (0);
  if (!(((*(__lc_ctype->core.iswctype)) == 0L) 		? (int) (__lc_ctype->_mask[*bp] & (0x010)) 		: (*(__lc_ctype->core.iswctype)) (*bp,0x010,__lc_ctype))) {
    error ("invalid format `#line' command");
    return 0;
  }
  new_lineno = atoi (bp) - 1;
  while ((((*(__lc_ctype->core.iswctype)) == 0L) 		? (int) (__lc_ctype->_mask[*bp] & (0x010)) 		: (*(__lc_ctype->core.iswctype)) (*bp,0x010,__lc_ctype)))
    bp++;
  do { while (is_hor_space[*bp]) bp++; } while (0);
  if (*bp == '\"') {
    static HASHNODE *fname_table[37];
    HASHNODE *hp, **hash_bucket;
    U_CHAR *fname;
    int fname_length;
    fname = ++bp;
    while (*bp && *bp != '\"')
      bp++;
    if (*bp != '\"') {
      error ("invalid format `#line' command");
      return 0;
    }
    fname_length = bp - fname;
    bp++;
    do { while (is_hor_space[*bp]) bp++; } while (0);
    if (*bp) {
      if (*bp == '1')
	file_change = enter_file;
      else if (*bp == '2')
	file_change = leave_file;
      else if (*bp == '3')
	ip->system_header_p = 1;
      else {
	error ("invalid format `#line' command");
	return 0;
      }
      bp++;
      do { while (is_hor_space[*bp]) bp++; } while (0);
      if (*bp == '3') {
	ip->system_header_p = 1;
	bp++;
	do { while (is_hor_space[*bp]) bp++; } while (0);
      }
      if (*bp) {
	error ("invalid format `#line' command");
	return 0;
      }
    }
    hash_bucket =
      &fname_table[hashf (fname, fname_length, 37)];
    for (hp = *hash_bucket; hp != 0L; hp = hp->next)
      if (hp->length == fname_length &&
	  strncmp (hp->value.cpval, fname, fname_length) == 0) {
	ip->nominal_fname = hp->value.cpval;
	break;
      }
    if (hp == 0) {
      hp = (HASHNODE *) xcalloc (1, sizeof (HASHNODE) + fname_length + 1);
      hp->next = *hash_bucket;
      *hash_bucket = hp;
      hp->length = fname_length;
      ip->nominal_fname = hp->value.cpval = ((char *) hp) + sizeof (HASHNODE);
      bcopy (fname, hp->value.cpval, fname_length);
    }
  } else if (*bp) {
    error ("invalid format `#line' command");
    return 0;
  }
  ip->lineno = new_lineno;
  output_line_command (ip, op, 0, file_change);
    (((op)->length - ((op)->bufp - (op)->buf) <= ( ip->length - (ip->bufp - ip->buf)))      ? grow_outbuf ((op), ( ip->length - (ip->bufp - ip->buf))) : 0);
  return 0;
}
static int
do_undef (buf, limit, op, keyword)
     U_CHAR *buf, *limit;
     FILE_BUF *op;
     struct directive *keyword;
{
  int sym_length;
  HASHNODE *hp;
  U_CHAR *orig_buf = buf;
  if (pcp_outfile && op)
    pass_thru_directive (buf, limit, op, keyword);
  do { while (is_hor_space[*buf]) buf++; } while (0);
  sym_length = check_macro_name (buf, "macro");
  while ((hp = lookup (buf, sym_length, -1)) != 0L) {
    if (debug_output && op)
      pass_thru_directive (orig_buf, limit, op, keyword);
    if (hp->type != T_MACRO)
      warning ("undefining `%s'", hp->name);
    delete_macro (hp);
  }
  if (pedantic) {
    buf += sym_length;
    do { while (is_hor_space[*buf]) buf++; } while (0);
    if (buf != limit)
      pedwarn ("garbage after `#undef' directive");
  }
  return 0;
}
static int
do_error (buf, limit, op, keyword)
     U_CHAR *buf, *limit;
     FILE_BUF *op;
     struct directive *keyword;
{
  int length = limit - buf;
  char *copy = (char *) xmalloc (length + 1);
  bcopy (buf, copy, length);
  copy[length] = 0;
  do { while (is_hor_space[*copy]) copy++; } while (0);
  error ("#error %s", copy);
  exit (33	);
  return 0;
}
static int
do_warning (buf, limit, op, keyword)
     U_CHAR *buf, *limit;
     FILE_BUF *op;
     struct directive *keyword;
{
  int length = limit - buf;
  char *copy = (char *) xmalloc (length + 1);
  bcopy (buf, copy, length);
  copy[length] = 0;
  do { while (is_hor_space[*copy]) copy++; } while (0);
  warning ("#warning %s", copy);
  return 0;
}
static int
do_once ()
{
  int i;
  FILE_BUF *ip = 0L;
  for (i = indepth; i >= 0; i--)
    if (instack[i].fname != 0L) {
      ip = &instack[i];
      break;
    }
  if (ip != 0L) {
    struct file_name_list *new;
    new = (struct file_name_list *) xmalloc (sizeof (struct file_name_list));
    new->next = dont_repeat_files;
    dont_repeat_files = new;
    new->fname = savestring (ip->fname);
    new->control_macro = 0;
  }
  return 0;
}
static int
do_ident (buf, limit)
     U_CHAR *buf, *limit;
{
  if (pedantic && !instack[indepth].system_header_p)
    pedwarn ("ANSI C does not allow `#ident'");
  return 0;
}
static int
do_pragma (buf, limit)
     U_CHAR *buf, *limit;
{
  while (*buf == ' ' || *buf == '\t')
    buf++;
  if (!strncmp (buf, "once", 4)) {
    if (!instack[indepth].system_header_p)
      warning ("`#pragma once' is obsolete");
    do_once ();
  }
  if (!strncmp (buf, "implementation", 14)) {
    struct file_name_list *ptr;
    U_CHAR *p = buf + 14, *fname, *inc_fname;
    do { while (is_hor_space[*p]) p++; } while (0);
    if (*p == '\n' || *p != '\"')
      return 0;
    fname = p + 1;
    if (p = (U_CHAR *) index (fname, '\"'))
      *p = '\0';
    for (ptr = all_include_files; ptr; ptr = ptr->next) {
      inc_fname = (U_CHAR *) rindex (ptr->fname, '/');
      inc_fname = inc_fname ? inc_fname + 1 : (U_CHAR *) ptr->fname;
      if (inc_fname && !strcmp (inc_fname, fname))
	warning ("`#pragma implementation' for `%s' appears after file is included",
		 fname);
    }
  }
  return 0;
}
static int
do_sccs ()
{
  if (pedantic)
    pedwarn ("ANSI C does not allow `#sccs'");
  return 0;
}
static int
do_if (buf, limit, op, keyword)
     U_CHAR *buf, *limit;
     FILE_BUF *op;
     struct directive *keyword;
{
  int value;
  FILE_BUF *ip = &instack[indepth];
  value = eval_if_expression (buf, limit - buf);
  conditional_skip (ip, value == 0, T_IF, ((char *)0));
  return 0;
}
static int
do_elif (buf, limit, op, keyword)
     U_CHAR *buf, *limit;
     FILE_BUF *op;
     struct directive *keyword;
{
  int value;
  FILE_BUF *ip = &instack[indepth];
  if (if_stack == instack[indepth].if_stack) {
    error ("`#elif' not within a conditional");
    return 0;
  } else {
    if (if_stack->type != T_IF && if_stack->type != T_ELIF) {
      error ("`#elif' after `#else'");
      fprintf ((&_iob[2]), " (matches line %d", if_stack->lineno);
      if (if_stack->fname != 0L && ip->fname != 0L &&
	  strcmp (if_stack->fname, ip->nominal_fname) != 0)
	fprintf ((&_iob[2]), ", file %s", if_stack->fname);
      fprintf ((&_iob[2]), ")\n");
    }
    if_stack->type = T_ELIF;
  }
  if (if_stack->if_succeeded)
    skip_if_group (ip, 0);
  else {
    value = eval_if_expression (buf, limit - buf);
    if (value == 0)
      skip_if_group (ip, 0);
    else {
      ++if_stack->if_succeeded;	
      output_line_command (ip, op, 1, same_file);
    }
  }
  return 0;
}
static int
eval_if_expression (buf, length)
     U_CHAR *buf;
     int length;
{
  FILE_BUF temp_obuf;
  HASHNODE *save_defined;
  int value;
  save_defined = install ("defined", -1, T_SPEC_DEFINED, 0, 0, -1);
  pcp_inside_if = 1;
  temp_obuf = expand_to_temp_buffer (buf, buf + length, 0, 1);
  pcp_inside_if = 0;
  delete_macro (save_defined);	
  value = parse_c_expression (temp_obuf.buf);
  free (temp_obuf.buf);
  return value;
}
static int
do_xifdef (buf, limit, op, keyword)
     U_CHAR *buf, *limit;
     FILE_BUF *op;
     struct directive *keyword;
{
  int skip;
  FILE_BUF *ip = &instack[indepth];
  U_CHAR *end; 
  int start_of_file = 0;
  U_CHAR *control_macro = 0;
  if (ip->fname != 0 && keyword->type == T_IFNDEF) {
    U_CHAR *p = ip->buf;
    while (p != directive_start) {
      char c = *p++;
      if (is_space[c])
	;
      else if (c == '/' && p != ip->bufp && *p == '*') {
	int junk;
	U_CHAR *save_bufp = ip->bufp;
	ip->bufp = p + 1;
	p = skip_to_end_of_comment (ip, &junk, 1);
	ip->bufp = save_bufp;
      } else {
	goto fail;
      }
    }
    start_of_file = 1;
  fail: ;
  }
  do { while (is_hor_space[*buf]) buf++; } while (0);
  while (limit != buf && is_hor_space[limit[-1]]) limit--;
  for (end = buf; is_idchar[*end]; end++);
  if (end == buf) {
    skip = (keyword->type == T_IFDEF);
    if (! traditional)
      pedwarn (end == limit ? "`#%s' with no argument"
	       : "`#%s' argument starts with punctuation",
	       keyword->name);
  } else {
    HASHNODE *hp;
    if (pedantic && buf[0] >= '0' && buf[0] <= '9')
      pedwarn ("`#%s' argument starts with a digit", keyword->name);
    else if (end != limit && !traditional)
      pedwarn ("garbage at end of `#%s' argument", keyword->name);
    hp = lookup (buf, end-buf, -1);
    if (pcp_outfile) {
      if (hp && hp->value.defn->predefined)
	fprintf(pcp_outfile, "#define %s\n", hp->name);
      else {
	U_CHAR *cp = buf;
	fprintf(pcp_outfile, "#undef ");
	while (is_idchar[*cp]) 
	  fputc (*cp++, pcp_outfile);
			(--(  pcp_outfile)->_cnt < 0 ? 			_flsbuf((int) ('\n'), (  pcp_outfile)) : 			(int) (*(  pcp_outfile)->_ptr++ = (unsigned char) ('\n')));
      }
    }
    skip = (hp == 0L) ^ (keyword->type == T_IFNDEF);
    if (start_of_file && !skip) {
      control_macro = (U_CHAR *) xmalloc (end - buf + 1);
      bcopy (buf, control_macro, end - buf);
      control_macro[end - buf] = 0;
    }
  }
  conditional_skip (ip, skip, T_IF, control_macro);
  return 0;
}
static void
conditional_skip (ip, skip, type, control_macro)
     FILE_BUF *ip;
     int skip;
     enum node_type type;
     U_CHAR *control_macro;
{
  IF_STACK_FRAME *temp;
  temp = (IF_STACK_FRAME *) xcalloc (1, sizeof (IF_STACK_FRAME));
  temp->fname = ip->nominal_fname;
  temp->lineno = ip->lineno;
  temp->next = if_stack;
  temp->control_macro = control_macro;
  if_stack = temp;
  if_stack->type = type;
  if (skip != 0) {
    skip_if_group (ip, 0);
    return;
  } else {
    ++if_stack->if_succeeded;
    output_line_command (ip, &outbuf, 1, same_file);
  }
}
static void
skip_if_group (ip, any)
     FILE_BUF *ip;
     int any;
{
  register U_CHAR *bp = ip->bufp, *cp;
  register U_CHAR *endb = ip->buf + ip->length;
  struct directive *kt;
  IF_STACK_FRAME *save_if_stack = if_stack; 
  U_CHAR *beg_of_line = bp;
  register int ident_length;
  U_CHAR *ident, *after_ident;
  while (bp < endb) {
    switch (*bp++) {
    case '/':			
      if (*bp == '\\' && bp[1] == '\n')
	newline_fix (bp);
      if (*bp == '*'
	  || (cplusplus_comments && *bp == '/')) {
	ip->bufp = ++bp;
	bp = skip_to_end_of_comment (ip, &ip->lineno, 0);
      }
      break;
    case '\"':
    case '\'':
      bp = skip_quoted_string (bp - 1, endb, ip->lineno, &ip->lineno,
			       ((char *)0), ((char *)0));
      break;
    case '\\':
      if (bp < endb) {
	if (*bp == '\n')
	  ++ip->lineno;		
	bp++;
      }
      break;
    case '\n':
      ++ip->lineno;
      beg_of_line = bp;
      break;
    case '#':
      ip->bufp = bp - 1;
      if (beg_of_line == 0)
	break;
      bp = beg_of_line;
      while (1) {
	if (is_hor_space[*bp])
	  bp++;
	else if (*bp == '\\' && bp[1] == '\n')
	  bp += 2;
	else if (*bp == '/' && bp[1] == '*') {
	  bp += 2;
	  while (!(*bp == '*' && bp[1] == '/'))
	    bp++;
	  bp += 2;
	} else if (cplusplus_comments && *bp == '/' && bp[1] == '/') {
	  bp += 2;
	  while (*bp++ != '\n') ;
        }
	else break;
      }
      if (bp != ip->bufp) {
	bp = ip->bufp + 1;	
	break;
      }
      bp = ip->bufp + 1;	
      while (1) {
	if (is_hor_space[*bp])
	  bp++;
	else if (*bp == '\\' && bp[1] == '\n')
	  bp += 2;
	else if (*bp == '/' && bp[1] == '*') {
	  bp += 2;
	  while (!(*bp == '*' && bp[1] == '/')) {
	    if (*bp == '\n')
	      ip->lineno++;
	    bp++;
	  }
	  bp += 2;
	} else if (cplusplus_comments && *bp == '/' && bp[1] == '/') {
	  bp += 2;
	  while (*bp++ != '\n') ;
        }
	else break;
      }
      cp = bp;
      while (1) {
	if (is_idchar[*bp])
	  bp++;
	else {
	  if (*bp == '\\' && bp[1] == '\n')
	    name_newline_fix (bp);
	  if (is_idchar[*bp])
	    bp++;
	  else break;
	}
      }
      ident_length = bp - cp;
      ident = cp;
      after_ident = bp;
      if (ident_length == 0 && *after_ident == '\n') {
	continue;
      }
      if (ident_length == 0 || !is_idstart[*ident]) {
	U_CHAR *p = ident;
	while (is_idchar[*p]) {
	  if (*p < '0' || *p > '9')
	    break;
	  p++;
	}
	if (p != ident && !is_idchar[*p]) {
	  if (pedantic)
	    pedwarn ("`#' followed by integer");
	  continue;
	}
	if (p == ident) {
	  while (*p == '#' || is_hor_space[*p]) p++;
	  if (*p == '\n') {
	    if (pedantic && !lang_asm)
	      pedwarn ("invalid preprocessor directive");
	    continue;
	  }
	}
	if (!lang_asm && pedantic)
	  pedwarn ("invalid preprocessor directive name");
	continue;
      }
      for (kt = directive_table; kt->length >= 0; kt++) {
	IF_STACK_FRAME *temp;
	if (ident_length == kt->length
	    && strncmp (cp, kt->name, kt->length) == 0) {
	  if (any)
	    return;
	  switch (kt->type) {
	  case T_IF:
	  case T_IFDEF:
	  case T_IFNDEF:
	    temp = (IF_STACK_FRAME *) xcalloc (1, sizeof (IF_STACK_FRAME));
	    temp->next = if_stack;
	    if_stack = temp;
	    temp->lineno = ip->lineno;
	    temp->fname = ip->nominal_fname;
	    temp->type = kt->type;
	    break;
	  case T_ELSE:
	  case T_ENDIF:
	    if (pedantic && if_stack != save_if_stack)
	      validate_else (bp);
	  case T_ELIF:
	    if (if_stack == instack[indepth].if_stack) {
	      error ("`#%s' not within a conditional", kt->name);
	      break;
	    }
	    else if (if_stack == save_if_stack)
	      return;		
	    if (kt->type != T_ENDIF) {
	      if (if_stack->type == T_ELSE)
		error ("`#else' or `#elif' after `#else'");
	      if_stack->type = kt->type;
	      break;
	    }
	    temp = if_stack;
	    if_stack = if_stack->next;
	    free (temp);
	    break;
	  }
	  break;
	}
      }
      if (kt->length < 0 && !lang_asm && pedantic)
	pedwarn ("invalid preprocessor directive name");
    }
  }
  ip->bufp = bp;
}
static int
do_else (buf, limit, op, keyword)
     U_CHAR *buf, *limit;
     FILE_BUF *op;
     struct directive *keyword;
{
  FILE_BUF *ip = &instack[indepth];
  if (pedantic) {
    do { while (is_hor_space[*buf]) buf++; } while (0);
    if (buf != limit)
      pedwarn ("text following `#else' violates ANSI standard");
  }
  if (if_stack == instack[indepth].if_stack) {
    error ("`#else' not within a conditional");
    return 0;
  } else {
    if_stack->control_macro = 0;
    if (if_stack->type != T_IF && if_stack->type != T_ELIF) {
      error ("`#else' after `#else'");
      fprintf ((&_iob[2]), " (matches line %d", if_stack->lineno);
      if (strcmp (if_stack->fname, ip->nominal_fname) != 0)
	fprintf ((&_iob[2]), ", file %s", if_stack->fname);
      fprintf ((&_iob[2]), ")\n");
    }
    if_stack->type = T_ELSE;
  }
  if (if_stack->if_succeeded)
    skip_if_group (ip, 0);
  else {
    ++if_stack->if_succeeded;	
    output_line_command (ip, op, 1, same_file);
  }
  return 0;
}
static int
do_endif (buf, limit, op, keyword)
     U_CHAR *buf, *limit;
     FILE_BUF *op;
     struct directive *keyword;
{
  if (pedantic) {
    do { while (is_hor_space[*buf]) buf++; } while (0);
    if (buf != limit)
      pedwarn ("text following `#endif' violates ANSI standard");
  }
  if (if_stack == instack[indepth].if_stack)
    error ("unbalanced `#endif'");
  else {
    IF_STACK_FRAME *temp = if_stack;
    if_stack = if_stack->next;
    if (temp->control_macro != 0) {
      FILE_BUF *ip = &instack[indepth];
      U_CHAR *p = ip->bufp;
      U_CHAR *ep = ip->buf + ip->length;
      while (p != ep) {
	U_CHAR c = *p++;
	switch (c) {
	case ' ':
	case '\t':
	case '\n':
	  break;
	case '/':
	  if (p != ep && *p == '*') {
	    int junk;
	    U_CHAR *save_bufp = ip->bufp;
	    ip->bufp = p + 1;
	    p = skip_to_end_of_comment (ip, &junk, 1);
	    ip->bufp = save_bufp;
	  }
	  break;
	default:
	  goto fail;
	}
      }
      if (indepth != 0)
	record_control_macro (ip->fname, temp->control_macro);
    fail: ;
    }
    free (temp);
    output_line_command (&instack[indepth], op, 1, same_file);
  }
  return 0;
}
static void
validate_else (p)
     register U_CHAR *p;
{
  while (1) {
    if (*p == '\\' && p[1] == '\n')
      p += 2;
    if (is_hor_space[*p])
      p++;
    else if (*p == '/') {
      if (p[1] == '\\' && p[2] == '\n')
	newline_fix (p + 1);
      if (p[1] == '*') {
	p += 2;
	while (*p) {
	  if (p[1] == '\\' && p[2] == '\n')
	    newline_fix (p + 1);
	  if (*p == '*' && p[1] == '/') {
	    p += 2;
	    break;
	  }
	  p++;
	}
      }
      else if (cplusplus_comments && p[1] == '/') {
	p += 2;
	while (*p && *p++ != '\n') ;
      }
    } else break;
  }
  if (*p && *p != '\n')
    pedwarn ("text following `#else' or `#endif' violates ANSI standard");
}
static U_CHAR *
skip_to_end_of_comment (ip, line_counter, nowarn)
     register FILE_BUF *ip;
     int *line_counter;		
     int nowarn;
{
  register U_CHAR *limit = ip->buf + ip->length;
  register U_CHAR *bp = ip->bufp;
  FILE_BUF *op = &outbuf;	
  int output = put_out_comments && !line_counter;
  if (output) {
    *op->bufp++ = '/';
    *op->bufp++ = '*';
  }
  if (cplusplus_comments && bp[-1] == '/') {
    if (output) {
      while (bp < limit)
	if ((*op->bufp++ = *bp++) == '\n') {
	  bp--;
	  break;
	}
      op->bufp[-1] = '*';
      *op->bufp++ = '/';
      *op->bufp++ = '\n';
    } else {
      while (bp < limit) {
	if (*bp++ == '\n') {
	  bp--;
	  break;
	}
      }
    }
    ip->bufp = bp;
    return bp;
  }
  while (bp < limit) {
    if (output)
      *op->bufp++ = *bp;
    switch (*bp++) {
    case '/':
      if (warn_comments && !nowarn && bp < limit && *bp == '*')
	warning ("`/*' within comment");
      break;
    case '\n':
      if (line_counter != 0L)
	++*line_counter;
      if (output)
	++op->lineno;
      break;
    case '*':
      if (*bp == '\\' && bp[1] == '\n')
	newline_fix (bp);
      if (*bp == '/') {
        if (output)
	  *op->bufp++ = '/';
	ip->bufp = ++bp;
	return bp;
      }
      break;
    }
  }
  ip->bufp = bp;
  return bp;
}
static U_CHAR *
skip_quoted_string (bp, limit, start_line, count_newlines, backslash_newlines_p, eofp)
     register U_CHAR *bp;
     register U_CHAR *limit;
     int start_line;
     int *count_newlines;
     int *backslash_newlines_p;
     int *eofp;
{
  register U_CHAR c, match;
  match = *bp++;
  while (1) {
    if (bp >= limit) {
      error_with_line (line_for_error (start_line),
		       "unterminated string or character constant");
      if (eofp)
	*eofp = 1;
      break;
    }
    c = *bp++;
    if (c == '\\') {
      while (*bp == '\\' && bp[1] == '\n') {
	if (backslash_newlines_p)
	  *backslash_newlines_p = 1;
	if (count_newlines)
	  ++*count_newlines;
	bp += 2;
      }
      if (*bp == '\n' && count_newlines) {
	if (backslash_newlines_p)
	  *backslash_newlines_p = 1;
	++*count_newlines;
      }
      bp++;
    } else if (c == '\n') {
      if (traditional) {
 	bp--;	
 	if (eofp)
 	  *eofp = 1;
 	break;
      }
      if (match == '\'') {
	error_with_line (line_for_error (start_line),
			 "unterminated character constant");
	bp--;
	if (eofp)
	  *eofp = 1;
	break;
      }
      if (traditional) {	
	if (eofp)
	  *eofp = 1;
	break;
      }
      if (count_newlines)
	++*count_newlines;
    } else if (c == match)
      break;
  }
  return bp;
}
static U_CHAR *
skip_paren_group (ip)
     register FILE_BUF *ip;
{
  U_CHAR *limit = ip->buf + ip->length;
  U_CHAR *p = ip->bufp;
  int depth = 0;
  int lines_dummy = 0;
  while (p != limit) {
    int c = *p++;
    switch (c) {
    case '(':
      depth++;
      break;
    case ')':
      depth--;
      if (depth == 0)
	return ip->bufp = p;
      break;
    case '/':
      if (*p == '*') {
	ip->bufp = p;
	p = skip_to_end_of_comment (ip, &lines_dummy, 0);
	p = ip->bufp;
      }
    case '"':
    case '\'':
      {
	int eofp = 0;
	p = skip_quoted_string (p - 1, limit, 0, ((char *)0), ((char *)0), &eofp);
	if (eofp)
	  return ip->bufp = p;
      }
      break;
    }
  }
  ip->bufp = p;
  return p;
}
static void
output_line_command (ip, op, conditional, file_change)
     FILE_BUF *ip, *op;
     int conditional;
     enum file_change_code file_change;
{
  int len;
  char *line_cmd_buf;
  if (no_line_commands
      || ip->fname == 0L
      || no_output) {
    op->lineno = ip->lineno;
    return;
  }
  if (conditional) {
    if (ip->lineno == op->lineno)
      return;
    if (ip->lineno > op->lineno && ip->lineno < op->lineno + 8) {
        (((op)->length - ((op)->bufp - (op)->buf) <= ( 10))      ? grow_outbuf ((op), ( 10)) : 0);
      while (ip->lineno > op->lineno) {
	*op->bufp++ = '\n';
	op->lineno++;
      }
      return;
    }
  }
  if (ip->lineno == 0 && ip->bufp - ip->buf < ip->length
      && *ip->bufp == '\n') {
    ip->lineno++;
    ip->bufp++;
  }
  line_cmd_buf = (char *) alloca (strlen (ip->nominal_fname) + 100);
  sprintf (line_cmd_buf, "# %d \"%s\"", ip->lineno, ip->nominal_fname);
  if (file_change != same_file)
    strcat (line_cmd_buf, file_change == enter_file ? " 1" : " 2");
  if (ip->system_header_p)
    strcat (line_cmd_buf, " 3");
  len = strlen (line_cmd_buf);
  line_cmd_buf[len++] = '\n';
    (((op)->length - ((op)->bufp - (op)->buf) <= ( len + 1))      ? grow_outbuf ((op), ( len + 1)) : 0);
  if (op->bufp > op->buf && op->bufp[-1] != '\n')
    *op->bufp++ = '\n';
  bcopy (line_cmd_buf, op->bufp, len);
  op->bufp += len;
  op->lineno = ip->lineno;
}
struct argdata {
  U_CHAR *raw, *expanded;
  int raw_length, expand_length;
  int stringified_length;
  U_CHAR *free1, *free2;
  char newlines;
  char comments;
  char use_count;
};
static void
macroexpand (hp, op)
     HASHNODE *hp;
     FILE_BUF *op;
{
  int nargs;
  DEFINITION *defn = hp->value.defn;
  register U_CHAR *xbuf;
  int xbuf_len;
  int start_line = instack[indepth].lineno;
  int rest_args, rest_zero;
    if (indepth >= (200 - 1))					    {									      error_with_line (line_for_error (instack[indepth].lineno),			       "macro or `#include' recursion too deep");	      return;;								    };
  if (hp->type != T_MACRO) {
    special_symbol (hp, op);
    return;
  }
  if (pcp_inside_if && pcp_outfile && defn->predefined)
    dump_single_macro (hp, pcp_outfile);
  nargs = defn->nargs;
  if (nargs >= 0) {
    register int i;
    struct argdata *args;
    char *parse_error = 0;
    args = (struct argdata *) alloca ((nargs + 1) * sizeof (struct argdata));
    for (i = 0; i < nargs; i++) {
      args[i].raw = args[i].expanded = (U_CHAR *) "";
      args[i].raw_length = args[i].expand_length
	= args[i].stringified_length = 0;
      args[i].free1 = args[i].free2 = 0;
      args[i].use_count = 0;
    }
    i = 0;
    rest_args = 0;
    do {
      ++instack[indepth].bufp;
      if (rest_args)
	continue;
      if (i < nargs || (nargs == 0 && i == 0)) {
	if (i == nargs - 1 && defn->rest_args)
	  rest_args = 1;
	parse_error = macarg (&args[i], rest_args);
      }
      else
	parse_error = macarg (((char *)0), 0);
      if (parse_error) {
	error_with_line (line_for_error (start_line), parse_error);
	break;
      }
      i++;
    } while (*instack[indepth].bufp != ')');
    if (i == 1) {
      register U_CHAR *bp = args[0].raw;
      register U_CHAR *lim = bp + args[0].raw_length;
      while (bp != lim && is_space[*bp]) bp++;
      if (bp == lim)
	i = 0;
    }
    rest_zero = 0;
    if (nargs == 0 && i > 0) {
      if (! parse_error)
	error ("arguments given to macro `%s'", hp->name);
    } else if (i < nargs) {
      if (nargs == 1 && i == 0 && traditional)
	;
      else if (i == nargs - 1 && defn->rest_args)
	rest_zero = 1;
      else if (parse_error)
	;
      else if (i == 0)
	error ("macro `%s' used without args", hp->name);
      else if (i == 1)
	error ("macro `%s' used with just one arg", hp->name);
      else
	error ("macro `%s' used with only %d args", hp->name, i);
    } else if (i > nargs) {
      if (! parse_error)
	error ("macro `%s' used with too many (%d) args", hp->name, i);
    }
    ++instack[indepth].bufp;
    if (nargs == 0) {
      xbuf = defn->expansion;
      xbuf_len = defn->length;
    } else {
      register U_CHAR *exp = defn->expansion;
      register int offset;	
      register int totlen;	
      register struct reflist *ap, *last_ap;
      xbuf_len = defn->length;
      for (ap = defn->pattern; ap != 0L; ap = ap->next) {
	if (ap->stringify)
	  xbuf_len += args[ap->argno].stringified_length;
	else if (ap->raw_before || ap->raw_after || traditional)
	  xbuf_len += args[ap->argno].raw_length;
	else
	  xbuf_len += args[ap->argno].expand_length;
	if (args[ap->argno].use_count < 10)
	  args[ap->argno].use_count++;
      }
      xbuf = (U_CHAR *) xmalloc (xbuf_len + 1);
      offset = totlen = 0;
      for (last_ap = 0L, ap = defn->pattern; ap != 0L;
	   last_ap = ap, ap = ap->next) {
	register struct argdata *arg = &args[ap->argno];
	for (i = 0; i < ap->nchars; i++, offset++)
	  if (! (rest_zero && ((ap->rest_args && ap->raw_before)
			       || (last_ap != 0L && last_ap->rest_args
				   && last_ap->raw_after))))
	    xbuf[totlen++] = exp[offset];
	if (ap->stringify != 0) {
	  int arglen = arg->raw_length;
	  int escaped = 0;
	  int in_string = 0;
	  int c;
	  i = 0;
	  while (i < arglen
		 && (c = arg->raw[i], is_space[c]))
	    i++;
	  while (i < arglen
		 && (c = arg->raw[arglen - 1], is_space[c]))
	    arglen--;
	  if (!traditional)
	    xbuf[totlen++] = '\"'; 
	  for (; i < arglen; i++) {
	    c = arg->raw[i];
	    if (c == '\n' && arg->raw[i+1] != '\n') {
	      i++;
	      continue;
	    }
	    if (! in_string
		&& (c == '\n' ? arg->raw[i+1] == '\n' : is_space[c])) {
	      while (1) {
		if (c == '\n' && is_space[arg->raw[i+1]])
		  i += 2;
		else if (c != '\n' && is_space[c])
		  i++;
		else break;
		c = arg->raw[i];
	      }
	      i--;
	      c = ' ';
	    }
	    if (escaped)
	      escaped = 0;
	    else {
	      if (c == '\\')
		escaped = 1;
	      if (in_string) {
		if (c == in_string)
		  in_string = 0;
	      } else if (c == '\"' || c == '\'')
		in_string = c;
	    }
	    if (c == '\"' || (in_string && c == '\\'))
	      xbuf[totlen++] = '\\';
	    if ((((*(__lc_ctype->core.iswctype)) == 0L) 		? (int) (__lc_ctype->_mask[c] & (0x080)) 		: (*(__lc_ctype->core.iswctype)) (c,0x080,__lc_ctype)))
	      xbuf[totlen++] = c;
	    else {
	      sprintf ((char *) &xbuf[totlen], "\\%03o", (unsigned int) c);
	      totlen += 4;
	    }
	  }
	  if (!traditional)
	    xbuf[totlen++] = '\"'; 
	} else if (ap->raw_before || ap->raw_after || traditional) {
	  U_CHAR *p1 = arg->raw;
	  U_CHAR *l1 = p1 + arg->raw_length;
	  if (ap->raw_before) {
	    while (p1 != l1 && is_space[*p1]) p1++;
	    while (p1 != l1 && is_idchar[*p1])
	      xbuf[totlen++] = *p1++;
	    if (p1[0] == '\n' && p1[1] == '-')
	      p1 += 2;
	  }
	  if (ap->raw_after) {
	    while (p1 != l1) {
	      if (is_space[l1[-1]]) l1--;
	      else if (l1[-1] == '-') {
		U_CHAR *p2 = l1 - 1;
		while (p2 != p1 && p2[-1] == '\n') p2--;
		if ((l1 - 1 - p2) & 1) {
		  l1 -= 2;
		}
		else break;
	      }
	      else break;
	    }
	  }
	  bcopy (p1, xbuf + totlen, l1 - p1);
	  totlen += l1 - p1;
	} else {
	  bcopy (arg->expanded, xbuf + totlen, arg->expand_length);
	  totlen += arg->expand_length;
	  if (arg->use_count > 1 && arg->newlines > 0) {
	    arg->use_count = 1;
	    arg->expand_length
	      = delete_newlines (arg->expanded, arg->expand_length);
	  }
	}
	if (totlen > xbuf_len)
	  abort ();
      }
      for (i = offset; i < defn->length; i++) {
	if (exp[i] == ')')
	  rest_zero = 0;
	if (! (rest_zero && last_ap != 0L && last_ap->rest_args
	       && last_ap->raw_after))
	  xbuf[totlen++] = exp[i];
      }
      xbuf[totlen] = 0;
      xbuf_len = totlen;
      for (i = 0; i < nargs; i++) {
	if (args[i].free1 != 0)
	  free (args[i].free1);
	if (args[i].free2 != 0)
	  free (args[i].free2);
      }
    }
  } else {
    xbuf = defn->expansion;
    xbuf_len = defn->length;
  }
  {
    register FILE_BUF *ip2;
    ip2 = &instack[++indepth];
    ip2->fname = 0;
    ip2->nominal_fname = 0;
    ip2->lineno = 0;
    ip2->buf = xbuf;
    ip2->length = xbuf_len;
    ip2->bufp = xbuf;
    ip2->free_ptr = (nargs > 0) ? xbuf : 0;
    ip2->macro = hp;
    ip2->if_stack = if_stack;
    ip2->system_header_p = 0;
    if (!traditional)
      hp->type = T_DISABLED;
  }
}
static char *
macarg (argptr, rest_args)
     register struct argdata *argptr;
     int rest_args;
{
  FILE_BUF *ip = &instack[indepth];
  int paren = 0;
  int newlines = 0;
  int comments = 0;
  U_CHAR *bp = macarg1 (ip->bufp, ip->buf + ip->length,
			&paren, &newlines, &comments, rest_args);
  if (!(ip->fname != 0 && (newlines != 0 || comments != 0))
      && bp != ip->buf + ip->length) {
    if (argptr != 0) {
      argptr->raw = ip->bufp;
      argptr->raw_length = bp - ip->bufp;
      argptr->newlines = newlines;
    }
    ip->bufp = bp;
  } else {
    int bufsize = bp - ip->bufp;
    int extra = newlines;
    U_CHAR *buffer = (U_CHAR *) xmalloc (bufsize + extra + 1);
    int final_start = 0;
    bcopy (ip->bufp, buffer, bufsize);
    ip->bufp = bp;
    ip->lineno += newlines;
    while (bp == ip->buf + ip->length) {
      if (instack[indepth].macro == 0) {
	free (buffer);
	return "unterminated macro call";
      }
      ip->macro->type = T_MACRO;
      if (ip->free_ptr)
	free (ip->free_ptr);
      ip = &instack[--indepth];
      newlines = 0;
      comments = 0;
      bp = macarg1 (ip->bufp, ip->buf + ip->length, &paren,
		    &newlines, &comments, rest_args);
      final_start = bufsize;
      bufsize += bp - ip->bufp;
      extra += newlines;
      buffer = (U_CHAR *) xrealloc (buffer, bufsize + extra + 1);
      bcopy (ip->bufp, buffer + bufsize - (bp - ip->bufp), bp - ip->bufp);
      ip->bufp = bp;
      ip->lineno += newlines;
    }
    if (argptr != 0) {
      argptr->raw = buffer;
      argptr->raw_length = bufsize;
      argptr->free1 = buffer;
      argptr->newlines = newlines;
      argptr->comments = comments;
      if ((newlines || comments) && ip->fname != 0)
	argptr->raw_length
	  = final_start +
	    discard_comments (argptr->raw + final_start,
			      argptr->raw_length - final_start,
			      newlines);
      argptr->raw[argptr->raw_length] = 0;
      if (argptr->raw_length > bufsize + extra)
	abort ();
    }
  }
  if (argptr != 0) {
    FILE_BUF obuf;
    register U_CHAR *buf, *lim;
    register int totlen;
    obuf = expand_to_temp_buffer (argptr->raw,
				  argptr->raw + argptr->raw_length,
				  1, 0);
    argptr->expanded = obuf.buf;
    argptr->expand_length = obuf.length;
    argptr->free2 = obuf.buf;
    buf = argptr->raw;
    lim = buf + argptr->raw_length;
    while (buf != lim && is_space[*buf])
      buf++;
    while (buf != lim && is_space[lim[-1]])
      lim--;
    totlen = traditional ? 0 : 2;	
    while (buf != lim) {
      register U_CHAR c = *buf++;
      totlen++;
      if (c == '\"' || c == '\\') 
	totlen++;
      else if (!(((*(__lc_ctype->core.iswctype)) == 0L) 		? (int) (__lc_ctype->_mask[c] & (0x080)) 		: (*(__lc_ctype->core.iswctype)) (c,0x080,__lc_ctype)))
	totlen += 3;
    }
    argptr->stringified_length = totlen;
  }
  return 0;
}
static U_CHAR *
macarg1 (start, limit, depthptr, newlines, comments, rest_args)
     U_CHAR *start;
     register U_CHAR *limit;
     int *depthptr, *newlines, *comments;
     int rest_args;
{
  register U_CHAR *bp = start;
  while (bp < limit) {
    switch (*bp) {
    case '(':
      (*depthptr)++;
      break;
    case ')':
      if (--(*depthptr) < 0)
	return bp;
      break;
    case '\\':
      if (bp + 1 < limit && traditional)
	{
	  bp++;
	  if (*bp == '\n')
	    ++*newlines;
	}
      break;
    case '\n':
      ++*newlines;
      break;
    case '/':
      if (bp[1] == '\\' && bp[2] == '\n')
	newline_fix (bp + 1);
      if (cplusplus_comments && bp[1] == '/') {
	*comments = 1;
	bp += 2;
	while (bp < limit && *bp++ != '\n') ;
	++*newlines;
	break;
      }
      if (bp[1] != '*' || bp + 1 >= limit)
	break;
      *comments = 1;
      bp += 2;
      while (bp + 1 < limit) {
	if (bp[0] == '*'
	    && bp[1] == '\\' && bp[2] == '\n')
	  newline_fix (bp + 1);
	if (bp[0] == '*' && bp[1] == '/')
	  break;
	if (*bp == '\n') ++*newlines;
	bp++;
      }
      break;
    case '\'':
    case '\"':
      {
	int quotec;
	for (quotec = *bp++; bp + 1 < limit && *bp != quotec; bp++) {
	  if (*bp == '\\') {
	    bp++;
	    if (*bp == '\n')
	      ++*newlines;
	    while (*bp == '\\' && bp[1] == '\n') {
	      bp += 2;
	    }
	  } else if (*bp == '\n') {
	    ++*newlines;
	    if (quotec == '\'')
	      break;
	  }
	}
      }
      break;
    case ',':
      if ((*depthptr) == 0 && rest_args == 0)
	return bp;
      break;
    }
    bp++;
  }
  return bp;
}
static int
discard_comments (start, length, newlines)
     U_CHAR *start;
     int length;
     int newlines;
{
  register U_CHAR *ibp;
  register U_CHAR *obp;
  register U_CHAR *limit;
  register int c;
  if (newlines > 0) {
    ibp = start + length;
    obp = ibp + newlines;
    limit = start;
    while (limit != ibp)
      *--obp = *--ibp;
  }
  ibp = start + newlines;
  limit = start + length + newlines;
  obp = start;
  while (ibp < limit) {
    *obp++ = c = *ibp++;
    switch (c) {
    case '\n':
      *obp++ = '\n';
      break;
    case '\\':
      if (*ibp == '\n') {
	obp--;
	ibp++;
      }
      break;
    case '/':
      if (*ibp == '\\' && ibp[1] == '\n')
	newline_fix (ibp);
      if (cplusplus_comments && ibp[0] == '/') {
	obp--;
	ibp++;
	while (ibp < limit && *ibp++ != '\n') ;
	break;
      }
      if (ibp[0] != '*' || ibp + 1 >= limit)
	break;
      obp--;
      ibp++;
      while (ibp + 1 < limit) {
	if (ibp[0] == '*'
	    && ibp[1] == '\\' && ibp[2] == '\n')
	  newline_fix (ibp + 1);
	if (ibp[0] == '*' && ibp[1] == '/')
	  break;
	ibp++;
      }
      ibp += 2;
      break;
    case '\'':
    case '\"':
      {
	int quotec = c;
	while (ibp < limit) {
	  *obp++ = c = *ibp++;
	  if (c == quotec)
	    break;
	  if (c == '\n' && quotec == '\'')
	    break;
	  if (c == '\\' && ibp < limit) {
	    while (*ibp == '\\' && ibp[1] == '\n')
	      ibp += 2;
	    *obp++ = *ibp++;
	  }
	}
      }
      break;
    }
  }
  return obp - start;
}
static int
delete_newlines (start, length)
     U_CHAR *start;
     int length;
{
  register U_CHAR *ibp;
  register U_CHAR *obp;
  register U_CHAR *limit;
  register int c;
  ibp = start;
  limit = start + length;
  obp = start;
  while (ibp < limit) {
    *obp++ = c = *ibp++;
    switch (c) {
    case '\n':
      if (*ibp == '\n')
	{
	  ibp++;
	  obp--;
	}
      break;
    case '\'':
    case '\"':
      {
	int quotec = c;
	while (ibp < limit) {
	  *obp++ = c = *ibp++;
	  if (c == quotec)
	    break;
	  if (c == '\n' && quotec == '\'')
	    break;
	}
      }
      break;
    }
  }
  return obp - start;
}
void
error (msg, arg1, arg2, arg3)
     char *msg;
     char *arg1, *arg2, *arg3;
{
  int i;
  FILE_BUF *ip = 0L;
  print_containing_files ();
  for (i = indepth; i >= 0; i--)
    if (instack[i].fname != 0L) {
      ip = &instack[i];
      break;
    }
  if (ip != 0L)
    fprintf ((&_iob[2]), "%s:%d: ", ip->nominal_fname, ip->lineno);
  fprintf ((&_iob[2]), msg, arg1, arg2, arg3);
  fprintf ((&_iob[2]), "\n");
  errors++;
}
static void
error_from_errno (name)
     char *name;
{
  int i;
  FILE_BUF *ip = 0L;
  print_containing_files ();
  for (i = indepth; i >= 0; i--)
    if (instack[i].fname != 0L) {
      ip = &instack[i];
      break;
    }
  if (ip != 0L)
    fprintf ((&_iob[2]), "%s:%d: ", ip->nominal_fname, ip->lineno);
  if (errno < sys_nerr)
    fprintf ((&_iob[2]), "%s: %s\n", name, sys_errlist[errno]);
  else
    fprintf ((&_iob[2]), "%s: undocumented I/O error\n", name);
  errors++;
}
void
warning (msg, arg1, arg2, arg3)
     char *msg;
     char *arg1, *arg2, *arg3;
{
  int i;
  FILE_BUF *ip = 0L;
  if (inhibit_warnings)
    return;
  if (warnings_are_errors)
    errors++;
  print_containing_files ();
  for (i = indepth; i >= 0; i--)
    if (instack[i].fname != 0L) {
      ip = &instack[i];
      break;
    }
  if (ip != 0L)
    fprintf ((&_iob[2]), "%s:%d: ", ip->nominal_fname, ip->lineno);
  fprintf ((&_iob[2]), "warning: ");
  fprintf ((&_iob[2]), msg, arg1, arg2, arg3);
  fprintf ((&_iob[2]), "\n");
}
static void
error_with_line (line, msg, arg1, arg2, arg3)
     int line;
     char *msg;
     char *arg1, *arg2, *arg3;
{
  int i;
  FILE_BUF *ip = 0L;
  print_containing_files ();
  for (i = indepth; i >= 0; i--)
    if (instack[i].fname != 0L) {
      ip = &instack[i];
      break;
    }
  if (ip != 0L)
    fprintf ((&_iob[2]), "%s:%d: ", ip->nominal_fname, line);
  fprintf ((&_iob[2]), msg, arg1, arg2, arg3);
  fprintf ((&_iob[2]), "\n");
  errors++;
}
void
pedwarn (msg, arg1, arg2, arg3)
     char *msg;
     char *arg1, *arg2, *arg3;
{
  if (pedantic_errors)
    error (msg, arg1, arg2, arg3);
  else
    warning (msg, arg1, arg2, arg3);
}
static void
pedwarn_with_file_and_line (file, line, msg, arg1, arg2, arg3)
     char *file;
     int line;
     char *msg;
     char *arg1, *arg2, *arg3;
{
  int i;
  if (!pedantic_errors && inhibit_warnings)
    return;
  if (file != 0L)
    fprintf ((&_iob[2]), "%s:%d: ", file, line);
  if (pedantic_errors || warnings_are_errors)
    errors++;
  if (!pedantic_errors)
    fprintf ((&_iob[2]), "warning: ");
  fprintf ((&_iob[2]), msg, arg1, arg2, arg3);
  fprintf ((&_iob[2]), "\n");
}
static void
print_containing_files ()
{
  FILE_BUF *ip = 0L;
  int i;
  int first = 1;
  if (last_error_tick == input_file_stack_tick)
    return;
  for (i = indepth; i >= 0; i--)
    if (instack[i].fname != 0L) {
      ip = &instack[i];
      break;
    }
  if (ip == 0L)
    return;
  for (i--; i >= 0; i--)
    if (instack[i].fname != 0L) {
      ip = &instack[i];
      if (first) {
	first = 0;
	fprintf ((&_iob[2]), "In file included");
      } else {
	fprintf ((&_iob[2]), ",");
      }
      fprintf ((&_iob[2]), " from %s:%d", ip->nominal_fname, ip->lineno);
    }
  if (! first)
    fprintf ((&_iob[2]), ":\n");
  last_error_tick = input_file_stack_tick;
}
static int
line_for_error (line)
     int line;
{
  int i;
  int line1 = line;
  for (i = indepth; i >= 0; ) {
    if (instack[i].fname != 0)
      return line1;
    i--;
    if (i < 0)
      return 0;
    line1 = instack[i].lineno;
  }
  abort ();
  return 0;
}
static int
grow_outbuf (obuf, needed)
     register FILE_BUF *obuf;
     register int needed;
{
  register U_CHAR *p;
  int minsize;
  if (obuf->length - (obuf->bufp - obuf->buf) > needed)
    return 0;
  obuf->length *= 2;
  minsize = (3 * needed) / 2 + (obuf->bufp - obuf->buf);
  if (minsize > obuf->length)
    obuf->length = minsize;
  if ((p = (U_CHAR *) xrealloc (obuf->buf, obuf->length)) == 0L)
    memory_full ();
  obuf->bufp = p + (obuf->bufp - obuf->buf);
  obuf->buf = p;
  return 0;
}
static HASHNODE *
install (name, len, type, ivalue, value, hash)
     U_CHAR *name;
     int len;
     enum node_type type;
     int ivalue;
     char *value;
     int hash;
{
  register HASHNODE *hp;
  register int i, bucket;
  register U_CHAR *p, *q;
  if (len < 0) {
    p = name;
    while (is_idchar[*p])
      p++;
    len = p - name;
  }
  if (hash < 0)
    hash = hashf (name, len, 1403);
  i = sizeof (HASHNODE) + len + 1;
  hp = (HASHNODE *) xmalloc (i);
  bucket = hash;
  hp->bucket_hdr = &hashtab[bucket];
  hp->next = hashtab[bucket];
  hashtab[bucket] = hp;
  hp->prev = 0L;
  if (hp->next != 0L)
    hp->next->prev = hp;
  hp->type = type;
  hp->length = len;
  if (hp->type == T_CONST)
    hp->value.ival = ivalue;
  else
    hp->value.cpval = value;
  hp->name = ((U_CHAR *) hp) + sizeof (HASHNODE);
  p = hp->name;
  q = name;
  for (i = 0; i < len; i++)
    *p++ = *q++;
  hp->name[len] = 0;
  return hp;
}
HASHNODE *
lookup (name, len, hash)
     U_CHAR *name;
     int len;
     int hash;
{
  register U_CHAR *bp;
  register HASHNODE *bucket;
  if (len < 0) {
    for (bp = name; is_idchar[*bp]; bp++) ;
    len = bp - name;
  }
  if (hash < 0)
    hash = hashf (name, len, 1403);
  bucket = hashtab[hash];
  while (bucket) {
    if (bucket->length == len && strncmp (bucket->name, name, len) == 0)
      return bucket;
    bucket = bucket->next;
  }
  return 0L;
}
static void
delete_macro (hp)
     HASHNODE *hp;
{
  if (hp->prev != 0L)
    hp->prev->next = hp->next;
  if (hp->next != 0L)
    hp->next->prev = hp->prev;
  if (hp == *hp->bucket_hdr)
    *hp->bucket_hdr = hp->next;
  free (hp);
}
static int
hashf (name, len, hashsize)
     register U_CHAR *name;
     register int len;
     int hashsize;
{
  register int r = 0;
  while (len--)
    r = ((r << 2) +  *name++);
  return (r & 0x7fffffff)  % hashsize;
}
static void
dump_single_macro (hp, of)
     register HASHNODE *hp;
     FILE *of;
{
  register DEFINITION *defn = hp->value.defn;
  struct reflist *ap;
  int offset;
  int concat;
  fprintf (of, "#define %s", hp->name);
  if (defn->nargs >= 0) {
    int i;
    fprintf (of, "(");
    for (i = 0; i < defn->nargs; i++) {
      dump_arg_n (defn, i, of);
      if (i + 1 < defn->nargs)
	fprintf (of, ", ");
    }
    fprintf (of, ")");
  }
  fprintf (of, " ");
  offset = 0;
  concat = 0;
  for (ap = defn->pattern; ap != 0L; ap = ap->next) {
    dump_defn_1 (defn->expansion, offset, ap->nchars, of);
    if (ap->nchars != 0)
      concat = 0;
    offset += ap->nchars;
    if (ap->stringify)
      fprintf (of, " #");
    if (ap->raw_before && !concat)
      fprintf (of, " ## ");
    concat = 0;
    dump_arg_n (defn, ap->argno, of);
    if (ap->raw_after) {
      fprintf (of, " ## ");
      concat = 1;
    }
  }
  dump_defn_1 (defn->expansion, offset, defn->length - offset, of);
  fprintf (of, "\n");
}
static void
dump_all_macros ()
{
  int bucket;
  for (bucket = 0; bucket < 1403; bucket++) {
    register HASHNODE *hp;
    for (hp = hashtab[bucket]; hp; hp= hp->next) {
      if (hp->type == T_MACRO)
	dump_single_macro (hp, (&_iob[1]));
    }
  }
}
static void
dump_defn_1 (base, start, length, of)
     U_CHAR *base;
     int start;
     int length;
     FILE *of;
{
  U_CHAR *p = base + start;
  U_CHAR *limit = base + start + length;
  while (p < limit) {
    if (*p != '\n')
      		(--(  of)->_cnt < 0 ? 			_flsbuf((int) (*p), (  of)) : 			(int) (*(  of)->_ptr++ = (unsigned char) (*p)));
    else if (*p == '\"' || *p =='\'') {
      U_CHAR *p1 = skip_quoted_string (p, limit, 0, ((char *)0),
				       ((char *)0), ((char *)0));
      fwrite (p, p1 - p, 1, of);
      p = p1 - 1;
    }
    p++;
  }
}
static void
dump_arg_n (defn, argnum, of)
     DEFINITION *defn;
     int argnum;
     FILE *of;
{
  register U_CHAR *p = defn->args.argnames;
  while (argnum + 1 < defn->nargs) {
    p = (U_CHAR *) index (p, ' ') + 1;
    argnum++;
  }
  while (*p && *p != ',') {
    		(--(  of)->_cnt < 0 ? 			_flsbuf((int) (*p), (  of)) : 			(int) (*(  of)->_ptr++ = (unsigned char) (*p)));
    p++;
  }
}
static void
initialize_char_syntax ()
{
  register int i;
  for (i = 'a'; i <= 'z'; i++) {
    is_idchar[i - 'a' + 'A'] = 1;
    is_idchar[i] = 1;
    is_idstart[i - 'a' + 'A'] = 1;
    is_idstart[i] = 1;
  }
  for (i = '0'; i <= '9'; i++)
    is_idchar[i] = 1;
  is_idchar['_'] = 1;
  is_idstart['_'] = 1;
  is_idchar['$'] = dollars_in_ident;
  is_idstart['$'] = dollars_in_ident;
  is_hor_space[' '] = 1;
  is_hor_space['\t'] = 1;
  is_hor_space['\v'] = 1;
  is_hor_space['\f'] = 1;
  is_hor_space['\r'] = 1;
  is_space[' '] = 1;
  is_space['\t'] = 1;
  is_space['\v'] = 1;
  is_space['\f'] = 1;
  is_space['\n'] = 1;
  is_space['\r'] = 1;
}
static void
initialize_builtins (inp, outp)
     FILE_BUF *inp;
     FILE_BUF *outp;
{
  install ("__LINE__", -1, T_SPECLINE, 0, 0, -1);
  install ("__DATE__", -1, T_DATE, 0, 0, -1);
  install ("__FILE__", -1, T_FILE, 0, 0, -1);
  install ("__BASE_FILE__", -1, T_BASE_FILE, 0, 0, -1);
  install ("__INCLUDE_LEVEL__", -1, T_INCLUDE_LEVEL, 0, 0, -1);
  install ("__VERSION__", -1, T_VERSION, 0, 0, -1);
  install ("__SIZE_TYPE__", -1, T_SIZE_TYPE, 0, 0, -1);
  install ("__PTRDIFF_TYPE__ ", -1, T_PTRDIFF_TYPE, 0, 0, -1);
  install ("__WCHAR_TYPE__", -1, T_WCHAR_TYPE, 0, 0, -1);
  install ("__TIME__", -1, T_TIME, 0, 0, -1);
  if (!traditional)
    install ("__STDC__", -1, T_CONST, 1, 0, -1);
  if (objc)
    install ("__OBJC__", -1, T_CONST, 1, 0, -1);
  if (debug_output)
    {
      char directive[2048];
      register struct directive *dp = &directive_table[0];
      struct tm *timebuf = timestamp ();
      sprintf (directive, " __BASE_FILE__ \"%s\"\n",
	       instack[0].nominal_fname);
      output_line_command (inp, outp, 0, same_file);
      pass_thru_directive (directive, &directive[strlen (directive)], outp, dp);
      sprintf (directive, " __VERSION__ \"%s\"\n", version_string);
      output_line_command (inp, outp, 0, same_file);
      pass_thru_directive (directive, &directive[strlen (directive)], outp, dp);
      sprintf (directive, " __SIZE_TYPE__ %s\n", "long unsigned int");
      output_line_command (inp, outp, 0, same_file);
      pass_thru_directive (directive, &directive[strlen (directive)], outp, dp);
      sprintf (directive, " __PTRDIFF_TYPE__ %s\n", "long int");
      output_line_command (inp, outp, 0, same_file);
      pass_thru_directive (directive, &directive[strlen (directive)], outp, dp);
      sprintf (directive, " __WCHAR_TYPE__ %s\n", "short unsigned int");
      output_line_command (inp, outp, 0, same_file);
      pass_thru_directive (directive, &directive[strlen (directive)], outp, dp);
      sprintf (directive, " __DATE__ \"%s %2d %4d\"\n",
	       monthnames[timebuf->tm_mon],
	       timebuf->tm_mday, timebuf->tm_year + 1900);
      output_line_command (inp, outp, 0, same_file);
      pass_thru_directive (directive, &directive[strlen (directive)], outp, dp);
      sprintf (directive, " __TIME__ \"%02d:%02d:%02d\"\n",
	       timebuf->tm_hour, timebuf->tm_min, timebuf->tm_sec);
      output_line_command (inp, outp, 0, same_file);
      pass_thru_directive (directive, &directive[strlen (directive)], outp, dp);
      if (!traditional)
	{
          sprintf (directive, " __STDC__ 1");
          output_line_command (inp, outp, 0, same_file);
          pass_thru_directive (directive, &directive[strlen (directive)],
			       outp, dp);
	}
      if (objc)
	{
          sprintf (directive, " __OBJC__ 1");
          output_line_command (inp, outp, 0, same_file);
          pass_thru_directive (directive, &directive[strlen (directive)],
			       outp, dp);
	}
    }
}
static void
make_definition (str, op)
     U_CHAR *str;
     FILE_BUF *op;
{
  FILE_BUF *ip;
  struct directive *kt;
  U_CHAR *buf, *p;
  buf = str;
  p = str;
  if (!is_idstart[*p]) {
    error ("malformed option `-D %s'", str);
    return;
  }
  while (is_idchar[*++p])
    ;
  if (*p == 0) {
    buf = (U_CHAR *) alloca (p - buf + 4);
    strcpy ((char *)buf, str);
    strcat ((char *)buf, " 1");
  } else if (*p != '=') {
    error ("malformed option `-D %s'", str);
    return;
  } else {
    U_CHAR *q;
    buf = (U_CHAR *) alloca (2 * strlen (str) + 1);
    strncpy (buf, str, p - str);
    buf[p - str] = ' ';
    p++;
    q = &buf[p - str];
    while (*p) {
      if (*p == '\\' && p[1] == '\n')
	p += 2;
      else if (*p == '\n')
	{
	  *q++ = '\n';
	  *q++ = '\n';
	  p++;
	}
      else
	*q++ = *p++;
    }
    *q = 0;
  }
  ip = &instack[++indepth];
  ip->nominal_fname = ip->fname = "*Initialization*";
  ip->buf = ip->bufp = buf;
  ip->length = strlen (buf);
  ip->lineno = 1;
  ip->macro = 0;
  ip->free_ptr = 0;
  ip->if_stack = if_stack;
  ip->system_header_p = 0;
  for (kt = directive_table; kt->type != T_DEFINE; kt++)
    ;
  do_define (buf, buf + strlen (buf) , op, kt);
  --indepth;
}
static void
make_undef (str, op)
     U_CHAR *str;
     FILE_BUF *op;
{
  FILE_BUF *ip;
  struct directive *kt;
  ip = &instack[++indepth];
  ip->nominal_fname = ip->fname = "*undef*";
  ip->buf = ip->bufp = str;
  ip->length = strlen (str);
  ip->lineno = 1;
  ip->macro = 0;
  ip->free_ptr = 0;
  ip->if_stack = if_stack;
  ip->system_header_p = 0;
  for (kt = directive_table; kt->type != T_UNDEF; kt++)
    ;
  do_undef (str, str + strlen (str), op, kt);
  --indepth;
}
static void
make_assertion (option, str)
     char *option;
     U_CHAR *str;
{
  FILE_BUF *ip;
  struct directive *kt;
  U_CHAR *buf, *p, *q;
  buf = (U_CHAR *) alloca (strlen (str) + 1);
  strcpy ((char *) buf, str);
  p = q = buf;
  while (*p) {
    if (*p == '\\' && p[1] == '\n')
      p += 2;
    else
      *q++ = *p++;
  }
  *q = 0;
  p = buf;
  if (!is_idstart[*p]) {
    error ("malformed option `%s %s'", option, str);
    return;
  }
  while (is_idchar[*++p])
    ;
  while (*p == ' ' || *p == '\t') p++;
  if (! (*p == 0 || *p == '(')) {
    error ("malformed option `%s %s'", option, str);
    return;
  }
  ip = &instack[++indepth];
  ip->nominal_fname = ip->fname = "*Initialization*";
  ip->buf = ip->bufp = buf;
  ip->length = strlen (buf);
  ip->lineno = 1;
  ip->macro = 0;
  ip->free_ptr = 0;
  ip->if_stack = if_stack;
  ip->system_header_p = 0;
  for (kt = directive_table; kt->type != T_ASSERT; kt++)
    ;
  do_assert (buf, buf + strlen (buf) , ((char *)0), kt);
  --indepth;
}
static void
append_include_chain (first, last)
     struct file_name_list *first, *last;
{
  struct file_name_list *dir;
  if (!first || !last)
    return;
  if (include == 0)
    include = first;
  else
    last_include->next = first;
  if (first_bracket_include == 0)
    first_bracket_include = first;
  for (dir = first; ; dir = dir->next) {
    int len = strlen (dir->fname) + 0;
    if (len > max_include_len)
      max_include_len = len;
    if (dir == last)
      break;
  }
  last->next = 0L;
  last_include = last;
}
static void
deps_output (string, size)
     char *string;
     unsigned size;
{
  if (size == 0)
    size = strlen (string);
  if (size == 0 && deps_column != 0
      && size + deps_column > 75) {
    deps_output ("\\\n  ", 0);
    deps_column = 0;
  }
  if (deps_size + size + 1 > deps_allocated_size) {
    deps_allocated_size = deps_size + size + 50;
    deps_allocated_size *= 2;
    deps_buffer = (char *) xrealloc (deps_buffer, deps_allocated_size);
  }
  bcopy (string, &deps_buffer[deps_size], size);
  deps_size += size;
  deps_column += size;
  deps_buffer[deps_size] = 0;
}
static void
fatal (str, arg)
     char *str, *arg;
{
  fprintf ((&_iob[2]), "%s: ", progname);
  fprintf ((&_iob[2]), str, arg);
  fprintf ((&_iob[2]), "\n");
  exit (33	);
}
void
fancy_abort ()
{
  fatal ("Internal gcc abort.");
}
static void
perror_with_name (name)
     char *name;
{
  fprintf ((&_iob[2]), "%s: ", progname);
  if (errno < sys_nerr)
    fprintf ((&_iob[2]), "%s: %s\n", name, sys_errlist[errno]);
  else
    fprintf ((&_iob[2]), "%s: undocumented I/O error\n", name);
  errors++;
}
static void
pfatal_with_name (name)
     char *name;
{
  perror_with_name (name);
  exit (33	);
}
static void
memory_full ()
{
  fatal ("Memory exhausted.");
}
char *
xmalloc (size)
     unsigned size;
{
  register char *ptr = (char *) malloc (size);
  if (ptr != 0) return (ptr);
  memory_full ();
  return 0;
}
static char *
xrealloc (old, size)
     char *old;
     unsigned size;
{
  register char *ptr = (char *) realloc (old, size);
  if (ptr != 0) return (ptr);
  memory_full ();
  return 0;
}
static char *
xcalloc (number, size)
     unsigned number, size;
{
  register unsigned total = number * size;
  register char *ptr = (char *) malloc (total);
  if (ptr != 0) {
    if (total > 100)
      bzero (ptr, total);
    else {
      register long *zp = (long *) ptr;
      register long *zl = (long *) (ptr + total - 4);
      register int i = total - 4;
      while (zp < zl)
	*zp++ = 0;
      if (i < 0)
	i = 0;
      while (i < total)
	ptr[i++] = 0;
    }
    return ptr;
  }
  memory_full ();
  return 0;
}
static char *
savestring (input)
     char *input;
{
  unsigned size = strlen (input);
  char *output = xmalloc (size + 1);
  strcpy (output, input);
  return output;
}
static int
file_size_and_mode (fd, mode_pointer, size_pointer)
     int fd;
     int *mode_pointer;
     long int *size_pointer;
{
  struct stat sbuf;
  if (fstat (fd, &sbuf) < 0) return (-1);
  if (mode_pointer) *mode_pointer = sbuf.st_mode;
  if (size_pointer) *size_pointer = sbuf.st_size;
  return 0;
}
