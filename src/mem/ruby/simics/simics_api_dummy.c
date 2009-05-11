#include <assert.h>

extern "C" {

typedef int generic_transaction_t;
typedef int generic_transaction;
typedef int la_t;
typedef int integer_t;
typedef int uint64;
typedef int attr_value_t;
typedef int data_or_instr_t;
typedef int sim_exception_t;
typedef int processor_t;
typedef int conf_object_t;
typedef int conf_object;
typedef int physical_address_t;
typedef int logical_address_t;
typedef int read_or_write_t;
typedef int interface_t;
typedef int set_error_t;
typedef int ireg_t;
typedef int pc_step_t;
typedef int event_handler_t;
typedef int lang_void;
typedef int cycles_t;
typedef int sync_t;
typedef int FILE;
typedef int va_list;
typedef int log_object;
typedef int hap_handle_t;
typedef int str_hap_func_t;
typedef int hap_type_t;
typedef int cb_func_em_t;
typedef int sync_t;

///////////////////////////////////////////////////////////////////////////////

void SIM_number_processors() { assert(0); return; };
void SIM_c_set_mem_op_value_buf(generic_transaction_t *mem_op, char *buf) { assert(0); return; };
void SIM_c_get_mem_op_value_buf(generic_transaction_t *mem_op, char *buf) { assert(0); return; };
sim_exception_t SIM_clear_exception(void) { assert(0); return 0; };
processor_t *SIM_conf_object_to_processor(conf_object_t* obj) { assert(0); return 0; };
processor_t *SIM_current_processor(void) { assert(0); return 0; };
const char *SIM_disassemble(processor_t *cpu_ptr, physical_address_t pa, int type) { assert(0); return 0; };
interface_t *SIM_get_interface(conf_object const *object, const char *interface_name) { assert(0); return 0; };
conf_object_t *SIM_get_object(const char *name) { assert(0); return 0; };
sim_exception_t SIM_get_pending_exception(void) { assert(0); return 0; };
int SIM_get_proc_no(const processor_t *cpu_ptr) { assert(0); return 0; };
la_t SIM_get_program_counter(processor_t *cpu) { assert(0); return 0; };
const char *SIM_last_error(void) { assert(0); return 0; };
physical_address_t SIM_logical_to_physical(conf_object *cpu_ptr, data_or_instr_t data_or_instr, logical_address_t address) { assert(0); return 0; };
const char * SIM_get_exception_name( processor_t * p, int exc ) { assert(0); return 0;};
processor_t *SIM_proc_no_2_ptr(int cpu_nr) { assert(0); return 0; };
conf_object_t *SIM_processor_to_conf_object(processor_t* p) { assert(0); return 0; };
ireg_t SIM_read_control_register(processor_t *cpu_ptr, int reg) { assert(0); return 0; };
double SIM_read_fp_register_d(processor_t *cpu_ptr, int reg) { assert(0); return 0; };
uint64 SIM_read_fp_register_x(processor_t *cpu_ptr, int reg) { assert(0); return 0; };
ireg_t SIM_read_global_register(processor_t *cpu_ptr, int globals, int reg) { assert(0); return 0; };
integer_t SIM_read_phys_memory(conf_object *cpu, physical_address_t address, int len) { assert(0); return 0; };
ireg_t SIM_read_window_register(processor_t *cpu_ptr, int window, int reg) { assert(0); return 0; };
set_error_t SIM_set_attribute(conf_object_t *object, char const *name, attr_value_t *value) { assert(0); return 0; };
void SIM_free_attribute(attr_value_t *value) { assert(0); };
void SIM_stall_cycle(conf_object_t *obj, cycles_t stall) { assert(0); return; };
cycles_t SIM_stall_count(conf_object_t *obj) { assert(0); return 0; };
void SIM_stall(conf_object_t *obj, cycles_t stall) { assert(0); return; };
pc_step_t SIM_step_count(conf_object_t *p) { assert(0); return 0; };
cycles_t SIM_cycle_count(conf_object_t *p) { assert(0); return 0; };
cycles_t SIM_stalled_until(conf_object_t *p) { assert(0); return 0; };
void SIM_time_clean(conf_object_t *obj, sync_t t, event_handler_t handler, lang_void * arg) { assert(0); return; };
void SIM_time_post_cycle(conf_object_t * obj, cycles_t delta, sync_t sync, event_handler_t handler, lang_void * arg) { assert(0); return; };
const char *SIM_version(void) { return 0; };
void SIM_set_program_counter(conf_object_t *cpu, logical_address_t pc){assert(0);};
void SIM_write_control_register(processor_t *cpu_ptr, int reg, ireg_t value) { assert(0); return; };
void SIM_write_fp_register_x(processor_t *cpu_ptr, int reg, uint64 value) { assert(0); return; };
void SIM_write_global_register(processor_t *cpu_ptr, int globals, int reg, ireg_t value) { assert(0); return; };
void SIM_write_window_register(processor_t *cpu_ptr, int window, int reg, ireg_t value) { assert(0); return; };
void SIM_write_phys_memory(conf_object *cpu, physical_address_t address, integer_t value, int len) { assert(0); };
int __sparc_v9_vtvfprintf(FILE *stream, const char *format, va_list va) { assert(0); return 0; };
int __l32_p32_vtvfprintf(FILE *stream, const char *format, va_list va) { assert(0); return 0; };
int __l32_p64_vtvfprintf(FILE *stream, const char *format, va_list va) { assert(0); return 0; };
int __l64_p64_vtvfprintf(FILE *stream, const char *format, va_list va) { assert(0); return 0; };
void __sparc_v9_vtdebug_log_vararg(int lvl, log_object *dev, char const *str, va_list va) { assert (0); return; };
hap_handle_t SIM_hap_add_callback(const char *id, str_hap_func_t cb, lang_void *data) { assert(0); return 0; };
hap_type_t SIM_hap_get_number(const char *id) { assert(0); return 0; };
void SIM_hap_delete_callback_id(hap_type_t hap, hap_handle_t hdl) { assert (0); return; };
int SIM_flush(void) { assert(0); return 0; };
void SIM_write_register(processor_t *cpu_ptr, int registerNumber, integer_t value){ assert(0); };
integer_t SIM_read_register(processor_t *cpu_ptr, int registerNumber) { assert(0); return 0; };
int SIM_get_register_number(processor_t *cpu_ptr, const char * register_name){ assert(0); return 0; }
const char * SIM_get_register_name(processor_t *cpu_ptr, int reg_num){ assert(0); return 0; }

void SIM_break_simulation(const char * msg){ assert(0); }
void SIM_printf(const char *format, va_list ap){ assert(0); }
set_error_t ruby_session_set( void *id, conf_object_t *obj,
                              attr_value_t *val, attr_value_t *idx ) { assert (0); return 0; };
attr_value_t ruby_session_get( void *id, conf_object_t *obj,
                               attr_value_t *idx ) { assert (0); return 0; };
  void SIM_stacked_post(conf_object_t *obj, event_handler_t, lang_void *arg){};
  pc_step_t SIM_step_next_occurrence( conf_object_t * obj, event_handler_t, lang_void * arg){ assert(0); return 0;};
void SIM_enable_processor(conf_object_t *p) { assert(0); };
void SIM_disable_processor(conf_object_t *p) { assert(0); };
int  SIM_cpu_enabled(conf_object_t *p)       { assert(0); };

attr_value_t SIM_get_attribute(conf_object_t *object, const char *name) { assert(0); };
} // extern "C"
