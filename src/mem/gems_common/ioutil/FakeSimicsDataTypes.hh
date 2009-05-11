#ifndef FAKE_SIMICS_DATA_TYPES_H
#define FAKE_SIMICS_DATA_TYPES_H

typedef struct attr_value attr_value_t;

typedef enum {
  Sim_Val_Invalid = 0,
  Sim_Val_String = 1,
  Sim_Val_Integer = 2,
  Sim_Val_Floating = 3,
  Sim_Val_List = 4,
  Sim_Val_Data = 5,
  Sim_Val_Nil = 6,
  Sim_Val_Object = 7
} attr_kind_t;

typedef struct attr_list attr_list_t;

struct attr_list {
  int size;
  struct attr_value *vector;
};

struct attr_value {
  attr_kind_t kind;
  union {
    const char *string; /* Sim_Val_String */
    unsigned long long integer; /* Sim_Val_Integer */
    double floating; /* Sim_Val_Floating */
    void *object; /* Sim_Val_Object */
    attr_list_t list; /* Sim_Val_List */
  } u;
};

typedef enum {
Sim_Set_Ok,
Sim_Set_Need_Integer,
Sim_Set_Need_Floating,
Sim_Set_Need_String,
Sim_Set_Need_List,
Sim_Set_Need_Data,
Sim_Set_Need_Object,
Sim_Set_Object_Not_Found,
Sim_Set_Interface_Not_Found,
Sim_Set_Illegal_Value,
Sim_Set_Illegal_Type,
Sim_Set_Illegal_Index,
Sim_Set_Attribute_Not_Found,
Sim_Set_Not_Writable,
Sim_Set_Ignored
} set_error_t;


typedef attr_value_t (*get_attr_t)(void *ptr,
                                   void *obj,
                                   attr_value_t *idx);

typedef set_error_t (*set_attr_t)(void *ptr,
                                  void *obj,
                                  attr_value_t *val,
                                  attr_value_t *idx);

#endif // #ifndef FAKE_SIMICS_DATA_TYPES_H
