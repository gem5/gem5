
// *******************************************************************
// User-defined memory manager, which maintains a pool of transactions
// *******************************************************************

#include "tlm.h"

class mm: public tlm::tlm_mm_interface
{
  typedef tlm::tlm_generic_payload gp_t;

public:
  mm() : free_list(0), empties(0) {}

  virtual ~mm()
  {
    gp_t* ptr;

    while (free_list)
    {
      ptr = free_list->trans;

      // Delete generic payload and all extensions
      sc_assert(ptr);
      delete ptr;

      free_list = free_list->next;
    }

    while (empties)
    {
      access* x = empties;
      empties = empties->next;

      // Delete free list access struct
      delete x;
    }
  }

  gp_t* allocate();
  void  free(gp_t* trans);

private:
  struct access
  {
    gp_t* trans;
    access* next;
    access* prev;
  };

  access* free_list;
  access* empties;
};

mm::gp_t* mm::allocate()
{
  gp_t* ptr;
  if (free_list)
  {
    ptr = free_list->trans;
    empties = free_list;
    free_list = free_list->next;
  }
  else
  {
    ptr = new gp_t(this);
  }
  return ptr;
}

void mm::free(gp_t* trans)
{
  trans->reset(); // Delete auto extensions
  if (!empties)
  {
    empties = new access;
    empties->next = free_list;
    empties->prev = 0;
    if (free_list)
      free_list->prev = empties;
  }
  free_list = empties;
  free_list->trans = trans;
  empties = free_list->prev;
}
