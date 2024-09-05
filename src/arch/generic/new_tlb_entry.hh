#include "base/types.hh"
class TLBEntry : public CacheEntry {

  public:
    TLBEntry() = default;
    ~TLBEntry() = default;

    // list of members, these are the most common ones that i see
    Addr vaddr;
    Addr paddr;
    unsigned int logBytes;
    Addr id; 
    Addr tag;
    bool uncacheable;
    
    virtual bool isValid() const {
        return valid;
    }

    virtual Addr getTag() const {
        return tag;
    }


    virtual bool matchTag(const Addr tag) const
    {
        return isValid() && (getTag() == tag);
    }

    virtual void insert(const Addr tag)
    {
        setValid();
        setTag(tag);
    }

    // this marks something as true or not
    virtual void invalidate() {
        valid = false;
        setTag(MaxAddr);
    }

}
