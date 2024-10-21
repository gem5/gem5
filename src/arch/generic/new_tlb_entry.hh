#include "base/types.hh"
class TLBEntry : public CacheEntry {

  public:
    TLBEntry() = default;
    ~TLBEntry() = default;

    Addr vaddr;
    Addr paddr;
    unsigned int logBytes; // supposed to be the page size
    Addr id;
    Addr tag;
    bool uncacheable;

    /* Functions that are used from CachEntry

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
    virtual void invalidate() {
        valid = false;
        setTag(MaxAddr);
    }

    */

}
