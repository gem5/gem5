// I-cache-resident TPC classification and immutable dynamic fetch provenance.
#ifndef __MEM_CACHE_PREFETCH_INSTRUCTION_STATE_HH__
#define __MEM_CACHE_PREFETCH_INSTRUCTION_STATE_HH__
#include <array>
#include <memory>
#include <vector>
#include "mem/request.hh"
namespace gem5 {
class BaseCache;
namespace prefetch {
struct InstructionSource {
    const BaseCache *cache = nullptr;
    Addr line = 0;
    uint64_t generation = 0;
    unsigned offset = 0;
    bool secure = false;
    bool valid() const { return cache && generation; }
    bool sameLine(const InstructionSource &o) const {
        return cache == o.cache && line == o.line &&
               generation == o.generation && secure == o.secure;
    }
    bool operator==(const InstructionSource &o) const {
        return sameLine(o) && offset == o.offset;
    }
};
// One byte per possible x86 instruction start: T2 bits 0:1, P1 bits 2:3,
// C1 bits 4:5. The two spare bits are not algorithm state.
struct InstructionLineState {
    uint64_t generation = 0;
    std::vector<uint8_t> states;
    void initialize(uint64_t g, unsigned size) {
        generation = g; states.assign(size, 0);
    }
    bool matches(const InstructionSource &source) const {
        return source.valid() && source.generation == generation && source.offset < states.size();
    }
    int read(unsigned offset, int component) const {
        if (offset >= states.size() || component < 1 || component > 3) return -1;
        return (states[offset] >> ((component - 1) * 2)) & 3;
    }
    bool update(unsigned offset, int component, int expected, int state) {
        if (offset >= states.size() || component < 1 || component > 3 ||
            state < 0 || state > 3 || read(offset, component) != expected) return false;
        unsigned shift = (component - 1) * 2;
        states[offset] = (states[offset] & ~(3 << shift)) | (state << shift);
        return true;
    }
};
struct InstructionProvenance : public Extension<Request, InstructionProvenance> {
    InstructionSource source;
    explicit InstructionProvenance(InstructionSource s) : source(s) {}
    std::unique_ptr<ExtensionBase> clone() const override {
        return std::make_unique<InstructionProvenance>(source);
    }
};
inline InstructionSource instructionSource(const RequestPtr &req) {
    auto ext = req->getExtension<InstructionProvenance>();
    return ext ? ext->source : InstructionSource{};
}
inline void setInstructionSource(const RequestPtr &req, InstructionSource s) {
    req->setExtension(std::make_shared<InstructionProvenance>(s));
}
class InstructionStateObserver {
  public:
    virtual ~InstructionStateObserver() = default;
    virtual void instructionInvalidated(const InstructionSource &) = 0;
};
} // namespace prefetch
} // namespace gem5
#endif
