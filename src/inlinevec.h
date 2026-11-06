#pragma once
/* inlinevec.h -- a vector whose first N elements live inside the object.

   Why this exists (#668). The attitude authority rows (ActuatorRow plus the Part*
   that owns each one) are rebuilt on every actuation substep and again once per
   frame per open window, and the struct that carries them is a VALUE, so a
   std::vector inside it can never keep its buffer between calls: pre-sizing it
   with reserve() still means four fresh mallocs per walk. heaptrack 2026_11_06
   measured that as the flight frame's single biggest allocator -- 85,386
   allocations per 20 s of atmospheric flight, 65% of that run's whole
   steady-state delta, and still 24,216 after the reserves landed. A row is 32
   bytes and a shipped hull carries a handful of them, so the heap was being
   used to move ~100 bytes around a few hundred times a second.

   What this type promises:

   - The first N elements cost ZERO allocations, and the common hull never
     leaves that region. Overflow past N is allowed and honest: the vector moves
     to the heap and keeps growing, so a 200-part hull gets the same answer as a
     4-part one. There is no cap to silently truncate actuator rows and no
     "not enough room" branch for a law to get wrong.
   - reserve(n) for n <= N is free; for n > N it is exactly ONE allocation. That
     is why the authority walk still counts its rows first rather than letting
     push_back double its way there -- the count is what keeps a big hull at one
     malloc instead of a chain.
   - It stays a value: copy and move do the obvious thing, so AttitudeAuthority
     can go on being returned by value and held in a local. That matters: the
     rows are rebuilt per substep BY LAW (the comment above
     Vehicle::attitudeAuthorityRows, from #649's review pass -- cheaper rows,
     not a later read), and a value type is what lets that stay true without
     anyone holding a reference into a buffer that a later call refills.
   - T must be trivially copyable AND trivially destructible: this stores raw
     slots and memcpy's them. A row that grows a std::string fails to compile
     here rather than leaking at runtime.

   Invariant: `heap_ != nullptr` exactly when `cap_ > N`. Nothing ever shrinks
   cap_, so the two never drift; data() asserts it. */

#include <array>
#include <cassert>
#include <cstddef>
#include <cstring>
#include <new>
#include <type_traits>

template<typename T, size_t N>
class InlineVec {
    static_assert(std::is_trivially_copyable<T>::value,
                  "InlineVec memcpy's its slots; T must be trivially copyable");
    static_assert(std::is_trivially_destructible<T>::value,
                  "InlineVec never runs element destructors; T must be trivially destructible");
    /* ::operator new gives max_align_t; anything stricter needs aligned storage
       that this deliberately does not implement (glm vectors top out at 16). */
    static_assert(alignof(T) <= alignof(std::max_align_t),
                  "InlineVec over-aligns nothing; T must fit the default alignment");
    static_assert(N > 0, "an InlineVec with no inline room is just a vector");

public:
    using iterator = T *;
    using const_iterator = const T *;

    InlineVec() = default;

    InlineVec(const InlineVec &o) { copyFrom(o); }
    InlineVec &operator=(const InlineVec &o) {
        if(this != &o) { freeHeap(); copyFrom(o); }
        return *this;
    }
    InlineVec(InlineVec &&o) noexcept { steal(o); }
    InlineVec &operator=(InlineVec &&o) noexcept {
        if(this != &o) { freeHeap(); steal(o); }
        return *this;
    }
    ~InlineVec() { freeHeap(); }

    size_t size() const { return n_; }
    size_t capacity() const { return cap_; }
    bool empty() const { return n_ == 0; }

    T *data() { assert(!heap_ || cap_ > N); return heap_ ? heap_ : slot_.data(); }
    const T *data() const { assert(!heap_ || cap_ > N); return heap_ ? heap_ : slot_.data(); }
    T *begin() { return data(); }
    const T *begin() const { return data(); }
    T *end() { return data() + n_; }
    const T *end() const { return data() + n_; }
    T &operator[](size_t k) { return data()[k]; }
    const T &operator[](size_t k) const { return data()[k]; }

    void clear() { n_ = 0; }

    void reserve(size_t cap) { if(cap > cap_) { grow(cap); } }

    void push_back(const T &v) {
        if(n_ == cap_) { grow(cap_ * 2); }
        data()[n_++] = v;
    }

private:
    void grow(size_t cap) {
        T *p = static_cast<T *>(::operator new(cap * sizeof(T)));
        std::memcpy(p, data(), n_ * sizeof(T));
        freeHeap();                       // the inline slots are not freeable
        heap_ = p;
        cap_ = cap;
    }
    void freeHeap() { if(heap_ != nullptr) { ::operator delete(heap_); heap_ = nullptr; } }
    void copyFrom(const InlineVec &o) {
        n_ = o.n_;
        cap_ = o.cap_;
        if(o.heap_ != nullptr) {
            heap_ = static_cast<T *>(::operator new(o.cap_ * sizeof(T)));
            std::memcpy(heap_, o.heap_, o.n_ * sizeof(T));
        }
        else {
            /* The invariant says an inline source holds at most N rows, but that is
               an invariant, not something the compiler can see -- so copy through a
               loop bounded by N rather than a memcpy whose length it has to guess at
               (-Wstringop-overflow). Trivially copyable T, so it optimizes to the
               same block move. */
            heap_ = nullptr;
            for(size_t k = 0; k < N && k < n_; k++) { slot_[k] = o.slot_[k]; }
        }
        assert((heap_ != nullptr) == (cap_ > N));
    }
    /* A move of an inline vector is a block copy either way -- the point is that the
       source is left EMPTY, so it cannot double-free what it no longer owns. */
    void steal(InlineVec &o) {
        n_ = o.n_;
        cap_ = o.cap_;
        heap_ = o.heap_;
        if(o.heap_ == nullptr) {
            for(size_t k = 0; k < N && k < n_; k++) { slot_[k] = o.slot_[k]; }
        }
        o.heap_ = nullptr;
        o.n_ = 0;
        o.cap_ = N;
    }

    std::array<T, N> slot_{};
    T *heap_ = nullptr;
    size_t n_ = 0;
    size_t cap_ = N;
};
