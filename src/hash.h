#pragma once

// hash.h -- one FNV-1a 64-bit mixer. Byte at a time, so a value never depends on
// endianness: that matters now that a hash is WRITTEN TO DISK (a save's part
// geometry fingerprint, #456) and not just lived inside one process (the staging
// table cache, staging.cpp).

#include <cstdint>
#include <cstring>   // memcpy, for a double's exact bits
#include <string>

namespace fnv {

inline uint64_t start() { return 1469598103934665603ull; }

// Eight mixes, low byte first.
inline void mix(uint64_t &h, uint64_t v) {
    for(int i = 0; i < 8; i++) {
        h = (h ^ ((v >> (i * 8)) & 0xffull)) * 1099511628211ull;
    }
}

// A double's exact bit pattern: the fingerprint moves when the number does, and
// -0.0 is not mixed as if it were 0.0.
inline void mixDouble(uint64_t &h, double d) {
    uint64_t bits;
    std::memcpy(&bits, &d, sizeof(bits));
    mix(h, bits);
}

// The length goes in first so "ab","c" and "a","bc" are not one byte stream.
inline void mixText(uint64_t &h, const std::string &s) {
    mix(h, (uint64_t)s.size());
    for(size_t i = 0; i < s.size(); i++) {
        h = (h ^ (uint64_t)(unsigned char)s[i]) * 1099511628211ull;
    }
}

}   // namespace fnv
