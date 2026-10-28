#pragma once

// hash.h -- one FNV-1a 64-bit mixer. Byte at a time, so a value never depends on
// endianness: that matters now that a hash is WRITTEN TO DISK (a save's part
// geometry fingerprint, #456) and not just lived inside one process (the staging
// table cache, staging.cpp).

#include <cstdint>
#include <string>

namespace fnv {

/* FNV-1a 64's offset basis, the canonical 14695981039346656037.
   Anything already hashed with a different start is stale by definition: the
   staging signature is a per-session cache key (nothing notices), and a part
   geometry fingerprint is stamped into saves, so changing this is exactly what
   shipdef.cpp's kGeomEpoch says a geometry-invalidator is for. Do not "fix" it
   quietly. */
inline uint64_t start() { return 14695981039346656037ull; }

// Eight mixes, low byte first.
inline void mix(uint64_t &h, uint64_t v) {
    for(int i = 0; i < 8; i++) {
        h = (h ^ ((v >> (i * 8)) & 0xffull)) * 1099511628211ull;
    }
}

/* A double's exact bytes, in address order: the same value on the same file
   means the same hash on any host, which mixDouble-by-way-of-uint64 would not
   give (it mixes the integer's low byte first, and that is the high address on
   a big-endian machine). Same order as `mix` on the little-endian hosts that
   write today's saves, so this changes no stored value. */
inline void mixDouble(uint64_t &h, double d) {
    const unsigned char *b = reinterpret_cast<const unsigned char *>(&d);
    for(size_t i = 0; i < sizeof(d); i++) {
        h = (h ^ (uint64_t)b[i]) * 1099511628211ull;
    }
}

// The length goes in first so "ab","c" and "a","bc" are not one byte stream.
inline void mixText(uint64_t &h, const std::string &s) {
    mix(h, (uint64_t)s.size());
    for(size_t i = 0; i < s.size(); i++) {
        h = (h ^ (uint64_t)(unsigned char)s[i]) * 1099511628211ull;
    }
}

}   // namespace fnv
